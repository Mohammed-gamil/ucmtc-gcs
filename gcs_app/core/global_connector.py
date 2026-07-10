"""Global Connector – multi-peer ROS2 network bridge for team telemetry aggregation.

This module enables the GCS to discover and connect to one or more remote
team ROS2 systems (rovers, drones, base stations) over a shared network.

Connection strategies
---------------------
1. **UDP broadcast discovery** – peers announce themselves on a configurable
   broadcast port.  The connector listens for announcements and auto-registers
   new peers.
2. **Direct TCP endpoints** – manually configured IP:port pairs for known
   team members (useful behind NATs or VLANs).
3. **DDS domain ID bridging** – when both systems share a DDS transport, the
   connector can subscribe across a different ROS_DOMAIN_ID.

Architecture
------------
* `PeerInfo` – immutable snapshot of a discovered peer.
* `PeerConnection` – live connection state + buffered telemetry for one peer.
* `GlobalConnector` – the main engine: runs discovery, manages connections,
  and merges incoming telemetry into a shared buffer that the web server and
  Qt UI can read.
"""

from __future__ import annotations

import json
import os
import socket
import struct
import threading
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DISCOVERY_PORT = int(os.environ.get("GCS_DISCOVERY_PORT", "9876"))
DISCOVERY_MAGIC = b"UCMTC-GCS-PEER-v1"
ANNOUNCE_INTERVAL = 2.0          # seconds between outgoing announcements
PEER_TIMEOUT = 8.0               # mark peer dead after this many seconds
TCP_RECV_BUF = 65536
TCP_CONNECT_TIMEOUT = 3.0
TCP_RECONNECT_DELAY = 5.0
MAX_PEERS = 16


class PeerStatus(str, Enum):
    """Connection state machine for a single peer."""
    DISCOVERED = "discovered"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    STALE = "stale"
    DISCONNECTED = "disconnected"
    ERROR = "error"


@dataclass
class PeerInfo:
    """Immutable snapshot of a discovered peer."""
    peer_id: str
    hostname: str
    ip_address: str
    port: int
    ros_domain_id: int = 0
    role: str = "unknown"           # e.g. "rover", "drone", "base_station"
    team_name: str = ""
    discovery_method: str = "udp"   # udp | manual | dds


@dataclass
class PeerConnection:
    """Live connection state and telemetry buffer for one peer."""
    info: PeerInfo
    status: PeerStatus = PeerStatus.DISCOVERED
    last_seen: float = 0.0
    last_telemetry: dict[str, Any] | None = None
    last_telemetry_time: float = 0.0
    error_message: str = ""
    latency_ms: float = 0.0
    packets_received: int = 0
    packets_dropped: int = 0
    _socket: socket.socket | None = field(default=None, repr=False)

    def to_dict(self) -> dict[str, Any]:
        """Serialisable snapshot for the web frontend."""
        return {
            "peer_id": self.info.peer_id,
            "hostname": self.info.hostname,
            "ip_address": self.info.ip_address,
            "port": self.info.port,
            "ros_domain_id": self.info.ros_domain_id,
            "role": self.info.role,
            "team_name": self.info.team_name,
            "discovery_method": self.info.discovery_method,
            "status": self.status.value,
            "last_seen": self.last_seen,
            "last_telemetry_time": self.last_telemetry_time,
            "latency_ms": round(self.latency_ms, 1),
            "packets_received": self.packets_received,
            "packets_dropped": self.packets_dropped,
            "error_message": self.error_message,
            "has_telemetry": self.last_telemetry is not None,
        }


# ---------------------------------------------------------------------------
# UDP announcement protocol
# ---------------------------------------------------------------------------

def _build_announce_packet(
    peer_id: str,
    hostname: str,
    port: int,
    ros_domain_id: int,
    role: str,
    team_name: str,
) -> bytes:
    """Build a compact UDP discovery announcement."""
    body = json.dumps({
        "peer_id": peer_id,
        "hostname": hostname,
        "port": port,
        "ros_domain_id": ros_domain_id,
        "role": role,
        "team_name": team_name,
        "ts": time.time(),
    }, separators=(",", ":")).encode("utf-8")
    return DISCOVERY_MAGIC + body


def _parse_announce_packet(data: bytes, sender_ip: str) -> PeerInfo | None:
    """Parse an incoming UDP announcement; return None on invalid data."""
    if not data.startswith(DISCOVERY_MAGIC):
        return None
    try:
        payload = json.loads(data[len(DISCOVERY_MAGIC):].decode("utf-8"))
        return PeerInfo(
            peer_id=str(payload["peer_id"]),
            hostname=str(payload.get("hostname", "unknown")),
            ip_address=sender_ip,
            port=int(payload.get("port", 0)),
            ros_domain_id=int(payload.get("ros_domain_id", 0)),
            role=str(payload.get("role", "unknown")),
            team_name=str(payload.get("team_name", "")),
            discovery_method="udp",
        )
    except (json.JSONDecodeError, KeyError, ValueError):
        return None


# ---------------------------------------------------------------------------
# GlobalConnector
# ---------------------------------------------------------------------------

class GlobalConnector:
    """Network bridge that discovers and aggregates telemetry from team peers.

    Thread-safety: all public methods that touch ``_peers`` or ``_state``
    acquire ``_lock`` first.
    """

    def __init__(
        self,
        *,
        local_peer_id: str | None = None,
        local_port: int = 8090,
        ros_domain_id: int = 0,
        role: str = "ground_station",
        team_name: str = "UCMTC",
        enable_discovery: bool = True,
        enable_announcements: bool = True,
    ):
        self._local_peer_id = local_peer_id or f"gcs-{socket.gethostname()}"
        self._local_port = local_port
        self._ros_domain_id = ros_domain_id
        self._role = role
        self._team_name = team_name
        self._enable_discovery = enable_discovery
        self._enable_announcements = enable_announcements

        self._lock = threading.Lock()
        self._peers: dict[str, PeerConnection] = {}
        self._running = threading.Event()
        self._threads: list[threading.Thread] = []

        # TCP listener for incoming peer data streams
        self._tcp_server_sock: socket.socket | None = None

        # Local telemetry to share with peers (set by the aggregator)
        self._local_telemetry: dict[str, Any] | None = None
        self._local_telemetry_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Boot all background threads."""
        if self._running.is_set():
            return
        self._running.set()

        if self._enable_discovery:
            t = threading.Thread(target=self._discovery_listener, daemon=True, name="gcs-discovery-rx")
            self._threads.append(t)
            t.start()

        if self._enable_announcements:
            t = threading.Thread(target=self._announcer, daemon=True, name="gcs-announcer")
            self._threads.append(t)
            t.start()

        # TCP listener for incoming telemetry streams
        t = threading.Thread(target=self._tcp_listener, daemon=True, name="gcs-tcp-listener")
        self._threads.append(t)
        t.start()

        # Maintenance loop (timeouts, reconnects)
        t = threading.Thread(target=self._maintenance_loop, daemon=True, name="gcs-maintenance")
        self._threads.append(t)
        t.start()

        print(f"[GlobalConnector] Started – peer_id={self._local_peer_id}  "
              f"discovery_port={DISCOVERY_PORT}  tcp_port={self._local_port}")

    def stop(self) -> None:
        """Gracefully stop all threads."""
        self._running.clear()
        if self._tcp_server_sock:
            try:
                self._tcp_server_sock.close()
            except OSError:
                pass
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads.clear()

        with self._lock:
            for pc in self._peers.values():
                if pc._socket:
                    try:
                        pc._socket.close()
                    except OSError:
                        pass
            self._peers.clear()
        print("[GlobalConnector] Stopped")

    def add_manual_peer(self, peer_id: str, ip: str, port: int,
                        role: str = "rover", team_name: str = "",
                        ros_domain_id: int = 0) -> None:
        """Register a peer by explicit IP:port (no auto-discovery)."""
        info = PeerInfo(
            peer_id=peer_id,
            hostname=peer_id,
            ip_address=ip,
            port=port,
            ros_domain_id=ros_domain_id,
            role=role,
            team_name=team_name or self._team_name,
            discovery_method="manual",
        )
        with self._lock:
            if peer_id not in self._peers:
                self._peers[peer_id] = PeerConnection(info=info, last_seen=time.time())
                print(f"[GlobalConnector] Manual peer added: {peer_id} @ {ip}:{port}")
        # Start a connection thread for this peer
        t = threading.Thread(target=self._connect_to_peer, args=(peer_id,), daemon=True,
                             name=f"gcs-peer-{peer_id}")
        self._threads.append(t)
        t.start()

    def remove_peer(self, peer_id: str) -> bool:
        """Remove a peer by ID."""
        with self._lock:
            pc = self._peers.pop(peer_id, None)
        if pc and pc._socket:
            try:
                pc._socket.close()
            except OSError:
                pass
            return True
        return pc is not None

    def set_local_telemetry(self, telemetry: dict[str, Any]) -> None:
        """Update the local telemetry payload to relay to connected peers."""
        with self._local_telemetry_lock:
            self._local_telemetry = telemetry

    def get_peers_snapshot(self) -> list[dict[str, Any]]:
        """Return a JSON-serialisable list of all peer states."""
        with self._lock:
            return [pc.to_dict() for pc in self._peers.values()]

    def get_peer_telemetry(self, peer_id: str) -> dict[str, Any] | None:
        """Return the latest telemetry from a specific peer."""
        with self._lock:
            pc = self._peers.get(peer_id)
            return pc.last_telemetry if pc else None

    def get_all_peer_telemetry(self) -> dict[str, dict[str, Any]]:
        """Return {peer_id: telemetry_dict} for all peers with data."""
        with self._lock:
            return {
                pid: pc.last_telemetry
                for pid, pc in self._peers.items()
                if pc.last_telemetry is not None
            }

    def get_merged_telemetry(self) -> dict[str, Any]:
        """Build a merged view: local telemetry + all peer telemetries."""
        result: dict[str, Any] = {}
        with self._local_telemetry_lock:
            if self._local_telemetry:
                result["local"] = self._local_telemetry
        with self._lock:
            peers_data = {}
            for pid, pc in self._peers.items():
                peers_data[pid] = {
                    "status": pc.status.value,
                    "role": pc.info.role,
                    "team_name": pc.info.team_name,
                    "latency_ms": round(pc.latency_ms, 1),
                    "telemetry": pc.last_telemetry,
                }
            result["peers"] = peers_data
        result["peer_count"] = len(result.get("peers", {}))
        result["timestamp"] = time.time()
        return result

    @property
    def peer_count(self) -> int:
        with self._lock:
            return len(self._peers)

    @property
    def connected_count(self) -> int:
        with self._lock:
            return sum(1 for pc in self._peers.values()
                       if pc.status == PeerStatus.CONNECTED)

    # ------------------------------------------------------------------
    # UDP discovery
    # ------------------------------------------------------------------

    def _discovery_listener(self) -> None:
        """Listen for UDP broadcast announcements from peers."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except OSError:
            pass
        sock.settimeout(1.0)

        try:
            sock.bind(("", DISCOVERY_PORT))
        except OSError as exc:
            print(f"[GlobalConnector] Cannot bind discovery port {DISCOVERY_PORT}: {exc}")
            return

        print(f"[GlobalConnector] Discovery listener active on UDP :{DISCOVERY_PORT}")
        while self._running.is_set():
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            peer_info = _parse_announce_packet(data, addr[0])
            if peer_info is None or peer_info.peer_id == self._local_peer_id:
                continue

            with self._lock:
                if peer_info.peer_id in self._peers:
                    self._peers[peer_info.peer_id].last_seen = time.time()
                    if self._peers[peer_info.peer_id].status == PeerStatus.DISCONNECTED:
                        self._peers[peer_info.peer_id].status = PeerStatus.DISCOVERED
                elif len(self._peers) < MAX_PEERS:
                    self._peers[peer_info.peer_id] = PeerConnection(
                        info=peer_info,
                        last_seen=time.time(),
                    )
                    print(f"[GlobalConnector] Discovered peer: {peer_info.peer_id} "
                          f"@ {peer_info.ip_address}:{peer_info.port} role={peer_info.role}")
                    # Spawn connection thread
                    t = threading.Thread(
                        target=self._connect_to_peer,
                        args=(peer_info.peer_id,),
                        daemon=True,
                        name=f"gcs-peer-{peer_info.peer_id}",
                    )
                    self._threads.append(t)
                    t.start()
        sock.close()

    def _announcer(self) -> None:
        """Periodically broadcast our presence on the network."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        packet = _build_announce_packet(
            self._local_peer_id,
            socket.gethostname(),
            self._local_port,
            self._ros_domain_id,
            self._role,
            self._team_name,
        )

        while self._running.is_set():
            for target in ["255.255.255.255", "<broadcast>"]:
                try:
                    sock.sendto(packet, (target, DISCOVERY_PORT))
                except OSError:
                    pass
            time.sleep(ANNOUNCE_INTERVAL)
        sock.close()

    # ------------------------------------------------------------------
    # TCP peer connection
    # ------------------------------------------------------------------

    def _connect_to_peer(self, peer_id: str) -> None:
        """Try to establish a TCP data connection to a peer."""
        while self._running.is_set():
            with self._lock:
                pc = self._peers.get(peer_id)
                if pc is None:
                    return

            if pc.status == PeerStatus.CONNECTED:
                time.sleep(1.0)
                continue

            with self._lock:
                pc.status = PeerStatus.CONNECTING

            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(TCP_CONNECT_TIMEOUT)
                sock.connect((pc.info.ip_address, pc.info.port))
                sock.settimeout(5.0)

                # Send handshake
                handshake = json.dumps({
                    "type": "handshake",
                    "peer_id": self._local_peer_id,
                    "role": self._role,
                    "team_name": self._team_name,
                }).encode("utf-8") + b"\n"
                sock.sendall(handshake)

                with self._lock:
                    pc._socket = sock
                    pc.status = PeerStatus.CONNECTED
                    pc.last_seen = time.time()
                    pc.error_message = ""

                print(f"[GlobalConnector] Connected to peer: {peer_id}")
                self._receive_loop(peer_id, sock)

            except (OSError, ConnectionRefusedError, socket.timeout) as exc:
                with self._lock:
                    if pc:
                        pc.status = PeerStatus.ERROR
                        pc.error_message = str(exc)

            # Wait before retry
            for _ in range(int(TCP_RECONNECT_DELAY * 10)):
                if not self._running.is_set():
                    return
                time.sleep(0.1)

    def _receive_loop(self, peer_id: str, sock: socket.socket) -> None:
        """Continuously receive telemetry JSON lines from a peer."""
        buffer = b""
        while self._running.is_set():
            try:
                chunk = sock.recv(TCP_RECV_BUF)
                if not chunk:
                    break  # peer closed
                buffer += chunk

                if len(buffer) > 1_048_576:  # 1 MB safety cap
                    raise ValueError("Receive buffer size limit exceeded without newline")

                # Process complete lines (newline-delimited JSON)
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if not line.strip():
                        continue
                    self._process_peer_message(peer_id, line)

            except socket.timeout:
                continue
            except (OSError, ConnectionResetError):
                break

        # Connection lost
        with self._lock:
            pc = self._peers.get(peer_id)
            if pc:
                pc.status = PeerStatus.DISCONNECTED
                pc._socket = None
        try:
            sock.close()
        except OSError:
            pass
        print(f"[GlobalConnector] Disconnected from peer: {peer_id}")

    def _process_peer_message(self, peer_id: str, raw: bytes) -> None:
        """Parse and store a single message from a peer."""
        t_start = time.monotonic()
        try:
            msg = json.loads(raw.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            with self._lock:
                pc = self._peers.get(peer_id)
                if pc:
                    pc.packets_dropped += 1
            return

        msg_type = msg.get("type", "telemetry")
        with self._lock:
            pc = self._peers.get(peer_id)
            if pc is None:
                return

            pc.last_seen = time.time()
            pc.packets_received += 1

            if msg_type == "telemetry":
                pc.last_telemetry = msg.get("data", msg)
                pc.last_telemetry_time = time.time()
                # Approximate latency from peer timestamp
                peer_ts = msg.get("timestamp", 0)
                if peer_ts > 0:
                    pc.latency_ms = max(0, (time.time() - peer_ts) * 1000)
                else:
                    pc.latency_ms = (time.monotonic() - t_start) * 1000

    # ------------------------------------------------------------------
    # TCP listener (accept incoming connections from peers)
    # ------------------------------------------------------------------

    def _tcp_listener(self) -> None:
        """Accept inbound TCP connections from peers that connect to us."""
        self._tcp_server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp_server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._tcp_server_sock.bind(("0.0.0.0", self._local_port))
            self._tcp_server_sock.listen(MAX_PEERS)
            self._tcp_server_sock.settimeout(1.0)
        except OSError as exc:
            print(f"[GlobalConnector] Cannot bind TCP port {self._local_port}: {exc}")
            return

        print(f"[GlobalConnector] TCP listener active on :{self._local_port}")
        while self._running.is_set():
            try:
                client_sock, addr = self._tcp_server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            t = threading.Thread(
                target=self._handle_inbound_peer,
                args=(client_sock, addr),
                daemon=True,
                name=f"gcs-inbound-{addr[0]}",
            )
            self._threads.append(t)
            t.start()
        try:
            self._tcp_server_sock.close()
        except OSError:
            pass

    def _handle_inbound_peer(self, sock: socket.socket, addr: tuple) -> None:
        """Handle an inbound peer connection."""
        sock.settimeout(5.0)
        buffer = b""
        peer_id = f"inbound-{addr[0]}:{addr[1]}"

        # Wait for handshake
        try:
            data = sock.recv(4096)
            if data:
                try:
                    msg = json.loads(data.strip().decode("utf-8"))
                    if msg.get("type") == "handshake":
                        peer_id = msg.get("peer_id", peer_id)
                        info = PeerInfo(
                            peer_id=peer_id,
                            hostname=msg.get("hostname", peer_id),
                            ip_address=addr[0],
                            port=addr[1],
                            role=msg.get("role", "unknown"),
                            team_name=msg.get("team_name", ""),
                            discovery_method="inbound",
                        )
                        with self._lock:
                            if peer_id not in self._peers:
                                self._peers[peer_id] = PeerConnection(info=info)
                            self._peers[peer_id].status = PeerStatus.CONNECTED
                            self._peers[peer_id].last_seen = time.time()
                            self._peers[peer_id]._socket = sock
                        print(f"[GlobalConnector] Inbound peer connected: {peer_id} from {addr[0]}")
                except (json.JSONDecodeError, UnicodeDecodeError):
                    pass
        except (socket.timeout, OSError):
            sock.close()
            return

        # Now stream telemetry in and relay our telemetry out
        self._receive_loop(peer_id, sock)

    # ------------------------------------------------------------------
    # Maintenance
    # ------------------------------------------------------------------

    def _maintenance_loop(self) -> None:
        """Periodically check peer health and clean stale connections."""
        while self._running.is_set():
            now = time.time()
            with self._lock:
                for pid, pc in list(self._peers.items()):
                    if pc.status == PeerStatus.CONNECTED:
                        if (now - pc.last_seen) > PEER_TIMEOUT:
                            pc.status = PeerStatus.STALE
                            print(f"[GlobalConnector] Peer stale: {pid}")
                    elif pc.status == PeerStatus.STALE:
                        if (now - pc.last_seen) > PEER_TIMEOUT * 2:
                            pc.status = PeerStatus.DISCONNECTED
            time.sleep(2.0)

    # ------------------------------------------------------------------
    # Relay: send our telemetry to all connected peers
    # ------------------------------------------------------------------

    def relay_telemetry_to_peers(self) -> None:
        """Send our local telemetry to all connected peers (fire-and-forget)."""
        with self._local_telemetry_lock:
            telemetry = self._local_telemetry
        if telemetry is None:
            return

        payload = json.dumps({
            "type": "telemetry",
            "peer_id": self._local_peer_id,
            "data": telemetry,
            "timestamp": time.time(),
        }, separators=(",", ":")).encode("utf-8") + b"\n"

        with self._lock:
            for pid, pc in self._peers.items():
                if pc.status == PeerStatus.CONNECTED and pc._socket:
                    try:
                        pc._socket.sendall(payload)
                    except OSError:
                        pc.status = PeerStatus.DISCONNECTED
                        pc._socket = None


# ---------------------------------------------------------------------------
# Singleton instance for the application
# ---------------------------------------------------------------------------

_connector_instance: GlobalConnector | None = None
_connector_lock = threading.Lock()


def get_global_connector(**kwargs: Any) -> GlobalConnector:
    """Get or create the singleton GlobalConnector instance."""
    global _connector_instance
    with _connector_lock:
        if _connector_instance is None:
            _connector_instance = GlobalConnector(**kwargs)
        return _connector_instance


def shutdown_global_connector() -> None:
    """Stop and dispose the singleton."""
    global _connector_instance
    with _connector_lock:
        if _connector_instance is not None:
            _connector_instance.stop()
            _connector_instance = None


__all__ = [
    "GlobalConnector",
    "PeerConnection",
    "PeerInfo",
    "PeerStatus",
    "get_global_connector",
    "shutdown_global_connector",
]
