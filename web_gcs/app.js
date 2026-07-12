// app.js — UCMTC Tactical GCS

// ─── DOM Element Cache ───
let isTelemetryConnected = false;
let lastScanData = null;
let latestTelemetryPayload = null;
const rttLbl            = document.getElementById("rtt-lbl");
const batPctLbl         = document.getElementById("bat-pct-lbl");
const batBar            = document.getElementById("bat-bar");
const alertBanner       = document.getElementById("alerts-hub-banner");
const alertContent      = document.getElementById("alert-content");
const telemPulse        = document.getElementById("telem-pulse");
const signalStrength    = document.getElementById("signal-strength");
const btnTopEstop       = document.getElementById("btn-top-estop");

// Compass (new SVG needle group)
const compassNeedle  = document.getElementById("compass-needle-group");
const headingDialVal = document.getElementById("heading-dial-val");
const headingCardinal = document.getElementById("heading-cardinal");

// Navigation
const navSpeed    = document.getElementById("nav-speed");
const navSpeedLimit = document.getElementById("nav-speed-limit");
const navDistance = document.getElementById("nav-distance");
const navPosition = document.getElementById("nav-position");
const navWaypoint = document.getElementById("nav-waypoint");

// Safety
const safeEstopMech    = document.getElementById("safe-estop-mech");
const safeEstopWire    = document.getElementById("safe-estop-wire");
const safeEstopTrig    = document.getElementById("safe-estop-trig");
const safeBlocked      = document.getElementById("safe-blocked");
const safeTouched      = document.getElementById("safe-touched");
const safeCollision    = document.getElementById("safe-collision");
const safeBorderCrossed = document.getElementById("safe-border-crossed");
const safeBorderPartial = document.getElementById("safe-border-partial");

// Vision
const confDial      = document.getElementById("conf-dial");
const visionConfPct = document.getElementById("vision-conf-pct");
const visionStatusLbl = document.getElementById("vision-status-lbl");
const visionLane    = document.getElementById("vision-lane");
const visionObstacles = document.getElementById("vision-obstacles");
const visionLaser   = document.getElementById("vision-laser");
const visionFps     = document.getElementById("vision-fps");
const visionUptime  = document.getElementById("vision-uptime");

// Link
const linkRssi      = document.getElementById("link-rssi");
const linkLoss      = document.getElementById("link-loss");
const linkFps       = document.getElementById("link-fps");
const linkHeartbeat = document.getElementById("link-heartbeat");

// Controls
const ctrlSpeed    = document.getElementById("ctrl-speed");
const speedBubble  = document.getElementById("speed-bubble");
const ctrlHeading  = document.getElementById("ctrl-heading");
const headingBubble = document.getElementById("heading-bubble");
const ctrlThrottle = document.getElementById("ctrl-throttle");
const ctrlWaypoint = document.getElementById("ctrl-waypoint");
const commandResultLbl = document.getElementById("command-result-lbl");

// Mission & Payload
const missionPhaseVal = document.getElementById("mission-phase-val");
const payloadArmVal = document.getElementById("payload-arm-val");

// Header dots — updated IDs for new UI
const connectionStatusDot = document.getElementById("telem-dot");
const safetyStatusDot     = document.getElementById("safety-dot");

// Team Mesh
const meshDot      = document.getElementById("mesh-dot");
const meshCountLbl = document.getElementById("mesh-count-lbl");
const meshPill     = document.getElementById("mesh-pill");
const peerList     = document.getElementById("peer-list");
const peerLiveCount = document.getElementById("peer-live-count");
const peerAddResult = document.getElementById("peer-add-result");

// Slide bubbles logic
ctrlSpeed.addEventListener("input", (e) => {
    speedBubble.textContent = parseFloat(e.target.value).toFixed(1);
});
ctrlHeading.addEventListener("input", (e) => {
    headingBubble.textContent = parseInt(e.target.value) + "°";
});

// Real-time metrics tracking
const maxDataPoints = 30;
const cpuData = Array(maxDataPoints).fill(0);
const tempData = Array(maxDataPoints).fill(0);
const lossData = Array(maxDataPoints).fill(0);

// Cardinal headings translator
function getCardinalDirection(deg) {
    deg = deg % 360;
    const directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"];
    const idx = Math.floor((deg + 11.25) / 22.5) % 16;
    return directions[idx];
}

// Role icon mapping
function getRoleIcon(role) {
    const icons = {
        "rover": "fa-robot",
        "drone": "fa-helicopter",
        "base_station": "fa-tower-broadcast",
        "ground_station": "fa-satellite-dish",
        "sensor_hub": "fa-microchip",
        "unknown": "fa-question"
    };
    return icons[role] || icons["unknown"];
}

// Status display label
function getStatusLabel(status) {
    const labels = {
        "connected": "ONLINE",
        "connecting": "SYNCING",
        "discovered": "FOUND",
        "stale": "STALE",
        "disconnected": "OFFLINE",
        "error": "ERROR"
    };
    return labels[status] || status.toUpperCase();
}

// Time ago formatting
function timeAgo(timestamp) {
    if (!timestamp || timestamp === 0) return "never";
    const secs = Math.floor(Date.now() / 1000 - timestamp);
    if (secs < 5) return "just now";
    if (secs < 60) return secs + "s ago";
    if (secs < 3600) return Math.floor(secs / 60) + "m ago";
    return Math.floor(secs / 3600) + "h ago";
}

// ═══════════════════════════════════════════════════
// PEER LIST RENDERING
// ═══════════════════════════════════════════════════

let previousPeers = [];
let previousPeersJson = "";

function renderPeerList(peers) {
    const peersJson = JSON.stringify(peers);
    if (peersJson === previousPeersJson) return;
    previousPeersJson = peersJson;
    const connectedCount = peers ? peers.filter(p => p.status === "connected").length : 0;
    const totalCount = peers ? peers.length : 0;

    // Update setup tab mesh info
    const setupMeshStatus = document.getElementById("setup-mesh-status");
    const setupMeshConnections = document.getElementById("setup-mesh-connections-lbl");
    if (setupMeshStatus) {
        if (connectedCount > 0) {
            setupMeshStatus.textContent = "ACTIVE";
            setupMeshStatus.className = "dv ok";
        } else {
            setupMeshStatus.textContent = "OFFLINE";
            setupMeshStatus.className = "dv err";
        }
    }
    if (setupMeshConnections) {
        setupMeshConnections.textContent = `${connectedCount} active peer${connectedCount !== 1 ? 's' : ''} (${totalCount} total)`;
    }

    // Track joins/leaves for mesh announcer logs
    const safePeers = peers || [];
    if (previousPeers.length !== safePeers.length) {
        for (const peer of safePeers) {
            const wasPresent = previousPeers.some(p => p.peer_id === peer.peer_id);
            if (!wasPresent) {
                appendMeshLog(`[Mesh] Discovered new peer: ${peer.peer_id} (${peer.ip_address}:${peer.port}) as ${peer.role}`);
            }
        }
        for (const prev of previousPeers) {
            const isPresent = safePeers.some(p => p.peer_id === prev.peer_id);
            if (!isPresent) {
                appendMeshLog(`[Mesh] Lost connection to peer: ${prev.peer_id}`);
            }
        }
    }
    previousPeers = JSON.parse(JSON.stringify(safePeers));

    if (!peers || peers.length === 0) {
        peerList.innerHTML = `
            <div class="peer-empty">
                <i class="fa-solid fa-satellite-dish"></i>
                <p>No peers discovered. Add a manual peer or enable network discovery.</p>
            </div>
        `;
        peerLiveCount.textContent = "0 / 0";
        meshDot.className = "chip-dot d-offline";
        // Guard meshCountLbl since it might be null
        if (meshCountLbl) {
            meshCountLbl.textContent = "0 PEERS";
        }
        if (meshPill) { meshPill.textContent = "DISCOVERY"; }
        return;
    }



    peerLiveCount.textContent = `${connectedCount} / ${totalCount}`;
    if (meshCountLbl) {
        meshCountLbl.textContent = `${connectedCount} PEER${connectedCount !== 1 ? "S" : ""}`;
    }

    if (connectedCount > 0) {
        meshDot.className = "chip-dot d-mesh";
        if (meshPill) { meshPill.textContent = `${connectedCount} LIVE`; }
    } else {
        meshDot.className = "chip-dot d-idle";
        if (meshPill) { meshPill.textContent = "SCANNING"; }
    }

    // Build peer cards
    let html = "";
    for (const peer of peers) {
        const roleIcon    = getRoleIcon(peer.role);
        const statusLabel = getStatusLabel(peer.status);
        const lastSeen    = timeAgo(peer.last_seen);
        const latency     = peer.latency_ms > 0 ? peer.latency_ms.toFixed(0) + " ms" : "—";
        const packets     = peer.packets_received || 0;

        html += `
            <div class="peer-card peer-${peer.status}" data-peer-id="${peer.peer_id}">
                <div class="peer-avatar">
                    <i class="fa-solid ${roleIcon}"></i>
                </div>
                <div class="peer-info">
                    <div class="peer-name">
                        ${escapeHtml(peer.peer_id)}
                        <span class="peer-role-badge role-${peer.role}">${peer.role.replace("_", " ")}</span>
                    </div>
                    <div class="peer-meta">
                        <span><i class="fa-solid fa-network-wired"></i> ${escapeHtml(peer.ip_address)}:${peer.port}</span>
                        <span><i class="fa-solid fa-clock"></i> ${lastSeen}</span>
                        ${peer.team_name ? `<span><i class="fa-solid fa-users"></i> ${escapeHtml(peer.team_name)}</span>` : ""}
                    </div>
                </div>
                <div class="peer-right">
                    <span class="peer-status-tag status-${peer.status}">${statusLabel}</span>
                    <span class="peer-latency">${latency} · ${packets} pkts</span>
                </div>
                <button class="peer-remove" onclick="removePeer('${escapeHtml(peer.peer_id)}')" title="Disconnect peer">
                    <i class="fa-solid fa-xmark"></i>
                </button>
            </div>
        `;
    }
    peerList.innerHTML = html;
}

function escapeHtml(text) {
    const div = document.createElement("div");
    div.textContent = text;
    return div.innerHTML;
}

// ═══════════════════════════════════════════════════
// PEER MANAGEMENT ACTIONS
// ═══════════════════════════════════════════════════

document.getElementById("btn-add-peer").addEventListener("click", async () => {
    const peerId = document.getElementById("peer-id-input").value.trim();
    const ipAddress = document.getElementById("peer-ip-input").value.trim();
    const port = parseInt(document.getElementById("peer-port-input").value) || 8090;
    const role = document.getElementById("peer-role-select").value;
    const teamName = document.getElementById("peer-team-input").value.trim();

    if (!peerId || !ipAddress) {
        peerAddResult.textContent = "⚠️ Peer ID and IP Address are required";
        peerAddResult.style.color = "var(--red-alert)";
        return;
    }

    peerAddResult.textContent = "⏳ Connecting to peer...";
    peerAddResult.style.color = "var(--cyan)";

    try {
        const response = await fetch("/api/peers", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
                peer_id: peerId,
                ip_address: ipAddress,
                port: port,
                role: role,
                team_name: teamName,
            })
        });
        const result = await response.json();

        if (result.status === "success") {
            peerAddResult.textContent = `✅ ${result.message}`;
            peerAddResult.style.color = "var(--green-op)";
            // Clear form
            document.getElementById("peer-id-input").value = "";
            document.getElementById("peer-ip-input").value = "";
            document.getElementById("peer-port-input").value = "8090";
            document.getElementById("peer-team-input").value = "";
        } else {
            peerAddResult.textContent = `❌ ${result.message}`;
            peerAddResult.style.color = "var(--red-alert)";
        }
    } catch (err) {
        peerAddResult.textContent = `❌ Network error: ${err.message}`;
        peerAddResult.style.color = "var(--red-alert)";
    }
});

async function removePeer(peerId) {
    try {
        const response = await fetch(`/api/peers/${encodeURIComponent(peerId)}`, {
            method: "DELETE",
        });
        const result = await response.json();
        if (result.status === "success") {
            peerAddResult.textContent = `🗑️ ${result.message}`;
            peerAddResult.style.color = "var(--amber)";
        }
    } catch (err) {
        console.error("Failed to remove peer:", err);
    }
}

// Update DOM elements from telemetry payload
// --- Live-Data-Only Rendering Rule Settings ---
// Topic: /imu (500ms), /odom (500ms), /fix (2000ms), /battery_state (2000ms), /cmd_vel (1000ms), /scan (1000ms)
const FRESHNESS_WINDOWS = {
    imu: 500,
    odom: 500,
    gps: 2000,
    battery: 2000,
    cmdVel: 1000,
    scan: 1000,
    safety: 1000,
    jetson: 2000,
    comm: 2000,
    ros: 2000,
    mission: 2000,
    payload_sys: 2000
};

let lastReceivedTime = {
    imu: null,
    odom: null,
    gps: null,
    battery: null,
    cmdVel: null,
    scan: null,
    safety: null,
    jetson: null,
    comm: null,
    ros: null,
    mission: null,
    payload_sys: null
};

// Quaternion conversion
function quaternionToEuler(x, y, z, w) {
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);

    const sinp = 2 * (w * y - z * x);
    let pitch;
    if (Math.abs(sinp) >= 1) {
        pitch = Math.sign(sinp) * Math.PI / 2;
    } else {
        pitch = Math.asin(sinp);
    }

    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    return {
        roll: roll * 180 / Math.PI,
        pitch: pitch * 180 / Math.PI,
        yaw: yaw * 180 / Math.PI
    };
}

// Update DOM elements from telemetry payload
function updateDashboard(payload, connected, peers, simulation_mode = false) {
    if (!connected || !payload) {
        isTelemetryConnected = false;
        if (connectionStatusDot) connectionStatusDot.className = "chip-dot d-offline";
        safetyStatusDot.className = "chip-dot d-offline";
        alertBanner.className = "alert-banner alert-mode";
        alertContent.innerHTML = "🔴 CRITICAL: TELEMETRY DISCONNECTED — Heartbeat stream offline!";
        if (telemPulse) telemPulse.className = "telemetry-pulse disconnected";
        if (signalStrength) signalStrength.className = "signal-bars";
        lastScanData = null;
        latestTelemetryPayload = null;

        // Push all telemetry to NO SIGNAL immediately on disconnection
        for (let key in lastReceivedTime) {
            lastReceivedTime[key] = null;
        }
        return;
    }

    // Toggle simulation mode banner
    const simBanner = document.getElementById("sim-banner");
    if (simBanner) {
        simBanner.style.display = simulation_mode ? "block" : "none";
    }

    // Normal connection
    isTelemetryConnected = true;
    connectionStatusDot.className = "chip-dot d-online";
    if (telemPulse) telemPulse.className = "telemetry-pulse";

    latestTelemetryPayload = payload;

    // Record reception times
    const now = Date.now();

    // 1. IMU
    if (payload.Sensors && payload.Sensors.imu && payload.Sensors.imu.available) {
        lastReceivedTime.imu = now;
    }
    // 2. Odom
    if (payload.Odom && payload.Odom.available) {
        lastReceivedTime.odom = now;
    }
    // 3. GPS
    if (payload.GPS && payload.GPS.available) {
        lastReceivedTime.gps = now;
    }
    // 4. Battery
    if (payload.Battery && payload.Battery.available) {
        lastReceivedTime.battery = now;
    }
    // 5. CmdVel Echo
    if (payload.CmdVelEcho) {
        lastReceivedTime.cmdVel = now;
    }
    // 6. LiDAR Scan
    if (payload.Sensors && payload.Sensors.scan) {
        lastScanData = payload.Sensors.scan;
        lastReceivedTime.scan = now;
    } else {
        lastScanData = null;
    }
    // 7. Safety
    if (payload.Safety && Object.keys(payload.Safety).length > 0) {
        lastReceivedTime.safety = now;
    }
    // 8. Jetson
    if (payload.Jetson && Object.keys(payload.Jetson).length > 0) {
        lastReceivedTime.jetson = now;
    }
    // 9. Communication
    if (payload.Communication && Object.keys(payload.Communication).length > 0) {
        lastReceivedTime.comm = now;
    }
    // 10. ROS
    if (payload.ROS && Object.keys(payload.ROS).length > 0) {
        lastReceivedTime.ros = now;
    }
    // 11. Mission
    if (payload.Mission && Object.keys(payload.Mission).length > 0) {
        lastReceivedTime.mission = now;
    }
    // 12. Payload
    if (payload.Payload && Object.keys(payload.Payload).length > 0) {
        lastReceivedTime.payload_sys = now;
    }

    const nav    = payload.Navigation  || {};
    const safety = payload.Safety      || {};
    const comm   = payload.Communication || {};
    const jetson = payload.Jetson      || {};
    const ros    = payload.ROS         || {};

    // Chart.js updates (guard)
    if (jetson.cpu_pct !== undefined) { cpuData.push(jetson.cpu_pct); cpuData.shift(); }
    if (jetson.temp_c !== undefined)  { tempData.push(jetson.temp_c); tempData.shift(); }
    if (comm.packet_loss_pct !== undefined) { lossData.push(comm.packet_loss_pct * 10); lossData.shift(); }

    // Alert analysis (guard fields)
    const warnings = [];
    if (safety.estop_triggered) warnings.push("EMERGENCY STOP TRIGGERED");
    if (safety.collision_detected) warnings.push("COLLISION DETECTED");
    if (safety.border_crossed) warnings.push("GEOFENCE BORDER BREACHED");
    if (jetson.temp_c > 75) warnings.push(`CPU TEMP CRITICAL (${jetson.temp_c}°C)`);
    if (jetson.bat_pct < 15) warnings.push("LOW BATTERY DETECTED");
    if (comm.packet_loss_pct > 8) warnings.push(`HIGH PACKET LOSS (${comm.packet_loss_pct}%)`);
    if (comm.rtt_ms > 150) warnings.push("HIGH LATENCY DELAY");

    const deadNodes = [];
    if (ros.node_lane_det  === false) deadNodes.push("Lane");
    if (ros.node_obs_avoid === false) deadNodes.push("Safety");
    if (ros.node_wp_nav    === false) deadNodes.push("Nav");
    if (ros.node_motor_ctrl=== false) deadNodes.push("Motor");
    if (deadNodes.length > 0) warnings.push(`ROS NODE FAIL (${deadNodes.join(",")})`);

    if (warnings.length > 0) {
        alertBanner.className = "alert-banner alert-mode";
        alertContent.innerHTML = "⚠️ ALERT: " + warnings.join(" | ");
    } else {
        alertBanner.className = "alert-banner";
        alertContent.innerHTML = "🟢 SYSTEM NOMINAL — All metrics within safe operating parameters";
    }

    // Update peer list from SSE data
    if (peers) {
        renderPeerList(peers);
    }
}

// Helper to evaluate state
function getTopicState(topicKey) {
    if (!isTelemetryConnected || lastReceivedTime[topicKey] === null) {
        return { state: "NO_SIGNAL", elapsedSec: 0 };
    }
    const elapsed = Date.now() - lastReceivedTime[topicKey];
    if (elapsed > FRESHNESS_WINDOWS[topicKey]) {
        return { state: "STALE", elapsedSec: Math.floor(elapsed / 1000) };
    }
    return { state: "LIVE", elapsedSec: 0 };
}

// Render loop for live telemetry updates and state visual styling
function renderAllTelemetry() {
    const now = Date.now();

    function applyStateStyles(element, state, elapsedSec) {
        if (!element) return;
        if (state === "NO_SIGNAL") {
            element.classList.add("metric-no-signal");
            element.classList.remove("metric-stale", "metric-live");
            element.title = "No signal received this session";
        } else if (state === "STALE") {
            element.classList.add("metric-stale");
            element.classList.remove("metric-no-signal", "metric-live");
            element.title = `Stale data: Last updated ${elapsedSec}s ago`;
        } else {
            element.classList.add("metric-live");
            element.classList.remove("metric-no-signal", "metric-stale");
            element.title = "Live telemetry";
        }
    }

    const payload = latestTelemetryPayload || {};
    const nav    = payload.Navigation  || {};
    const safety = payload.Safety      || {};
    const vision = payload.Vision      || {};
    const jetson = payload.Jetson      || {};
    const comm   = payload.Communication || {};
    const ros    = payload.ROS         || {};

    // 1. Comm / RTT
    const commState = getTopicState("comm");
    if (rttLbl) {
        if (commState.state === "NO_SIGNAL") {
            rttLbl.textContent = "-- ms";
        } else {
            rttLbl.textContent = (comm.rtt_ms !== undefined ? comm.rtt_ms : "--") + " ms";
        }
        applyStateStyles(rttLbl, commState.state, commState.elapsedSec);
    }
    if (linkRssi) {
        if (commState.state === "NO_SIGNAL") {
            linkRssi.textContent = "NO SIGNAL";
            linkRssi.className = "dv";
        } else {
            linkRssi.textContent = (comm.channel_rssi !== undefined ? comm.channel_rssi : "--") + " dBm";
            linkRssi.className = comm.channel_rssi < -75 ? "dv rssi-val err" : "dv rssi-val ok";
        }
        applyStateStyles(linkRssi, commState.state, commState.elapsedSec);
    }
    if (linkLoss) {
        if (commState.state === "NO_SIGNAL") {
            linkLoss.textContent = "-- %";
        } else {
            linkLoss.textContent = (comm.packet_loss_pct !== undefined ? comm.packet_loss_pct.toFixed(1) : "--") + " %";
        }
        applyStateStyles(linkLoss, commState.state, commState.elapsedSec);
    }
    if (linkFps) {
        if (commState.state === "NO_SIGNAL") {
            linkFps.textContent = "-- FPS";
        } else {
            linkFps.textContent = (comm.stream_fps !== undefined ? comm.stream_fps.toFixed(1) : "--") + " FPS";
        }
        applyStateStyles(linkFps, commState.state, commState.elapsedSec);
    }
    if (linkHeartbeat) {
        if (commState.state === "NO_SIGNAL") {
            linkHeartbeat.textContent = "--";
        } else {
            linkHeartbeat.textContent = comm.heartbeat_seq !== undefined ? "#" + comm.heartbeat_seq : "--";
        }
        applyStateStyles(linkHeartbeat, commState.state, commState.elapsedSec);
    }
    if (signalStrength) {
        if (commState.state === "NO_SIGNAL" || comm.channel_rssi === undefined) {
            signalStrength.className = "signal-bars";
        } else {
            if (comm.channel_rssi >= -60) {
                signalStrength.className = "signal-bars signal-good";
            } else if (comm.channel_rssi >= -75) {
                signalStrength.className = "signal-bars signal-fair";
            } else {
                signalStrength.className = "signal-bars signal-weak";
            }
        }
        applyStateStyles(signalStrength, commState.state, commState.elapsedSec);
    }

    // 2. Battery
    const batState = getTopicState("battery");
    let batPct = payload.Battery && payload.Battery.available ? payload.Battery.percentage : (jetson.bat_pct !== undefined ? jetson.bat_pct : undefined);
    let batVolt = payload.Battery && payload.Battery.available ? payload.Battery.voltage : (jetson.bat_voltage !== undefined ? jetson.bat_voltage : undefined);
    if (batPctLbl && batBar) {
        if (batState.state === "NO_SIGNAL" || batPct === undefined) {
            batPctLbl.textContent = "NO SIGNAL";
            batBar.style.width = "0%";
            batBar.style.backgroundColor = "var(--border-dim)";
        } else {
            batPctLbl.textContent = batPct.toFixed(1) + "%" + (batVolt !== undefined ? ` (${batVolt.toFixed(1)}V)` : "");
            batBar.style.width = batPct + "%";
            if (batPct < 20) {
                batBar.style.backgroundColor = "var(--red-alert)";
            } else if (batPct < 50) {
                batBar.style.backgroundColor = "var(--amber)";
            } else {
                batBar.style.backgroundColor = "var(--green-op)";
            }
        }
        applyStateStyles(batPctLbl, batState.state, batState.elapsedSec);
        applyStateStyles(batBar, batState.state, batState.elapsedSec);
    }

    // 3. IMU
    const imuState = getTopicState("imu");
    const headingState = (payload.Sensors && payload.Sensors.imu && payload.Sensors.imu.available) ? imuState : commState;
    let heading = nav.heading_deg;
    if (compassNeedle) {
        if (headingState.state === "NO_SIGNAL" || heading === undefined) {
            compassNeedle.style.transform = `rotate(0deg)`;
        } else {
            compassNeedle.style.transform = `rotate(${heading}deg)`;
        }
        applyStateStyles(compassNeedle, headingState.state, headingState.elapsedSec);
    }
    if (headingDialVal && headingCardinal) {
        if (headingState.state === "NO_SIGNAL" || heading === undefined) {
            headingDialVal.textContent = "NO SIGNAL";
            headingCardinal.textContent = "--";
        } else {
            headingDialVal.textContent = heading.toFixed(1) + "°";
            headingCardinal.textContent = getCardinalDirection(heading);
        }
        applyStateStyles(headingDialVal, headingState.state, headingState.elapsedSec);
        applyStateStyles(headingCardinal, headingState.state, headingState.elapsedSec);
    }

    // 4. Odom
    const odomState = getTopicState("odom");
    const speedState = (payload.Odom && payload.Odom.available) ? odomState : commState;
    let speed = payload.Odom && payload.Odom.available ? payload.Odom.speed_kmh : (nav.speed_kmh !== undefined ? nav.speed_kmh : undefined);
    let odomBadge = payload.Odom && payload.Odom.available ? " (ODOM)" : "";
    if (navSpeed) {
        if (speedState.state === "NO_SIGNAL" || speed === undefined) {
            navSpeed.textContent = "NO SIGNAL";
        } else {
            navSpeed.textContent = speed.toFixed(2) + " km/h" + odomBadge;
        }
        applyStateStyles(navSpeed, speedState.state, speedState.elapsedSec);
    }
    if (navSpeedLimit) {
        if (commState.state === "NO_SIGNAL" || nav.speed_limit === undefined) {
            navSpeedLimit.textContent = "-- km/h";
        } else {
            navSpeedLimit.textContent = nav.speed_limit.toFixed(1) + " km/h";
        }
        applyStateStyles(navSpeedLimit, commState.state, commState.elapsedSec);
    }
    if (navDistance) {
        const distState = (payload.Odom && payload.Odom.available) ? odomState : commState;
        if (distState.state === "NO_SIGNAL" || nav.dist_traveled_m === undefined) {
            navDistance.textContent = "NO SIGNAL";
        } else {
            navDistance.textContent = nav.dist_traveled_m.toFixed(1) + " m";
        }
        applyStateStyles(navDistance, distState.state, distState.elapsedSec);
    }

    // 5. GPS
    const gpsState = getTopicState("gps");
    const positionState = (payload.GPS && payload.GPS.available) ? gpsState : commState;
    let lat = payload.GPS && payload.GPS.available ? payload.GPS.latitude : (nav.pos_lat !== undefined ? nav.pos_lat : undefined);
    let lon = payload.GPS && payload.GPS.available ? payload.GPS.longitude : (nav.pos_lon !== undefined ? nav.pos_lon : undefined);
    let gpsBadge = payload.GPS && payload.GPS.available ? " (FIX)" : "";
    if (navPosition) {
        if (positionState.state === "NO_SIGNAL" || lat === undefined || lon === undefined) {
            navPosition.textContent = "NO SIGNAL";
        } else {
            navPosition.textContent = `${lat.toFixed(6)}, ${lon.toFixed(6)}${gpsBadge}`;
        }
        applyStateStyles(navPosition, positionState.state, positionState.elapsedSec);
    }

    // 6. Waypoint Nav
    if (navWaypoint) {
        if (commState.state === "NO_SIGNAL" || nav.wp_current === undefined) {
            navWaypoint.textContent = "NO SIGNAL";
        } else {
            navWaypoint.textContent = `WP ${nav.wp_current} (${(nav.wp_status||'').toUpperCase()}) (err: ${(nav.wp_error_m||0).toFixed(2)}m)`;
        }
        applyStateStyles(navWaypoint, commState.state, commState.elapsedSec);
    }

    // 6b. Mission Phase
    const missionPhaseLbl = document.getElementById("nav-mission-phase");
    if (missionPhaseLbl) {
        if (commState.state === "NO_SIGNAL" || nav.mission_phase === undefined) {
            missionPhaseLbl.textContent = "NO SIGNAL";
        } else {
            missionPhaseLbl.textContent = nav.mission_phase;
        }
        applyStateStyles(missionPhaseLbl, commState.state, commState.elapsedSec);
    }

    // 6c. Arm Status
    const armStatusLbl = document.getElementById("nav-arm-status");
    if (armStatusLbl) {
        if (commState.state === "NO_SIGNAL" || safety.arm_status === undefined) {
            armStatusLbl.textContent = "NO SIGNAL";
        } else {
            armStatusLbl.textContent = safety.arm_status;
        }
        applyStateStyles(armStatusLbl, commState.state, commState.elapsedSec);
    }

    // 7. Safety / Diagnostics
    const safetyState = getTopicState("safety");
    if (safetyStatusDot) {
        if (safetyState.state === "NO_SIGNAL") {
            safetyStatusDot.className = "chip-dot d-offline";
        } else {
            if (safety.estop_triggered || safety.collision_detected) {
                safetyStatusDot.className = "chip-dot d-offline";
            } else if (safety.mode) {
                safetyStatusDot.className = `chip-dot ${safety.mode === 'monitoring' ? 'd-online' : 'd-idle'}`;
            }
        }
    }

    const setBoolCell = (elem, val, invert = false) => {
        if (!elem) return;
        if (safetyState.state === "NO_SIGNAL" || val === undefined) {
            elem.textContent = "NO SIGNAL";
            elem.className = "dv";
            applyStateStyles(elem, "NO_SIGNAL");
            return;
        }
        const check = invert ? !val : val;
        if (check) {
            elem.textContent = "CLEAR";
            elem.className = "dv ok";
        } else {
            elem.textContent = invert ? "ARMED" : "DETECTED";
            elem.className = "dv err";
        }
        applyStateStyles(elem, safetyState.state, safetyState.elapsedSec);
    };

    const setEstopCell = (elem, armed) => {
        if (!elem) return;
        if (safetyState.state === "NO_SIGNAL" || armed === undefined) {
            elem.textContent = "NO SIGNAL";
            elem.className = "dv";
            applyStateStyles(elem, "NO_SIGNAL");
            return;
        }
        if (armed) {
            elem.textContent = "ARMED";
            elem.className = "dv err";
        } else {
            elem.textContent = "CLEAR";
            elem.className = "dv ok";
        }
        applyStateStyles(elem, safetyState.state, safetyState.elapsedSec);
    };

    const setTrigCell = (elem, triggered) => {
        if (!elem) return;
        if (safetyState.state === "NO_SIGNAL" || triggered === undefined) {
            elem.textContent = "NO SIGNAL";
            elem.className = "dv";
            applyStateStyles(elem, "NO_SIGNAL");
            return;
        }
        if (triggered) {
            elem.textContent = "TRIGGERED";
            elem.className = "dv err";
        } else {
            elem.textContent = "CLEAR";
            elem.className = "dv ok";
        }
        applyStateStyles(elem, safetyState.state, safetyState.elapsedSec);
    };

    setEstopCell(safeEstopMech, safety.estop_mech_armed);
    setEstopCell(safeEstopWire, safety.estop_wire_armed);
    setTrigCell(safeEstopTrig, safety.estop_triggered);

    setBoolCell(safeBlocked,       safety.is_blocked !== undefined ? !safety.is_blocked : undefined);
    setBoolCell(safeTouched,       safety.obstacle_touched !== undefined ? !safety.obstacle_touched : undefined);
    setBoolCell(safeCollision,     safety.collision_detected !== undefined ? !safety.collision_detected : undefined);
    setBoolCell(safeBorderCrossed, safety.border_crossed !== undefined ? !safety.border_crossed : undefined);
    setBoolCell(safeBorderPartial, safety.border_partial !== undefined ? !safety.border_partial : undefined);

    // 8. Jetson System node checklist
    const rosState = getTopicState("ros");
    const setNodeRow = (dotId, lblId, hbId, alive) => {
        const dotElem = document.getElementById(dotId);
        const lblElem = document.getElementById(lblId);
        const hbElem = document.getElementById(hbId);
        if (dotElem && lblElem && hbElem) {
            if (rosState.state === "NO_SIGNAL" || alive === undefined) {
                dotElem.className = "chip-dot d-offline";
                lblElem.textContent = "NO SIGNAL";
                lblElem.className = "dv";
                hbElem.textContent = "--";
                applyStateStyles(lblElem, "NO_SIGNAL");
            } else if (alive) {
                dotElem.className = "chip-dot d-online";
                lblElem.textContent = "ONLINE";
                lblElem.className = "dv ok";
                hbElem.textContent = jetson.uptime_sec ? `${jetson.uptime_sec}s` : "active";
                applyStateStyles(lblElem, "LIVE");
            } else {
                dotElem.className = "chip-dot d-offline";
                lblElem.textContent = "OFFLINE";
                lblElem.className = "dv err";
                hbElem.textContent = "--";
                applyStateStyles(lblElem, "LIVE");
            }
        }
    };
    setNodeRow("setup-node-motor-dot", "setup-node-motor-lbl", "setup-node-motor-heartbeat", ros.node_motor_ctrl);
    setNodeRow("setup-node-lane-dot",  "setup-node-lane-lbl",  "setup-node-lane-heartbeat",  ros.node_lane_det);
    setNodeRow("setup-node-avoid-dot", "setup-node-avoid-lbl", "setup-node-avoid-heartbeat", ros.node_obs_avoid);
    setNodeRow("setup-node-nav-dot",   "setup-node-nav-lbl",   "setup-node-nav-heartbeat",   ros.node_wp_nav);
    setNodeRow("setup-node-vision-dot","setup-node-vision-lbl","setup-node-vision-heartbeat",ros.node_img_recog);
    setNodeRow("setup-node-telem-dot", "setup-node-telem-lbl", "setup-node-telem-heartbeat", rosState.state !== "NO_SIGNAL");
    setNodeRow("setup-esp32-dot",      "setup-esp32-lbl",      "setup-esp32-heartbeat",      ros.esp32_connected);
    
    // 9. Mission & Payload
    const mission = payload.Mission || {};
    const payloadSys = payload.Payload || {};

    const missionState = getTopicState("mission");
    if (missionPhaseVal) {
        if (missionState.state === "NO_SIGNAL" || mission.phase === undefined) {
            missionPhaseVal.textContent = "NO SIGNAL";
            missionPhaseVal.className = "dv";
        } else {
            missionPhaseVal.textContent = mission.phase;
            missionPhaseVal.className = "dv ok";
        }
        applyStateStyles(missionPhaseVal, missionState.state, missionState.elapsedSec);
    }

    const payloadState = getTopicState("payload_sys");
    if (payloadArmVal) {
        if (payloadState.state === "NO_SIGNAL" || payloadSys.arm_status === undefined) {
            payloadArmVal.textContent = "NO SIGNAL";
            payloadArmVal.className = "dv";
        } else {
            payloadArmVal.textContent = payloadSys.arm_status;
            payloadArmVal.className = "dv ok";
        }
        applyStateStyles(payloadArmVal, payloadState.state, payloadState.elapsedSec);
    }

    // 9. Vision
    const visionState = getTopicState("jetson");
    if (visionConfPct && confDial) {
        if (visionState.state === "NO_SIGNAL" || vision.img_confidence === undefined) {
            visionConfPct.textContent = "NO SIGNAL";
            confDial.style.strokeDasharray = `0, 251.2`;
        } else {
            visionConfPct.textContent = Math.round(vision.img_confidence * 100) + "%";
            confDial.style.strokeDasharray = `${vision.img_confidence * 251.2}, 251.2`;
        }
        applyStateStyles(visionConfPct, visionState.state, visionState.elapsedSec);
    }
    if (visionStatusLbl) {
        if (visionState.state === "NO_SIGNAL" || vision.img_task_status === undefined) {
            visionStatusLbl.textContent = "NO SIGNAL";
        } else {
            visionStatusLbl.textContent = vision.img_task_status.toUpperCase();
        }
        applyStateStyles(visionStatusLbl, visionState.state, visionState.elapsedSec);
    }
    if (visionLane) {
        if (visionState.state === "NO_SIGNAL" || vision.lane_detected === undefined) {
            visionLane.textContent = "NO SIGNAL";
            visionLane.className = "dv";
        } else {
            visionLane.textContent = vision.lane_detected ? "YES" : "NO";
            visionLane.className = vision.lane_detected ? "dv ok" : "dv err";
        }
        applyStateStyles(visionLane, visionState.state, visionState.elapsedSec);
    }
    if (visionObstacles) {
        if (visionState.state === "NO_SIGNAL" || vision.obstacles_count === undefined) {
            visionObstacles.textContent = "NO SIGNAL";
        } else {
            visionObstacles.textContent = vision.obstacles_count + " detected";
        }
        applyStateStyles(visionObstacles, visionState.state, visionState.elapsedSec);
    }
    if (visionLaser) {
        if (visionState.state === "NO_SIGNAL" || vision.laser_active === undefined) {
            visionLaser.textContent = "NO SIGNAL";
            visionLaser.className = "dv";
        } else {
            visionLaser.textContent = vision.laser_active ? "EMITTING" : "STANDBY";
            visionLaser.className = vision.laser_active ? "dv ok" : "dv";
        }
        applyStateStyles(visionLaser, visionState.state, visionState.elapsedSec);
    }
    if (visionFps) {
        if (visionState.state === "NO_SIGNAL" || vision.fps_vision === undefined) {
            visionFps.textContent = "NO SIGNAL";
        } else {
            visionFps.textContent = vision.fps_vision.toFixed(1) + " FPS";
        }
        applyStateStyles(visionFps, visionState.state, visionState.elapsedSec);
    }
    if (visionUptime) {
        if (visionState.state === "NO_SIGNAL" || vision.img_elapsed_sec === undefined) {
            visionUptime.textContent = "NO SIGNAL";
        } else {
            visionUptime.textContent = vision.img_elapsed_sec + " sec";
        }
        applyStateStyles(visionUptime, visionState.state, visionState.elapsedSec);
    }
}

// Start periodic freshness updater
setInterval(renderAllTelemetry, 100);


let activeCommandAbortController = null;

// POST dispatch command
async function sendCommand(command) {
    if (!commandResultLbl) return;
    commandResultLbl.textContent = "⏳ Sending command...";
    commandResultLbl.style.color = "var(--text-secondary)";
    
    // Low-latency WebSocket drive command dispatch (if connected)
    const isSocketConnected = (typeof io !== "undefined" && socket && typeof socket.emit === "function" && socket.connected);
    if (isSocketConnected) {
        try {
            socket.emit("drive_command", command);
            commandResultLbl.textContent = `✅ Dispatched (WS): ${command.action.toUpperCase()}`;
            commandResultLbl.style.color = "var(--green-op)";
        } catch (err) {
            commandResultLbl.textContent = `🔌 WS Error: ${err.message}`;
            commandResultLbl.style.color = "var(--red-alert)";
        }
        setTimeout(() => {
            if (commandResultLbl) {
                commandResultLbl.textContent = "CONSOLE STANDBY";
                commandResultLbl.style.color = "";
            }
        }, 2500);
        return;
    }

    // Abort in-flight HTTP request to prevent connection queuing
    if (activeCommandAbortController) {
        activeCommandAbortController.abort();
    }
    activeCommandAbortController = new AbortController();
    const { signal } = activeCommandAbortController;

    // HTTP POST fallback
    try {
        const response = await fetch("/command", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(command),
            signal
        });
        if (!response.ok) {
            const text = await response.text();
            commandResultLbl.textContent = `⛔ HTTP ${response.status}: ${text.slice(0, 120)}`;
            commandResultLbl.style.color = "var(--red-alert)";
            return;
        }
        const res = await response.json();
        if (res.status === "success") {
            commandResultLbl.textContent = `✅ Dispatched: ${command.action.toUpperCase()}`;
            commandResultLbl.style.color = "var(--green-op)";
        } else {
            commandResultLbl.textContent = `❌ Error: ${res.message}`;
            commandResultLbl.style.color = "var(--red-alert)";
        }
    } catch (err) {
        if (err.name === "AbortError") {
            // Silently swallow aborted commands as they are superseded by newer commands
            return;
        }
        commandResultLbl.textContent = `🔌 Network Error: ${err.message}`;
        commandResultLbl.style.color = "var(--red-alert)";
    }
    // Auto-clear status after 4 seconds
    setTimeout(() => {
        if (commandResultLbl) {
            commandResultLbl.textContent = "CONSOLE STANDBY";
            commandResultLbl.style.color = "";
        }
    }, 4000);
}

// Command trigger listeners
document.getElementById("btn-drive").addEventListener("click", () => {
    const speed = parseFloat(ctrlSpeed.value);
    const heading = parseFloat(ctrlHeading.value);
    const throttle = parseFloat(ctrlThrottle.value);
    const waypoint = parseInt(ctrlWaypoint.value);

    sendCommand({
        action: "drive",
        speed_kmh: speed,
        heading_deg: heading,
        throttle_pct: throttle,
        wp_current: waypoint,
        source: "web_gcs",
        timestamp_ms: Date.now()
    });
});

document.getElementById("btn-stop").addEventListener("click", () => {
    sendCommand({
        action: "stop",
        speed_kmh: 0.0,
        heading_deg: parseFloat(ctrlHeading.value),
        throttle_pct: 0.0,
        source: "web_gcs",
        timestamp_ms: Date.now()
    });
});

document.getElementById("btn-resume").addEventListener("click", () => {
    sendCommand({
        action: "resume",
        source: "web_gcs",
        timestamp_ms: Date.now()
    });
});

document.getElementById("btn-estop").addEventListener("click", () => {
    sendCommand({
        action: "estop",
        speed_kmh: 0.0,
        heading_deg: parseFloat(ctrlHeading.value),
        throttle_pct: 0.0,
        estop_triggered: true,
        source: "web_gcs",
        timestamp_ms: Date.now()
    });
});

if (btnTopEstop) {
    btnTopEstop.addEventListener("click", () => {
        sendCommand({
            action: "estop",
            speed_kmh: 0.0,
            heading_deg: parseFloat(ctrlHeading.value),
            throttle_pct: 0.0,
            estop_triggered: true,
            source: "web_gcs",
            timestamp_ms: Date.now()
        });
    });
}

// Mission Elapsed Time timer
let missionStartTime = Date.now();
const missionTimerElem = document.getElementById("mission-timer");
if (missionTimerElem) {
    setInterval(() => {
        const elapsed = Date.now() - missionStartTime;
        const hrs = String(Math.floor(elapsed / 3600000)).padStart(2, "0");
        const mins = String(Math.floor((elapsed % 3600000) / 60000)).padStart(2, "0");
        const secs = String(Math.floor((elapsed % 60000) / 1000)).padStart(2, "0");
        missionTimerElem.textContent = `${hrs}:${mins}:${secs}`;
    }, 1000);
}

// Dialog Controller

class DialogController {
    constructor(backdropId, dialogId) {
        this.backdrop = document.getElementById(backdropId);
        this.dialog   = document.getElementById(dialogId);
        this.isOpen   = false;
    }

    open() {
        if (this.isOpen) return;
        this.isOpen = true;
        this.backdrop.classList.add('open');
        this.dialog.classList.add('open');
        document.body.style.overflow = 'hidden';
    }

    close() {
        if (!this.isOpen) return;
        this.isOpen = false;
        this.backdrop.classList.remove('open');
        this.dialog.classList.remove('open');
        document.body.style.overflow = '';
    }
}

const testSuiteDialog = new DialogController('test-suite-backdrop', 'test-suite-dialog');

// Wire dialog close buttons
document.getElementById('btn-close-dialog')?.addEventListener('click', () => testSuiteDialog.close());
document.getElementById('btn-dialog-cancel')?.addEventListener('click', () => testSuiteDialog.close());
// Close on backdrop click
document.getElementById('test-suite-backdrop')?.addEventListener('click', () => testSuiteDialog.close());

// Test Suite Trigger Listener
const btnStartTestSuite = document.getElementById("btn-start-test-suite");
const testSuiteStatus  = document.getElementById("test-suite-status");
const testSuiteResult  = document.getElementById("test-suite-result");

if (btnStartTestSuite) {
    btnStartTestSuite.addEventListener("click", () => {
        testSuiteDialog.open();
    });
}

const btnStopTestSuite = document.getElementById("btn-stop-test-suite");
if (btnStopTestSuite) {
    btnStopTestSuite.addEventListener("click", async () => {
        testSuiteStatus.textContent = "STOPPING";
        testSuiteResult.textContent = "⏳ Stopping active simulation nodes...";
        testSuiteResult.style.color = "var(--amber)";
        btnStopTestSuite.disabled = true;
        btnStopTestSuite.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> Stopping...';
        
        try {
            const response = await fetch("/api/test-suite/stop", { method: "POST" });
            const res = await response.json();
            btnStopTestSuite.disabled = false;
            btnStopTestSuite.innerHTML = '<i class="fa-solid fa-stop"></i> STOP SIM';
            
            if (res.status === "success") {
                testSuiteStatus.textContent = "READY";
                testSuiteResult.textContent = "🛑 " + res.message;
                testSuiteResult.style.color = "var(--text-secondary)";
            } else {
                testSuiteStatus.textContent = "ERROR";
                testSuiteResult.textContent = "❌ " + res.message;
                testSuiteResult.style.color = "var(--red-alert)";
            }
        } catch (err) {
            btnStopTestSuite.disabled = false;
            btnStopTestSuite.innerHTML = '<i class="fa-solid fa-stop"></i> STOP SIM';
            testSuiteStatus.textContent = "ERROR";
            testSuiteResult.textContent = `❌ Network Error: ${err.message}`;
            testSuiteResult.style.color = "var(--red-alert)";
        }
    });
}

// Wire the Proceed button in the dialog to start the test suite
const btnDialogProceed = document.getElementById('btn-dialog-proceed');
if (btnDialogProceed) {
    btnDialogProceed.addEventListener("click", async () => {
        testSuiteStatus.textContent = "SPAWNING";
        testSuiteResult.textContent = "⏳ Spawning local terminals...";
        testSuiteResult.style.color = "";

        btnDialogProceed.disabled = true;
        btnDialogProceed.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i>&nbsp; Spawning...';

        try {
            const response = await fetch("/api/test-suite/start", { method: "POST" });
            const res = await response.json();
            if (res.status === "success") {
                testSuiteStatus.textContent = "ACTIVE";
                testSuiteResult.textContent = "✅ " + res.message;
                testSuiteResult.style.color = "var(--green-op)";
                setTimeout(() => {
                    testSuiteDialog.close();
                    btnDialogProceed.disabled = false;
                    btnDialogProceed.innerHTML = '<i class="fa-solid fa-rocket"></i>&nbsp; Proceed';
                }, 1200);
            } else {
                testSuiteStatus.textContent = "ERROR";
                testSuiteResult.textContent = "❌ " + res.message;
                testSuiteResult.style.color = "var(--red-alert)";
                testSuiteDialog.close();
                btnDialogProceed.disabled = false;
                btnDialogProceed.innerHTML = '<i class="fa-solid fa-rocket"></i>&nbsp; Proceed';
            }
        } catch (err) {
            testSuiteStatus.textContent = "ERROR";
            testSuiteResult.textContent = `❌ Network Error: ${err.message}`;
            testSuiteResult.style.color = "var(--red-alert)";
            testSuiteDialog.close();
            btnDialogProceed.disabled = false;
            btnDialogProceed.innerHTML = '<i class="fa-solid fa-rocket"></i>&nbsp; Proceed';
        }
    });
}

// Wire the Run Tests button
const btnRunUnitTests = document.getElementById("btn-run-unit-tests");
if (btnRunUnitTests) {
    btnRunUnitTests.addEventListener("click", async () => {
        testSuiteStatus.textContent = "TESTING";
        testSuiteResult.textContent = "⏳ Running unittest suite...";
        testSuiteResult.style.color = "var(--cyan)";

        btnRunUnitTests.disabled = true;
        btnRunUnitTests.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> Testing...';

        try {
            const response = await fetch("/api/test-suite/run", { method: "POST" });
            const res = await response.json();
            btnRunUnitTests.disabled = false;
            btnRunUnitTests.innerHTML = '<i class="fa-solid fa-vial"></i> RUN TESTS';

            if (res.status === "success") {
                testSuiteStatus.textContent = "PASSED";
                testSuiteStatus.style.background = "rgba(16, 185, 129, 0.1)";
                testSuiteStatus.style.color = "var(--green-op)";

                const lines = res.logs.split("\n");
                const summaryLine = lines[lines.length - 1] || "All tests passed!";
                testSuiteResult.textContent = `✅ PASSED: ${summaryLine}`;
                testSuiteResult.style.color = "var(--green-op)";
            } else {
                testSuiteStatus.textContent = "FAILED";
                testSuiteStatus.style.background = "rgba(239, 68, 68, 0.1)";
                testSuiteStatus.style.color = "var(--red-alert)";
                testSuiteResult.textContent = `❌ FAILED: Check console logs.`;
                testSuiteResult.style.color = "var(--red-alert)";
                console.error("Test suite failures:\n", res.logs);
            }
        } catch (err) {
            btnRunUnitTests.disabled = false;
            btnRunUnitTests.innerHTML = '<i class="fa-solid fa-vial"></i> RUN TESTS';
            testSuiteStatus.textContent = "ERROR";
            testSuiteResult.textContent = `❌ Network Error: ${err.message}`;
            testSuiteResult.style.color = "var(--red-alert)";
        }
    });
}

// Wire the Run UI Tests button
const btnRunUiTests = document.getElementById("btn-run-ui-tests");
if (btnRunUiTests) {
    btnRunUiTests.addEventListener("click", async () => {
        testSuiteStatus.textContent = "TESTING UI";
        testSuiteResult.textContent = "⏳ Running UI integration test suite...";
        testSuiteResult.style.color = "var(--cyan)";

        btnRunUiTests.disabled = true;
        btnRunUiTests.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> Testing UI...';

        try {
            const response = await fetch("/api/test-suite/ui-run", { method: "POST" });
            const res = await response.json();
            btnRunUiTests.disabled = false;
            btnRunUiTests.innerHTML = '<i class="fa-solid fa-desktop"></i> RUN UI TESTS';

            if (res.status === "success") {
                testSuiteStatus.textContent = "PASSED";
                testSuiteStatus.style.background = "rgba(16, 185, 129, 0.1)";
                testSuiteStatus.style.color = "var(--green-op)";

                const lines = res.logs.split("\n");
                const summaryLine = lines[lines.length - 1] || "All UI tests passed!";
                testSuiteResult.textContent = `✅ UI PASSED: ${summaryLine}`;
                testSuiteResult.style.color = "var(--green-op)";
            } else {
                testSuiteStatus.textContent = "FAILED";
                testSuiteStatus.style.background = "rgba(239, 68, 68, 0.1)";
                testSuiteStatus.style.color = "var(--red-alert)";
                testSuiteResult.textContent = `❌ UI FAILED: Check console logs.`;
                testSuiteResult.style.color = "var(--red-alert)";
                console.error("UI Test suite failures:\n", res.logs);
            }
        } catch (err) {
            btnRunUiTests.disabled = false;
            btnRunUiTests.innerHTML = '<i class="fa-solid fa-desktop"></i> RUN UI TESTS';
            testSuiteStatus.textContent = "ERROR";
            testSuiteResult.textContent = `❌ Network Error: ${err.message}`;
            testSuiteResult.style.color = "var(--red-alert)";
        }
    });
}


// ─── Tab Switching Logic ───
const tabBtns = document.querySelectorAll(".tab-btn");
const tabContents = document.querySelectorAll(".tab-content");

tabBtns.forEach(btn => {
    btn.addEventListener("click", () => {
        const targetTab = btn.getAttribute("data-tab");
        
        tabBtns.forEach(b => b.classList.remove("active"));
        tabContents.forEach(c => c.classList.remove("active-content"));
        
        btn.classList.add("active");
        const targetContent = document.getElementById(targetTab);
        if (targetContent) {
            targetContent.classList.add("active-content");
        }
    });
});

// ─── Network Config Init & Event Handlers ───
const setupInterfaceSelect = document.getElementById("setup-interface");
const setupIpInput          = document.getElementById("setup-ip");
const setupDomainInput      = document.getElementById("setup-domain");
const setupDiscoverySelect   = document.getElementById("setup-discovery");
const setupPeerIdInput      = document.getElementById("setup-peer-id");
const btnApplyNetwork       = document.getElementById("btn-apply-network");
const networkConfigStatus   = document.getElementById("network-config-status");
const meshAnnouncerLogs     = document.getElementById("mesh-announcer-logs");

// Fetch local network interfaces
async function loadNetworkInterfaces() {
    try {
        const response = await fetch("/api/setup/interfaces");
        const res = await response.json();
        if (res.interfaces && res.interfaces.length > 0) {
            if (setupInterfaceSelect) {
                setupInterfaceSelect.innerHTML = "";
                res.interfaces.forEach(iface => {
                    const opt = document.createElement("option");
                    opt.value = iface.ip;
                    opt.textContent = `${iface.interface} (${iface.ip})`;
                    setupInterfaceSelect.appendChild(opt);
                });
                // Set initial value
                if (setupIpInput) {
                    setupIpInput.value = setupInterfaceSelect.value;
                }
            }
        }
    } catch (err) {
        console.error("Failed to load network interfaces:", err);
        appendMeshLog(`[Error] Failed to query network interfaces: ${err.message}`);
    }
}

if (setupInterfaceSelect) {
    setupInterfaceSelect.addEventListener("change", (e) => {
        if (setupIpInput) {
            setupIpInput.value = e.target.value;
        }
    });
}

function appendMeshLog(text) {
    if (meshAnnouncerLogs) {
        const timeStr = new Date().toLocaleTimeString();
        meshAnnouncerLogs.textContent += `\n[${timeStr}] ${text}`;
        meshAnnouncerLogs.scrollTop = meshAnnouncerLogs.scrollHeight;
    }
}

async function loadNetworkConfig() {
    try {
        const response = await fetch("/api/config/network");
        if (response.ok) {
            const config = await response.json();
            if (setupDomainInput && config.domain !== undefined) setupDomainInput.value = config.domain;
            if (setupDiscoverySelect && config.discovery !== undefined) setupDiscoverySelect.value = config.discovery;
            if (setupPeerIdInput && config.peer_id !== undefined) setupPeerIdInput.value = config.peer_id;
            if (setupIpInput && config.local_ip !== undefined) setupIpInput.value = config.local_ip;
            if (networkConfigStatus) {
                networkConfigStatus.textContent = `Applied Config: domain=${config.domain || "0"}, discovery=${config.discovery || "SUBNET"}, peer=${config.peer_id || "gcs-operator"}`;
                networkConfigStatus.style.color = "var(--cyan)";
            }
        }
    } catch (err) {
        console.error("Failed to load network config:", err);
    }
}

if (btnApplyNetwork) {
    btnApplyNetwork.addEventListener("click", async () => {
        const domain = setupDomainInput ? setupDomainInput.value : "0";
        const discovery = setupDiscoverySelect ? setupDiscoverySelect.value : "SUBNET";
        const peerId = setupPeerIdInput ? setupPeerIdInput.value : "gcs-operator";
        const localIp = setupIpInput ? setupIpInput.value : "127.0.0.1";

        if (networkConfigStatus) {
            networkConfigStatus.textContent = "⏳ Applying configuration...";
            networkConfigStatus.style.color = "var(--text-secondary)";
        }

        try {
            const response = await fetch("/api/config/network", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({
                    domain: domain,
                    discovery: discovery,
                    peer_id: peerId,
                    local_ip: localIp
                })
            });
            if (response.ok) {
                if (networkConfigStatus) {
                    networkConfigStatus.textContent = `Applied Config: domain=${domain}, discovery=${discovery}, peer=${peerId}`;
                    networkConfigStatus.style.color = "var(--cyan)";
                }
                appendMeshLog(`[System] Reconfigured local GCS Node properties.`);
                appendMeshLog(` - Peer ID: ${peerId}`);
                appendMeshLog(` - IP Bind: ${localIp}`);
                appendMeshLog(` - ROS Domain ID: ${domain}`);
                appendMeshLog(` - ROS Discovery Range: ${discovery}`);
                appendMeshLog(`[System] Restarting discovery listener with updated parameter group... OK`);
            } else {
                const errText = await response.text();
                if (networkConfigStatus) {
                    networkConfigStatus.textContent = `❌ Error: ${errText}`;
                    networkConfigStatus.style.color = "var(--red-alert)";
                }
            }
        } catch (err) {
            if (networkConfigStatus) {
                networkConfigStatus.textContent = `🔌 Network Error: ${err.message}`;
                networkConfigStatus.style.color = "var(--red-alert)";
            }
        }
    });
}

// ─── Topic Customizer ───
let gcsTopics = {};

const DEFAULT_TOPICS = {
    "motor_control": { label: "Motor Control", path: "/rover/commands/motor" },
    "lane_detection": { label: "Lane Detection", path: "/rover/sensors/lane" },
    "obstacle_avoidance": { label: "Obstacle Avoidance", path: "/scan" },
    "waypoint_navigator": { label: "Waypoint Navigator", path: "/rover/commands/nav" },
    "image_recognition": { label: "Image Recognition", path: "/rgb/image_raw/compressed" },
    "telemetry_aggregator": { label: "Telemetry Aggregator", path: "/rover/telemetry" },
    "telemetry_nav": { label: "Telemetry Nav", path: "/rover/telemetry/nav" },
    "telemetry_safety": { label: "Telemetry Safety", path: "/rover/telemetry/safety" },
    "telemetry_control": { label: "Telemetry Control", path: "/rover/telemetry/control" },
    "telemetry_vision": { label: "Telemetry Vision", path: "/rover/telemetry/vision" },
    "imu_accel": { label: "IMU ACCEL", path: "/imu" },
    "odom_coord": { label: "ODOM COORD", path: "/odom" },
    "cmd_vel_echo": { label: "CMD_VEL ECHO", path: "/cmd_vel" },
    "gps_fix": { label: "GPS /FIX", path: "/gps" },
    "battery_state": { label: "BATTERY STATE", path: "/battery_state" }
};

async function fetchTopicConfig() {
    try {
        const res = await fetch("/api/config/topics");
        if (res.ok) {
            gcsTopics = await res.json();
        } else {
            gcsTopics = JSON.parse(JSON.stringify(DEFAULT_TOPICS));
        }
    } catch (e) {
        console.error("Failed to fetch topic configuration from server", e);
        gcsTopics = JSON.parse(JSON.stringify(DEFAULT_TOPICS));
    }
    updateUIWithTopics();
}

function updateUIWithTopics() {
    // 1. Update Subsystems Table in Setup Tab
    for (const key of ["motor_control", "lane_detection", "obstacle_avoidance", "waypoint_navigator", "image_recognition", "telemetry_aggregator"]) {
        const labelEl = document.getElementById(`subsystem-label-${key}`);
        const pathEl = document.getElementById(`subsystem-path-${key}`);
        if (gcsTopics[key]) {
            if (labelEl) labelEl.textContent = gcsTopics[key].label;
            if (pathEl) pathEl.textContent = gcsTopics[key].path;
        }
    }
    
    // Update camera topic toggle button state
    if (gcsTopics["image_recognition"] && gcsTopics["image_recognition"].path) {
        if (typeof updateCamTopicButtonState === "function") {
            updateCamTopicButtonState(gcsTopics["image_recognition"].path);
        }
    }
    
    // 2. Render inputs in the customizer panel
    renderTopicConfigPanel();
}

function renderTopicConfigPanel() {
    const listEl = document.getElementById("topics-edit-list");
    if (!listEl) return;
    
    listEl.innerHTML = "";
    
    const keys = Object.keys(gcsTopics);
    
    keys.forEach(key => {
        const isStandard = ["imu_accel", "odom_coord", "cmd_vel_echo", "gps_fix", "battery_state"].includes(key);
        const typeBadge = isStandard ? "Standard HUD" : "Subsystem";
        const topic = gcsTopics[key];
        
        const row = document.createElement("div");
        row.className = "form-row";
        row.style = "margin-bottom: 12px; border-bottom: 1px solid rgba(28,35,51,0.3); padding-bottom: 8px;";
        row.innerHTML = `
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 4px;">
                <span class="stat-lbl" style="margin-bottom: 0; font-size: 11px; font-weight: bold; color: var(--text-bright);">${key.replace(/_/g, ' ').toUpperCase()}</span>
                <span class="ph-badge" style="font-size: 8px; color: ${isStandard ? 'var(--cyan)' : 'var(--green-op)'}; border-color: ${isStandard ? 'var(--cyan)' : 'var(--green-op)'}; background: transparent; padding: 1px 4px; line-height: 1;">${typeBadge}</span>
            </div>
            <div style="display: flex; gap: 6px;">
                <div style="flex: 1;">
                    <label style="font-size: 8px; color: var(--text-secondary); text-transform: uppercase; display: block; margin-bottom: 2px;">Display Label</label>
                    <input type="text" id="edit-label-${key}" class="setup-input" value="${topic.label}" style="font-size: 11px; height: 26px; padding: 0 6px; width: 100%;">
                </div>
                <div style="flex: 1.5;">
                    <label style="font-size: 8px; color: var(--text-secondary); text-transform: uppercase; display: block; margin-bottom: 2px;">ROS Topic Path</label>
                    <input type="text" id="edit-path-${key}" class="setup-input" value="${topic.path}" style="font-size: 11px; height: 26px; padding: 0 6px; font-family: var(--font-mono); width: 100%;">
                </div>
            </div>
        `;
        listEl.appendChild(row);
    });
}

async function saveTopicConfig() {
    const updated = {};
    Object.keys(gcsTopics).forEach(key => {
        const labelVal = document.getElementById(`edit-label-${key}`)?.value || gcsTopics[key].label;
        const pathVal = document.getElementById(`edit-path-${key}`)?.value || gcsTopics[key].path;
        updated[key] = { label: labelVal, path: pathVal };
    });
    
    const statusEl = document.getElementById("topics-config-status");
    if (statusEl) {
        statusEl.textContent = "Saving topic configuration...";
        statusEl.style.color = "var(--amber)";
    }
    
    try {
        const res = await fetch("/api/config/topics", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(updated)
        });
        
        if (res.ok) {
            gcsTopics = updated;
            updateUIWithTopics();
            if (statusEl) {
                statusEl.textContent = "Topic configuration applied successfully";
                statusEl.style.color = "var(--green-op)";
                setTimeout(() => {
                    statusEl.textContent = "Topic configuration synchronized";
                    statusEl.style.color = "var(--text-secondary)";
                }, 3000);
            }
        } else {
            const data = await res.json();
            throw new Error(data.message || "Server rejected update");
        }
    } catch (e) {
        console.error(e);
        if (statusEl) {
            statusEl.textContent = `Error: ${e.message}`;
            statusEl.style.color = "var(--red-alert)";
        }
    }
}

async function resetTopicConfig() {
    const statusEl = document.getElementById("topics-config-status");
    if (statusEl) {
        statusEl.textContent = "Reverting to default topics...";
        statusEl.style.color = "var(--amber)";
    }
    
    try {
        const res = await fetch("/api/config/topics/reset", {
            method: "POST"
        });
        
        if (res.ok) {
            await fetchTopicConfig();
            if (statusEl) {
                statusEl.textContent = "Reverted to standard topic defaults";
                statusEl.style.color = "var(--green-op)";
                setTimeout(() => {
                    statusEl.textContent = "Topic configuration synchronized";
                    statusEl.style.color = "var(--text-secondary)";
                }, 3000);
            }
        } else {
            const data = await res.json();
            throw new Error(data.message || "Server rejected reset");
        }
    } catch (e) {
        console.error(e);
        if (statusEl) {
            statusEl.textContent = `Error: ${e.message}`;
            statusEl.style.color = "var(--red-alert)";
        }
    }
}

// Fetch topic configuration on start
fetchTopicConfig();

// Bind button event listeners
document.getElementById("btn-save-topics")?.addEventListener("click", saveTopicConfig);
document.getElementById("btn-reset-topics")?.addEventListener("click", resetTopicConfig);

// ─── ROS 2 Active Network Topics ───
let activeNetworkTopics = [];

async function refreshNetworkTopics() {
    const listEl = document.getElementById("network-topics-list");
    const refreshBtn = document.getElementById("btn-refresh-topics");
    if (!listEl) return;
    
    if (refreshBtn) {
        refreshBtn.disabled = true;
        refreshBtn.innerHTML = '<i class="fa-solid fa-arrows-rotate fa-spin"></i> QUERYING...';
    }
    
    try {
        const res = await fetch("/api/ros2/topics");
        if (res.ok) {
            const data = await res.json();
            activeNetworkTopics = data.topics || [];
            renderNetworkTopics();
        } else {
            listEl.innerHTML = '<tr><td colspan="2" style="color: var(--red-alert); text-align: center; padding: 8px;">Failed to fetch topics</td></tr>';
        }
    } catch (e) {
        console.error(e);
        listEl.innerHTML = '<tr><td colspan="2" style="color: var(--red-alert); text-align: center; padding: 8px;">Error connecting to server</td></tr>';
    } finally {
        if (refreshBtn) {
            refreshBtn.disabled = false;
            refreshBtn.innerHTML = '<i class="fa-solid fa-arrows-rotate"></i> REFRESH';
        }
    }
}

function renderNetworkTopics() {
    const listEl = document.getElementById("network-topics-list");
    if (!listEl) return;
    
    const filter = document.getElementById("topic-search")?.value.toLowerCase() || "";
    listEl.innerHTML = "";
    
    const filtered = activeNetworkTopics.filter(t => t.name.toLowerCase().includes(filter));
    
    if (filtered.length === 0) {
        listEl.innerHTML = '<tr><td colspan="2" style="color: var(--text-secondary); text-align: center; padding: 8px;">No topics found</td></tr>';
        return;
    }
    
    filtered.forEach(t => {
        const row = document.createElement("tr");
        row.style.borderBottom = "1px solid rgba(28,35,51,0.2)";
        
        const typeStr = t.types && t.types.length > 0 ? t.types[0] : "unknown";
        
        row.innerHTML = `
            <td style="padding: 4px 6px; color: var(--cyan); text-align: left; word-break: break-all; cursor: pointer;" title="Click to copy path" class="clickable-topic">${t.name}</td>
            <td style="padding: 4px 6px; color: var(--text-secondary); text-align: left; font-size: 9px; word-break: break-all;">${typeStr}</td>
        `;
        listEl.appendChild(row);
    });
}

// Bind active network topics event listeners
document.getElementById("btn-refresh-topics")?.addEventListener("click", refreshNetworkTopics);
document.getElementById("topic-search")?.addEventListener("input", renderNetworkTopics);

const networkTopicsList = document.getElementById("network-topics-list");
if (networkTopicsList) {
    networkTopicsList.addEventListener("click", (e) => {
        if (e.target.classList.contains("clickable-topic")) {
            const path = e.target.textContent;
            navigator.clipboard.writeText(path).then(() => {
                const statusEl = document.getElementById("topics-config-status");
                if (statusEl) {
                    statusEl.textContent = `Copied topic path: ${path}`;
                    statusEl.style.color = "var(--cyan)";
                    setTimeout(() => {
                        statusEl.textContent = "Topic configuration synchronized";
                        statusEl.style.color = "var(--text-secondary)";
                    }, 3000);
                }
            }).catch(err => {
                console.error("Clipboard copy failed:", err);
            });
        }
    });
}

// Initial fetch of active network topics
refreshNetworkTopics();

// ─── Rover Diagnostics ───
const setupRoverIpInput = document.getElementById("setup-rover-ip");
const btnPingTest       = document.getElementById("btn-ping-test");
const btnRunDiagnostics = document.getElementById("btn-run-diagnostics");
const btnClearDiagLogs  = document.getElementById("btn-clear-diag-logs");
const diagPingRtt       = document.getElementById("diag-ping-rtt");
const diagConnHealth    = document.getElementById("diag-conn-health");
const diagConsoleLogs   = document.getElementById("diag-console-logs");

function appendDiagLog(text) {
    if (diagConsoleLogs) {
        const timeStr = new Date().toLocaleTimeString();
        diagConsoleLogs.textContent += `\n[${timeStr}] ${text}`;
        diagConsoleLogs.scrollTop = diagConsoleLogs.scrollHeight;
    }
}

if (btnClearDiagLogs) {
    btnClearDiagLogs.addEventListener("click", () => {
        if (diagConsoleLogs) {
            diagConsoleLogs.textContent = "Logs cleared.";
        }
    });
}

async function runPingTest(host) {
    appendDiagLog(`[Diagnostics] Spawning ping process targeting: ${host}...`);
    try {
        const response = await fetch("/api/setup/ping", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ host })
        });
        const res = await response.json();
        
        if (res.status === "success") {
            appendDiagLog(`[Ping Result] SUCCESS:\n${res.logs}`);
            // Attempt to extract avg RTT
            const rttMatch = res.logs.match(/rtt min\/avg\/max\/mdev = [\d\.]+\/([\d\.]+)/);
            if (rttMatch && rttMatch[1]) {
                const avgRtt = rttMatch[1];
                if (diagPingRtt) {
                    diagPingRtt.textContent = `${avgRtt} ms`;
                    diagPingRtt.className = "dv ok";
                }
            } else {
                if (diagPingRtt) {
                    diagPingRtt.textContent = "< 5 ms";
                    diagPingRtt.className = "dv ok";
                }
            }
            if (diagConnHealth) {
                diagConnHealth.textContent = "NOMINAL";
                diagConnHealth.className = "dv ok";
            }
            return true;
        } else {
            appendDiagLog(`[Ping Result] FAILED:\n${res.logs || res.message}`);
            if (diagPingRtt) {
                diagPingRtt.textContent = "TIMEOUT";
                diagPingRtt.className = "dv err";
            }
            if (diagConnHealth) {
                diagConnHealth.textContent = "DISCONNECTED";
                diagConnHealth.className = "dv err";
            }
            return false;
        }
    } catch (err) {
        appendDiagLog(`[Diagnostics Error] Network communication failure: ${err.message}`);
        if (diagPingRtt) {
            diagPingRtt.textContent = "ERROR";
            diagPingRtt.className = "dv err";
        }
        if (diagConnHealth) {
            diagConnHealth.textContent = "ERROR";
            diagConnHealth.className = "dv err";
        }
        return false;
    }
}

if (btnPingTest) {
    btnPingTest.addEventListener("click", async () => {
        const host = setupRoverIpInput ? setupRoverIpInput.value.trim() : "127.0.0.1";
        btnPingTest.disabled = true;
        btnPingTest.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> PINGING';
        
        await runPingTest(host);
        
        btnPingTest.disabled = false;
        btnPingTest.innerHTML = 'PING TEST';
    });
}

if (btnRunDiagnostics) {
    btnRunDiagnostics.addEventListener("click", async () => {
        const host = setupRoverIpInput ? setupRoverIpInput.value.trim() : "127.0.0.1";
        btnRunDiagnostics.disabled = true;
        btnRunDiagnostics.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> RUNNING';
        
        appendDiagLog("==============================================");
        appendDiagLog("[Diagnostics] Initializing Full System Connection Health Check");
        appendDiagLog("==============================================");
        
        appendDiagLog("[Step 1/3] Resolving host and checking network path...");
        const pingSuccess = await runPingTest(host);
        
        appendDiagLog("[Step 2/3] Verifying active ROS subsystem nodes...");
        const nodeMotorCtrl = document.getElementById("setup-node-motor-lbl")?.textContent === "ONLINE";
        const nodeNav = document.getElementById("setup-node-nav-lbl")?.textContent === "ONLINE";
        const nodeSafety = document.getElementById("setup-node-avoid-lbl")?.textContent === "ONLINE";
        const nodeVision = document.getElementById("setup-node-vision-lbl")?.textContent === "ONLINE";
        const esp32Conn = document.getElementById("setup-esp32-lbl")?.textContent === "ONLINE";

        appendDiagLog(` - Motor Control Node: ${nodeMotorCtrl ? 'ONLINE' : 'OFFLINE (Critical)'}`);
        appendDiagLog(` - ESP32 Serial Link: ${esp32Conn ? 'ONLINE' : 'OFFLINE (Critical)'}`);
        appendDiagLog(` - Waypoint Nav Node: ${nodeNav ? 'ONLINE' : 'OFFLINE (Warning)'}`);
        appendDiagLog(` - Obstacle Avoidance: ${nodeSafety ? 'ONLINE' : 'OFFLINE (Warning)'}`);
        appendDiagLog(` - Image Recognition: ${nodeVision ? 'ONLINE' : 'OFFLINE (Simulation)'}`);

        appendDiagLog("[Step 3/3] Parsing Global Team Mesh status...");
        const setupMeshConn = document.getElementById("setup-mesh-connections-lbl");
        const activePeers = setupMeshConn ? parseInt(setupMeshConn.textContent || "0") : 0;
        appendDiagLog(` - Active mesh connections: ${activePeers} peer(s)`);

        appendDiagLog("----------------------------------------------");
        if (pingSuccess && nodeMotorCtrl) {
            appendDiagLog("[Diagnostics Status] PASS: Rover GCS link established successfully.");
            if (diagConnHealth) {
                diagConnHealth.textContent = "NOMINAL";
                diagConnHealth.className = "dv ok";
            }
        } else if (pingSuccess) {
            appendDiagLog("[Diagnostics Status] WARN: Physical connection OK, but some ROS subsystems are offline.");
            if (diagConnHealth) {
                diagConnHealth.textContent = "DEGRADED";
                diagConnHealth.className = "dv warn";
            }
        } else {
            appendDiagLog("[Diagnostics Status] FAIL: Rover is unreachable. Verify physical layer/wireless bridge config.");
            if (diagConnHealth) {
                diagConnHealth.textContent = "CRITICAL FAIL";
                diagConnHealth.className = "dv err";
            }
        }
        appendDiagLog("==============================================");
        
        btnRunDiagnostics.disabled = false;
        btnRunDiagnostics.innerHTML = '<i class="fa-solid fa-calculator"></i> RUN DIAGNOSTICS';
    });
}

// Load interfaces on start
loadNetworkInterfaces();
loadNetworkConfig();


// ═══════════════════════════════════════════════════
// JETSON SSH REMOTE CONTROL BINDINGS
// ═══════════════════════════════════════════════════

const sshHostInput          = document.getElementById("ssh-host");
const sshPortInput          = document.getElementById("ssh-port");
const sshUserInput          = document.getElementById("ssh-user");
const sshPasswordInput      = document.getElementById("ssh-password");
const btnSshTunnelStart     = document.getElementById("btn-ssh-tunnel-start");
const btnSshTunnelStop      = document.getElementById("btn-ssh-tunnel-stop");
const btnSshLaunchBringup   = document.getElementById("btn-ssh-launch-bringup");
const btnSshStopBringup     = document.getElementById("btn-ssh-stop-bringup");
const sshCmdInput           = document.getElementById("ssh-cmd-input");
const btnSshExecCmd         = document.getElementById("btn-ssh-exec-cmd");
const sshTunnelStatusLbl    = document.getElementById("ssh-tunnel-status-lbl");
const sshNodesStatusLbl     = document.getElementById("ssh-nodes-status-lbl");
const sshConsoleLogs        = document.getElementById("ssh-console-logs");

function appendSshLog(text) {
    if (sshConsoleLogs) {
        const timeStr = new Date().toLocaleTimeString();
        sshConsoleLogs.textContent += `\n[${timeStr}] ${text}`;
        sshConsoleLogs.scrollTop = sshConsoleLogs.scrollHeight;
    }
}

function getSshConfig() {
    return {
        host: sshHostInput ? sshHostInput.value.trim() : "127.0.0.1",
        port: sshPortInput ? parseInt(sshPortInput.value) : 22,
        user: sshUserInput ? sshUserInput.value.trim() : "ubuntu",
        password: sshPasswordInput && sshPasswordInput.value ? sshPasswordInput.value : null,
        local_port: 8091,
        remote_port: 8090
    };
}

if (btnSshTunnelStart) {
    btnSshTunnelStart.addEventListener("click", async () => {
        const config = getSshConfig();
        appendSshLog(`[SSH Tunnel] Establishing tunnel to ${config.user}@${config.host}:${config.port}...`);
        btnSshTunnelStart.disabled = true;
        btnSshTunnelStart.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> CONNECTING';

        try {
            const res = await fetch("/api/ssh/tunnel/start", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(config)
            });
            const data = await res.json();
            if (data.status === "success") {
                appendSshLog(`[SSH Tunnel] SUCCESS: ${data.message}`);
                // Automatically add the tunneled peer to GCS Global Connector
                appendSshLog(`[GCS Connector] Mapping tunneled remote peer to 127.0.0.1:8091...`);
                await fetch("/api/peers", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({
                        peer_id: "ssh-jetson",
                        ip_address: "127.0.0.1",
                        port: 8091,
                        role: "rover",
                        team_name: "TunneledJetson"
                    })
                });
            } else {
                appendSshLog(`[SSH Tunnel] FAILED: ${data.message}`);
            }
        } catch (err) {
            appendSshLog(`[SSH Tunnel Error] Request failed: ${err.message}`);
        }
        btnSshTunnelStart.innerHTML = '<i class="fa-solid fa-link"></i> ESTABLISH TUNNEL';
        updateSshStatus();
    });
}

if (btnSshTunnelStop) {
    btnSshTunnelStop.addEventListener("click", async () => {
        appendSshLog(`[SSH Tunnel] Closing active tunnel connection...`);
        btnSshTunnelStop.disabled = true;
        btnSshTunnelStop.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> DISCONNECTING';

        try {
            const res = await fetch("/api/ssh/tunnel/stop", { method: "POST" });
            const data = await res.json();
            if (data.status === "success") {
                appendSshLog(`[SSH Tunnel] SUCCESS: ${data.message}`);
                // Remove the tunneled peer from GCS Global Connector
                await fetch("/api/peers/ssh-jetson", { method: "DELETE" });
            } else {
                appendSshLog(`[SSH Tunnel] FAILED: ${data.message}`);
            }
        } catch (err) {
            appendSshLog(`[SSH Tunnel Error] Request failed: ${err.message}`);
        }
        btnSshTunnelStop.innerHTML = '<i class="fa-solid fa-unlink"></i> CLOSE TUNNEL';
        updateSshStatus();
    });
}

if (btnSshLaunchBringup) {
    btnSshLaunchBringup.addEventListener("click", async () => {
        const config = getSshConfig();
        appendSshLog(`[SSH Remote] Launching ROS2 nodes on Jetson...`);
        btnSshLaunchBringup.disabled = true;
        btnSshLaunchBringup.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> LAUNCHING';

        try {
            const res = await fetch("/api/ssh/launch-bringup", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(config)
            });
            const data = await res.json();
            if (data.status === "success") {
                appendSshLog(`[SSH Remote] SUCCESS: ${data.message}`);
            } else {
                appendSshLog(`[SSH Remote] FAILED: ${data.message}`);
            }
        } catch (err) {
            appendSshLog(`[SSH Remote Error] Request failed: ${err.message}`);
        }
        btnSshLaunchBringup.disabled = false;
        btnSshLaunchBringup.innerHTML = '<i class="fa-solid fa-play"></i> START BRINGUP';
        updateSshStatus();
    });
}

if (btnSshStopBringup) {
    btnSshStopBringup.addEventListener("click", async () => {
        const config = getSshConfig();
        appendSshLog(`[SSH Remote] Killing ROS2 bringup on Jetson...`);
        btnSshStopBringup.disabled = true;
        btnSshStopBringup.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> STOPPING';

        try {
            const res = await fetch("/api/ssh/stop-bringup", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(config)
            });
            const data = await res.json();
            if (data.status === "success") {
                appendSshLog(`[SSH Remote] SUCCESS: ${data.message}`);
            } else {
                appendSshLog(`[SSH Remote] FAILED: ${data.message}`);
            }
        } catch (err) {
            appendSshLog(`[SSH Remote Error] Request failed: ${err.message}`);
        }
        btnSshStopBringup.disabled = false;
        btnSshStopBringup.innerHTML = '<i class="fa-solid fa-stop"></i> STOP BRINGUP';
        updateSshStatus();
    });
}

if (btnSshExecCmd) {
    btnSshExecCmd.addEventListener("click", async () => {
        const command = sshCmdInput ? sshCmdInput.value.trim() : "";
        if (!command) return;
        
        const config = getSshConfig();
        config.command = command;
        
        appendSshLog(`[SSH Exec] Running command: "${command}"...`);
        btnSshExecCmd.disabled = true;
        btnSshExecCmd.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> EXECUTING';

        try {
            const res = await fetch("/api/ssh/execute", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(config)
            });
            const data = await res.json();
            if (data.status === "success") {
                appendSshLog(`[SSH Exec Result] Success (code ${data.returncode}):\n${data.stdout || "(No stdout)"}`);
                if (data.stderr) {
                    appendSshLog(`[SSH Exec Stderr]:\n${data.stderr}`);
                }
            } else {
                appendSshLog(`[SSH Exec FAILED] Connection / auth error:\n${data.message || data.stderr}`);
            }
        } catch (err) {
            appendSshLog(`[SSH Exec Error] Request failed: ${err.message}`);
        }
        btnSshExecCmd.disabled = false;
        btnSshExecCmd.innerHTML = 'EXECUTE';
    });
}

async function updateSshStatus() {
    try {
        const res = await fetch("/api/ssh/status");
        const data = await res.json();
        
        if (sshTunnelStatusLbl) {
            sshTunnelStatusLbl.textContent = data.tunnel_active ? "CONNECTED" : "DISCONNECTED";
            sshTunnelStatusLbl.className = data.tunnel_active ? "dv ok" : "dv err";
        }
        if (sshNodesStatusLbl) {
            sshNodesStatusLbl.textContent = data.remote_bringup_active ? "ONLINE" : "OFFLINE";
            sshNodesStatusLbl.className = data.remote_bringup_active ? "dv ok" : "dv err";
        }
        
        if (btnSshTunnelStart) btnSshTunnelStart.disabled = data.tunnel_active;
        if (btnSshTunnelStop) btnSshTunnelStop.disabled = !data.tunnel_active;
    } catch (err) {
        console.error("Failed to query SSH status:", err);
    }
}

// Poll SSH status every 3 seconds
setInterval(updateSshStatus, 3000);
updateSshStatus();


// ─── SSE Connection with auto-reconnect ───
let _sseRetryDelay = 1000;  // ms, doubles on each failure, max 16s
let _sseActive = false;
let eventSource = null;

function connectSSE() {
    if (_sseActive) return;
    _sseActive = true;

    eventSource = new EventSource("/events");

    eventSource.onopen = () => {
        _sseRetryDelay = 1000;  // reset on successful connect
        if (connectionStatusDot) connectionStatusDot.className = "chip-dot d-online";
    };

    eventSource.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.connected) {
                const camPath = (gcsTopics && gcsTopics["image_recognition"] && gcsTopics["image_recognition"].path) ? gcsTopics["image_recognition"].path : "/rgb/image_raw/compressed";
                const streamUrl = `/api/topics${camPath}/stream`;
                if (!cameraImage.src.endsWith(streamUrl)) {
                    cameraImage.src = streamUrl;
                }
            } else {
                if (!cameraImage.src.endsWith("rover_camera_feed.jpg")) {
                    cameraImage.src = "rover_camera_feed.jpg";
                }
            }
            updateDashboard(data.telemetry, data.connected, data.peers, data.simulation_mode);
        } catch (e) {
            console.error("[SSE] Failed to parse event message:", e);
        }
    };


    eventSource.onerror = (err) => {
        console.warn("[SSE] Stream error — will reconnect in", _sseRetryDelay, "ms", err);
        updateDashboard(null, false, null);
        if (!cameraImage.src.endsWith("rover_camera_feed.jpg")) {
            cameraImage.src = "rover_camera_feed.jpg";
        }
        eventSource.close();
        _sseActive = false;
        setTimeout(connectSSE, _sseRetryDelay);
        _sseRetryDelay = Math.min(_sseRetryDelay * 2, 16000);
    };
}

connectSSE();

// ═══════════════════════════════════════════════════
// TACTICAL VIDEO HUD STREAM SIMULATION
// ═══════════════════════════════════════════════════

const videoCanvas = document.getElementById("video-hud-canvas");
const videoCtx = videoCanvas?.getContext("2d");

// Create background optical frame image using hidden in-DOM image for native MJPEG rendering
const cameraImage = document.getElementById("camera-stream-hidden") || new Image();
if (!cameraImage.src) {
    cameraImage.src = "rover_camera_feed.jpg";
}

let showHUD = true;
let isThermal = false;
let isNoiseFiltered = true;

// Wire control buttons
document.getElementById("btn-toggle-hud")?.addEventListener("click", (e) => {
    showHUD = !showHUD;
    e.target.classList.toggle("btn-dialog-primary");
});
document.getElementById("btn-toggle-thermal")?.addEventListener("click", (e) => {
    isThermal = !isThermal;
    e.target.classList.toggle("btn-dialog-primary");
});
document.getElementById("btn-toggle-noise")?.addEventListener("click", (e) => {
    isNoiseFiltered = !isNoiseFiltered;
    e.target.classList.toggle("btn-dialog-primary");
});

function updateCamTopicButtonState(path) {
    const btn = document.getElementById("btn-toggle-cam-topic");
    if (!btn) return;
    if (path === "/img") {
        btn.textContent = "STREAM: RAW IMAGE";
        btn.style.borderColor = "var(--cyan)";
        btn.style.color = "var(--cyan)";
    } else {
        btn.textContent = "STREAM: COMPRESSED";
        btn.style.borderColor = "var(--amber)";
        btn.style.color = "var(--amber)";
    }
}

document.getElementById("btn-toggle-cam-topic")?.addEventListener("click", async (e) => {
    e.preventDefault();
    const currentPath = (gcsTopics["image_recognition"] && gcsTopics["image_recognition"].path) 
        ? gcsTopics["image_recognition"].path 
        : "/rgb/image_raw/compressed";
    const newPath = (currentPath === "/rgb/image_raw/compressed") ? "/img" : "/rgb/image_raw/compressed";
    
    // Construct config payload
    const updatedPayload = JSON.parse(JSON.stringify(gcsTopics));
    if (!updatedPayload["image_recognition"]) {
        updatedPayload["image_recognition"] = { label: "Image Recognition", path: "/rgb/image_raw/compressed" };
    }
    updatedPayload["image_recognition"].path = newPath;
    
    try {
        const res = await fetch("/api/config/topics", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(updatedPayload)
        });
        
        if (res.ok) {
            // Update local state
            gcsTopics = updatedPayload;
            
            // Update UI button and config panel
            updateUIWithTopics();
            
            // Force immediate reload of the camera image source to point to the new stream endpoint!
            const streamUrl = `/api/topics${newPath}/stream`;
            cameraImage.src = streamUrl;
        }
    } catch (err) {
        console.error("Error toggling camera topic:", err);
    }
});

function drawTacticalHUD() {
    if (!videoCanvas || !videoCtx) return;

    // Sync canvas buffer size to match camera's natural source resolution to prevent blurry downscaling
    if (cameraImage.complete && cameraImage.naturalWidth > 0) {
        if (videoCanvas.width !== cameraImage.naturalWidth || videoCanvas.height !== cameraImage.naturalHeight) {
            videoCanvas.width = cameraImage.naturalWidth;
            videoCanvas.height = cameraImage.naturalHeight;
        }
    }

    const width = videoCanvas.width;
    const height = videoCanvas.height;
    
    // Clear canvas
    videoCtx.clearRect(0, 0, width, height);

    // Apply thermal optics filter
    if (isThermal) {
        // Cyan-orange thermal signature simulation filter
        videoCtx.filter = "contrast(1.6) brightness(1.2) invert(1) sepia(1) hue-rotate(200deg)";
    } else {
        videoCtx.filter = "none";
    }

    // Check if GCS is connected and telemetry is flowing, or if we have an active camera stream
    const isConnected = isTelemetryConnected;
    const hasCameraStream = cameraImage.complete && !cameraImage.src.endsWith("rover_camera_feed.jpg");

    if (hasCameraStream || (isConnected && cameraImage.complete)) {
        // Draw background primary camera optics feed
        videoCtx.drawImage(cameraImage, 0, 0, width, height);
    } else {
        // No connection: Draw tactical radar screen / noise
        videoCtx.fillStyle = "#03060a";
        videoCtx.fillRect(0, 0, width, height);
        
        // Draw signal noise static
        videoCtx.fillStyle = "rgba(0, 255, 102, 0.05)";
        for (let i = 0; i < 400; i++) {
            const x = Math.random() * width;
            const y = Math.random() * height;
            videoCtx.fillRect(x, y, 2, 2);
        }
        
        // Draw central searching message
        videoCtx.filter = "none";
        videoCtx.font = "bold 14px monospace";
        videoCtx.fillStyle = "rgba(239, 68, 68, 0.8)";
        videoCtx.textAlign = "center";
        videoCtx.fillText("🔴 SECURE_LINK: OFFLINE // SEARCHING...", width / 2, height / 2);
        
        requestAnimationFrame(drawTacticalHUD);
        return;
    }

    // Reset filter for HUD vector overlays to preserve neon color accuracy
    videoCtx.filter = "none";

    // Draw random signal noise glitches if noise filter is disabled
    if (!isNoiseFiltered) {
        videoCtx.fillStyle = "rgba(255, 255, 255, 0.08)";
        for (let i = 0; i < 150; i++) {
            videoCtx.fillRect(Math.random() * width, Math.random() * height, 3, 3);
        }
        if (Math.random() > 0.85) {
            // Screen scan glitch lines
            videoCtx.fillStyle = "rgba(0, 255, 102, 0.2)";
            videoCtx.fillRect(0, Math.random() * height, width, 2);
        }
    }

    if (showHUD) {
        const time = Date.now();
        const scanY = (time / 10) % (height + 100) - 50;

        // Draw animated scanning line
        if (scanY > 0 && scanY < height) {
            const grad = videoCtx.createLinearGradient(0, scanY - 15, 0, scanY + 2);
            grad.addColorStop(0, "rgba(0, 255, 102, 0)");
            grad.addColorStop(1, "rgba(0, 255, 102, 0.08)");
            videoCtx.fillStyle = grad;
            videoCtx.fillRect(0, 0, width, scanY);
            
            videoCtx.strokeStyle = "rgba(0, 255, 102, 0.4)";
            videoCtx.lineWidth = 1;
            videoCtx.beginPath();
            videoCtx.moveTo(0, scanY);
            videoCtx.lineTo(width, scanY);
            videoCtx.stroke();
        }

        // Draw Central Crosshair locking reticle
        videoCtx.strokeStyle = "rgba(0, 255, 102, 0.5)";
        videoCtx.lineWidth = 1.5;
        
        // Outer ring
        videoCtx.beginPath();
        videoCtx.arc(width / 2, height / 2, 35, 0, Math.PI * 2);
        videoCtx.setLineDash([4, 8]);
        videoCtx.stroke();
        videoCtx.setLineDash([]);

        // Center cursor
        videoCtx.beginPath();
        videoCtx.moveTo(width / 2 - 10, height / 2);
        videoCtx.lineTo(width / 2 - 2, height / 2);
        videoCtx.moveTo(width / 2 + 2, height / 2);
        videoCtx.lineTo(width / 2 + 10, height / 2);
        videoCtx.moveTo(width / 2, height / 2 - 10);
        videoCtx.lineTo(width / 2, height / 2 - 2);
        videoCtx.moveTo(width / 2, height / 2 + 2);
        videoCtx.lineTo(width / 2, height / 2 + 10);
        videoCtx.stroke();

        // Draw Target Bounding Box 1 (AI Waypoint Tracking)
        const odomTime = lastReceivedTime.odom;
        const isOdomLive = isTelemetryConnected && odomTime && (Date.now() - odomTime <= FRESHNESS_WINDOWS.odom);
        if (isOdomLive && latestTelemetryPayload && latestTelemetryPayload.Navigation && latestTelemetryPayload.Navigation.wp_current !== undefined) {
            const jitterX = Math.sin(time / 100) * 1.5;
            const jitterY = Math.cos(time / 120) * 1.5;
            const boxX = width * 0.38 + jitterX;
            const boxY = height * 0.42 + jitterY;
            const boxW = 55;
            const boxH = 45;

            videoCtx.strokeStyle = "#00ff66";
            videoCtx.lineWidth = 1.5;
            // Bounding brackets
            videoCtx.beginPath();
            // Top-left
            videoCtx.moveTo(boxX, boxY + 10);
            videoCtx.lineTo(boxX, boxY);
            videoCtx.lineTo(boxX + 10, boxY);
            // Top-right
            videoCtx.moveTo(boxX + boxW - 10, boxY);
            videoCtx.lineTo(boxX + boxW, boxY);
            videoCtx.lineTo(boxX + boxW, boxY + 10);
            // Bottom-left
            videoCtx.moveTo(boxX, boxY + boxH - 10);
            videoCtx.lineTo(boxX, boxY + boxH);
            videoCtx.lineTo(boxX + 10, boxY + boxH);
            // Bottom-right
            videoCtx.moveTo(boxX + boxW - 10, boxY + boxH);
            videoCtx.lineTo(boxX + boxW, boxY + boxH);
            videoCtx.lineTo(boxX + boxW, boxY + boxH - 10);
            videoCtx.stroke();

            // Tag label
            videoCtx.font = "8px monospace";
            videoCtx.fillStyle = "#00ff66";
            videoCtx.textAlign = "left";
            const wpStr = latestTelemetryPayload.Navigation.wp_current.toString().padStart(2, '0');
            videoCtx.fillText(`[TRG_A: WP_${wpStr}]`, boxX, boxY - 5);
        }

        // Draw Target Bounding Box 2 (Obstacle Avoidance)
        const safetyTime = lastReceivedTime.safety;
        const isSafetyLive = isTelemetryConnected && safetyTime && (Date.now() - safetyTime <= FRESHNESS_WINDOWS.safety);
        if (isSafetyLive && latestTelemetryPayload && latestTelemetryPayload.Safety && (latestTelemetryPayload.Safety.is_blocked || latestTelemetryPayload.Safety.collision_detected || latestTelemetryPayload.Safety.obstacle_touched)) {
            const box2X = width * 0.58;
            const box2Y = height * 0.38;
            const box2W = 45;
            const box2H = 45;
            
            videoCtx.strokeStyle = "rgba(255, 170, 0, 0.8)";
            videoCtx.beginPath();
            // Top-left
            videoCtx.moveTo(box2X, box2Y + 8);
            videoCtx.lineTo(box2X, box2Y);
            videoCtx.lineTo(box2X + 8, box2Y);
            // Bottom-right
            videoCtx.moveTo(box2X + box2W - 8, box2Y + box2H);
            videoCtx.lineTo(box2X + box2W, box2Y + box2H);
            videoCtx.lineTo(box2X + box2W, box2Y + box2H - 8);
            videoCtx.stroke();
            
            videoCtx.fillStyle = "rgba(255, 170, 0, 0.8)";
            videoCtx.fillText("[OBS_B: DETECTED]", box2X, box2Y - 5);
        }

        // Draw Dynamic HUD artificial horizon ladder (moves with heading/IMU pitch/roll)
        const headingStr = document.getElementById("heading-dial-val")?.textContent || "0";
        const heading = headingStr.includes("NO SIGNAL") ? 0 : parseFloat(headingStr);
        let roll = 0;
        let pitch = 0;
        const imuTime = lastReceivedTime.imu;
        const isImuLive = isTelemetryConnected && imuTime && (Date.now() - imuTime <= FRESHNESS_WINDOWS.imu);

        if (isImuLive && latestTelemetryPayload && latestTelemetryPayload.Sensors && latestTelemetryPayload.Sensors.imu) {
            const imu = latestTelemetryPayload.Sensors.imu;
            if (imu.orientation_x !== undefined) {
                const euler = quaternionToEuler(imu.orientation_x, imu.orientation_y, imu.orientation_z, imu.orientation_w);
                roll = euler.roll;
                pitch = euler.pitch;
            } else {
                pitch = Math.atan2(-imu.accel_x, Math.sqrt(imu.accel_y * imu.accel_y + imu.accel_z * imu.accel_z)) * 180 / Math.PI;
                roll = Math.atan2(imu.accel_y, imu.accel_z) * 180 / Math.PI;
            }
        }

        
        videoCtx.save();
        videoCtx.translate(width / 2, height / 2);
        videoCtx.rotate(roll * Math.PI / 180);
        
        videoCtx.strokeStyle = "rgba(0, 229, 255, 0.4)";
        videoCtx.lineWidth = 1;
        
        // Pitch lines
        const ladderY = pitch * 1.5;
        videoCtx.beginPath();
        // Left ladder bar
        videoCtx.moveTo(-70, -10 + ladderY);
        videoCtx.lineTo(-50, -10 + ladderY);
        videoCtx.lineTo(-50, ladderY);
        videoCtx.lineTo(-40, ladderY);
        // Right ladder bar
        videoCtx.moveTo(70, -10 + ladderY);
        videoCtx.lineTo(50, -10 + ladderY);
        videoCtx.lineTo(50, ladderY);
        videoCtx.lineTo(40, ladderY);
        videoCtx.stroke();
        
        videoCtx.restore();

        // Update timestamp display on overlay
        const timestampElem = document.getElementById("hud-timestamp");
        if (timestampElem) {
            const d = new Date();
            const pad = (n) => String(n).padStart(2, "0");
            const ms = String(d.getMilliseconds()).padStart(3, "0").slice(0, 2);
            timestampElem.textContent = `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}:${ms}`;
        }

        // Draw Handbrake / Drift overlay
        if (typeof handbrakeActive !== 'undefined' && handbrakeActive) {
            videoCtx.save();
            videoCtx.fillStyle = "rgba(255, 69, 0, 0.15)";
            videoCtx.fillRect(0, 0, width, height); // flash red/orange tint
            
            videoCtx.strokeStyle = "rgba(255, 69, 0, 0.8)";
            videoCtx.lineWidth = 3;
            videoCtx.strokeRect(10, 10, width - 20, height - 20); // border flash
            
            videoCtx.fillStyle = "rgba(255, 69, 0, 0.9)";
            videoCtx.font = "bold 13px monospace";
            videoCtx.textAlign = "center";
            videoCtx.fillText("⚡ TACTICAL DRIFT // HANDBRAKE ENGAGED ⚡", width / 2, height - 40);
            videoCtx.restore();
        }
    }

    requestAnimationFrame(drawTacticalHUD);
}

// ─── LIDAR Scan Rendering Loop ───
const scanCanvas = document.getElementById("lidar-scan-canvas");
const scanCtx = scanCanvas?.getContext("2d");

function drawLidarScan() {
    if (!scanCanvas || !scanCtx) return;
    const width = scanCanvas.width;
    const height = scanCanvas.height;
    
    // Grid and Circular scan centered at X=380, Y=110
    const centerX = 380;
    const centerY = 110;
    const maxRange = 12.0; // meters
    const radius = 95;
    const scale = radius / maxRange;

    // Clear canvas
    scanCtx.clearRect(0, 0, width, height);
    scanCtx.fillStyle = "#020408";
    scanCtx.fillRect(0, 0, width, height);

    // Draw Concentric Rings (3m, 6m, 9m, 12m)
    scanCtx.strokeStyle = "rgba(28, 35, 51, 0.4)";
    scanCtx.lineWidth = 1;
    const ringIntervals = [3, 6, 9, 12];
    ringIntervals.forEach((r) => {
        scanCtx.beginPath();
        scanCtx.arc(centerX, centerY, r * scale, 0, 2 * Math.PI);
        scanCtx.stroke();

        // Ring label
        scanCtx.fillStyle = "#6B7A99";
        scanCtx.font = "8px 'Share Tech Mono'";
        scanCtx.fillText(`${r}m`, centerX + r * scale - 12, centerY - 4);
    });

    // Draw Crosshairs
    scanCtx.beginPath();
    scanCtx.moveTo(centerX - radius, centerY);
    scanCtx.lineTo(centerX + radius, centerY);
    scanCtx.moveTo(centerX, centerY - radius);
    scanCtx.lineTo(centerX, centerY + radius);
    scanCtx.strokeStyle = "rgba(28, 35, 51, 0.3)";
    scanCtx.stroke();

    // Draw Angle reference lines (diagonal 45 deg lines)
    scanCtx.strokeStyle = "rgba(28, 35, 51, 0.15)";
    scanCtx.beginPath();
    scanCtx.moveTo(centerX - radius * 0.707, centerY - radius * 0.707);
    scanCtx.lineTo(centerX + radius * 0.707, centerY + radius * 0.707);
    scanCtx.moveTo(centerX - radius * 0.707, centerY + radius * 0.707);
    scanCtx.lineTo(centerX + radius * 0.707, centerY - radius * 0.707);
    scanCtx.stroke();

    // Draw Sweep line animation (adds high-tech look)
    const time = Date.now();
    const sweepAngle = (time / 1000) % (2 * Math.PI);
    scanCtx.beginPath();
    scanCtx.moveTo(centerX, centerY);
    scanCtx.lineTo(
        centerX + radius * Math.cos(sweepAngle),
        centerY + radius * Math.sin(sweepAngle)
    );
    scanCtx.strokeStyle = "rgba(0, 212, 255, 0.15)";
    scanCtx.lineWidth = 1.5;
    scanCtx.stroke();

    // Draw Robot Indicator in the center
    scanCtx.fillStyle = "#00D4FF";
    scanCtx.beginPath();
    // Draw a small triangle pointing up (since forward is up)
    scanCtx.moveTo(centerX, centerY - 6);
    scanCtx.lineTo(centerX - 4, centerY + 4);
    scanCtx.lineTo(centerX + 4, centerY + 4);
    scanCtx.closePath();
    scanCtx.fill();

    const statusBadge = document.getElementById("scan-view-status");

    // Sector Analysis ranges
    let fwdMin = null;
    let leftMin = null;
    let rightMin = null;

    // Check if we have active telemetry and scan data
    if (isTelemetryConnected && lastScanData && lastScanData.available) {
        if (statusBadge) {
            statusBadge.textContent = "ACTIVE";
            statusBadge.className = "ph-badge";
            statusBadge.style.color = "var(--cyan)";
            statusBadge.style.borderColor = "rgba(0, 212, 255, 0.2)";
            statusBadge.style.background = "rgba(0, 212, 255, 0.1)";
        }

        const ranges = lastScanData.ranges || [];
        const angleMin = lastScanData.angle_min_rad !== undefined ? lastScanData.angle_min_rad : -Math.PI;
        const angleMax = lastScanData.angle_max_rad !== undefined ? lastScanData.angle_max_rad : Math.PI;
        const numPoints = ranges.length;

        if (numPoints > 0) {
            // Plot points
            for (let i = 0; i < numPoints; i++) {
                const r = ranges[i];
                if (r !== null && r !== undefined && r > lastScanData.range_min_m) {
                    const angle = angleMin + i * (angleMax - angleMin) / (numPoints - 1);
                    
                    // Sector calculation
                    if (Math.abs(angle) < 0.5) { // ~30 deg
                        if (fwdMin === null || r < fwdMin) fwdMin = r;
                    } else if (angle >= 0.5 && angle < 2.0) { // left
                        if (leftMin === null || r < leftMin) leftMin = r;
                    } else if (angle <= -0.5 && angle > -2.0) { // right
                        if (rightMin === null || r < rightMin) rightMin = r;
                    }

                    // Canvas coordinates (forward is up, left is left)
                    const px = centerX - (r * scale) * Math.sin(angle);
                    const py = centerY - (r * scale) * Math.cos(angle);

                    // Color coding: red if very close in front
                    const isObstacleClose = (r < 1.5 && Math.abs(angle) < 0.4);
                    
                    scanCtx.fillStyle = isObstacleClose ? "#EF4444" : "#00D4FF";
                    scanCtx.beginPath();
                    scanCtx.arc(px, py, 1.5, 0, 2 * Math.PI);
                    scanCtx.fill();
                }
            }
        }

        // Draw Left Sector Info
        scanCtx.textAlign = "left";
        scanCtx.fillStyle = "#6B7A99";
        scanCtx.font = "10px 'Outfit'";
        scanCtx.fillText("SECTOR ANALYSIS", 20, 30);

        scanCtx.strokeStyle = "rgba(28, 35, 51, 0.4)";
        scanCtx.beginPath();
        scanCtx.moveTo(20, 38);
        scanCtx.lineTo(160, 38);
        scanCtx.stroke();

        const drawSectorMetric = (label, val, y) => {
            scanCtx.fillStyle = "#6B7A99";
            scanCtx.font = "9px 'Share Tech Mono'";
            scanCtx.fillText(label, 20, y);
            
            scanCtx.textAlign = "right";
            if (val !== null) {
                scanCtx.fillStyle = val < 1.5 ? "#EF4444" : "#00D4FF";
                scanCtx.fillText(`${val.toFixed(2)} m`, 160, y);
            } else {
                scanCtx.fillStyle = "#4A5568";
                scanCtx.fillText("CLEAR", 160, y);
            }
            scanCtx.textAlign = "left";
        };

        drawSectorMetric("FORWARD DETECT", fwdMin, 55);
        drawSectorMetric("LEFT SECTOR", leftMin, 75);
        drawSectorMetric("RIGHT SECTOR", rightMin, 95);

        // Draw Left Safety Warning Alert
        if (fwdMin !== null && fwdMin < 1.5) {
            scanCtx.fillStyle = "rgba(239, 68, 68, 0.1)";
            scanCtx.strokeStyle = "#EF4444";
            scanCtx.lineWidth = 1;
            scanCtx.fillRect(20, 115, 140, 35);
            scanCtx.strokeRect(20, 115, 140, 35);
            
            scanCtx.fillStyle = "#EF4444";
            scanCtx.font = "9px 'Share Tech Mono'";
            scanCtx.fillText("⚠️ COLLISION WARNING", 25, 128);
            scanCtx.font = "8px 'Share Tech Mono'";
            scanCtx.fillStyle = "#6B7A99";
            scanCtx.fillText("DECREASE SPEED IMMEDIATELY", 25, 142);
        } else {
            scanCtx.fillStyle = "rgba(16, 185, 129, 0.05)";
            scanCtx.strokeStyle = "rgba(16, 185, 129, 0.1)";
            scanCtx.lineWidth = 1;
            scanCtx.fillRect(20, 115, 140, 35);
            scanCtx.strokeRect(20, 115, 140, 35);
            
            scanCtx.fillStyle = "rgba(16, 185, 129, 0.8)";
            scanCtx.font = "9px 'Share Tech Mono'";
            scanCtx.fillText("✅ GEOFENCE NOMINAL", 25, 128);
            scanCtx.font = "8px 'Share Tech Mono'";
            scanCtx.fillStyle = "#6B7A99";
            scanCtx.fillText("NO OBSTACLES DETECTED", 25, 142);
        }

        // Draw Diagnostics on Right Column 1 (X = 500 to 640)
        scanCtx.textAlign = "left";
        scanCtx.fillStyle = "#6B7A99";
        scanCtx.font = "10px 'Outfit'";
        scanCtx.fillText("SENSOR DIAGNOSTICS", 500, 30);

        scanCtx.strokeStyle = "rgba(28, 35, 51, 0.4)";
        scanCtx.beginPath();
        scanCtx.moveTo(500, 38);
        scanCtx.lineTo(640, 38);
        scanCtx.stroke();

        const drawDiagMetric = (label, val, y) => {
            scanCtx.fillStyle = "#6B7A99";
            scanCtx.font = "9px 'Share Tech Mono'";
            scanCtx.fillText(label, 500, y);
            
            scanCtx.textAlign = "right";
            scanCtx.fillStyle = "#F0F6FF";
            scanCtx.fillText(String(val), 640, y);
            scanCtx.textAlign = "left";
        };

        const hz = lastScanData.num_points === 180 ? "10.0 Hz" : "30.0 Hz";
        drawDiagMetric("FRAME TARGET", lastScanData.frame_id || "laser_frame", 55);
        drawDiagMetric("SAMPLE RESOL", `${lastScanData.num_points || 0} pts`, 75);
        drawDiagMetric("VALID RETURNS", `${lastScanData.num_valid || 0} pts`, 95);
        drawDiagMetric("UPDATE RATE", isTelemetryConnected ? hz : "0.0 Hz", 115);
        drawDiagMetric("RANGE BOUNDS", `${lastScanData.range_min_m}m - ${lastScanData.range_max_m}m`, 135);

        // Draw standard topic info in Right Column 2 (X = 650 to 780)
        if (latestTelemetryPayload) {
            scanCtx.textAlign = "left";
            scanCtx.fillStyle = "#6B7A99";
            scanCtx.font = "10px 'Outfit'";
            scanCtx.fillText("STANDARD ROS 2 TOPICS", 650, 30);

            scanCtx.strokeStyle = "rgba(28, 35, 51, 0.4)";
            scanCtx.beginPath();
            scanCtx.moveTo(650, 38);
            scanCtx.lineTo(780, 38);
            scanCtx.stroke();

            const drawRosTopic = (label, val, y, state) => {
                scanCtx.fillStyle = "#6B7A99";
                scanCtx.font = "9px 'Share Tech Mono'";
                scanCtx.fillText(label, 650, y);
                
                scanCtx.textAlign = "right";
                if (state === "NO_SIGNAL") {
                    scanCtx.fillStyle = "#4A5568";
                } else if (state === "STALE") {
                    scanCtx.fillStyle = "#F59E0B";
                } else {
                    scanCtx.fillStyle = "#00D4FF";
                }
                scanCtx.fillText(val, 780, y);
                scanCtx.textAlign = "left";
            };

            const getFreshnessState = (topicKey) => {
                if (!isTelemetryConnected || lastReceivedTime[topicKey] === null) {
                    return "NO_SIGNAL";
                }
                const elapsed = Date.now() - lastReceivedTime[topicKey];
                if (elapsed > FRESHNESS_WINDOWS[topicKey]) {
                    return "STALE";
                }
                return "LIVE";
            };

            // 1. IMU status
            const imuState = getFreshnessState("imu");
            let imuVal = "OFFLINE";
            if (imuState !== "NO_SIGNAL" && latestTelemetryPayload.Sensors && latestTelemetryPayload.Sensors.imu && typeof latestTelemetryPayload.Sensors.imu.accel_x === "number" && typeof latestTelemetryPayload.Sensors.imu.accel_y === "number") {
                const imu = latestTelemetryPayload.Sensors.imu;
                imuVal = `${imu.accel_x.toFixed(2)},${imu.accel_y.toFixed(2)}`;
                if (imuState === "STALE") imuVal = "STALE";
            }
            drawRosTopic((gcsTopics.imu_accel && gcsTopics.imu_accel.label) || "IMU ACCEL", imuVal, 55, imuState);

            // 2. Odom position
            const odomState = getFreshnessState("odom");
            let odomVal = "OFFLINE";
            if (odomState !== "NO_SIGNAL" && latestTelemetryPayload.Odom && typeof latestTelemetryPayload.Odom.pos_x === "number" && typeof latestTelemetryPayload.Odom.pos_y === "number") {
                odomVal = `${latestTelemetryPayload.Odom.pos_x.toFixed(1)},${latestTelemetryPayload.Odom.pos_y.toFixed(1)} m`;
                if (odomState === "STALE") odomVal = "STALE";
            }
            drawRosTopic((gcsTopics.odom_coord && gcsTopics.odom_coord.label) || "ODOM COORD", odomVal, 75, odomState);

            // 3. CmdVel Echo
            const cmdVelState = getFreshnessState("cmdVel");
            let cmdVelVal = "OFFLINE";
            if (cmdVelState !== "NO_SIGNAL" && latestTelemetryPayload.CmdVelEcho && typeof latestTelemetryPayload.CmdVelEcho.linear_x === "number" && typeof latestTelemetryPayload.CmdVelEcho.angular_z === "number") {
                cmdVelVal = `${latestTelemetryPayload.CmdVelEcho.linear_x.toFixed(2)},${latestTelemetryPayload.CmdVelEcho.angular_z.toFixed(2)}`;
                if (cmdVelState === "STALE") cmdVelVal = "STALE";
            }
            drawRosTopic((gcsTopics.cmd_vel_echo && gcsTopics.cmd_vel_echo.label) || "CMD_VEL ECHO", cmdVelVal, 95, cmdVelState);

            // 4. GPS /fix status
            const gpsState = getFreshnessState("gps");
            let gpsVal = "OFFLINE";
            if (gpsState !== "NO_SIGNAL" && latestTelemetryPayload.GPS) {
                gpsVal = "ACTIVE FIX";
                if (gpsState === "STALE") gpsVal = "STALE";
            }
            drawRosTopic((gcsTopics.gps_fix && gcsTopics.gps_fix.label) || "GPS /FIX", gpsVal, 115, gpsState);

            // 5. Battery State
            const batState = getFreshnessState("battery");
            let batVal = "OFFLINE";
            if (batState !== "NO_SIGNAL" && latestTelemetryPayload.Battery && typeof latestTelemetryPayload.Battery.voltage === "number") {
                batVal = `${latestTelemetryPayload.Battery.voltage.toFixed(1)}V`;
                if (batState === "STALE") batVal = "STALE";
            }
            drawRosTopic((gcsTopics.battery_state && gcsTopics.battery_state.label) || "BATTERY STATE", batVal, 135, batState);

        }

    } else {
        if (statusBadge) {
            statusBadge.textContent = "OFFLINE";
            statusBadge.className = "ph-badge";
            statusBadge.style.color = "var(--text-secondary)";
            statusBadge.style.borderColor = "var(--border-dim)";
            statusBadge.style.background = "rgba(28, 35, 51, 0.15)";
        }
        
        // No Signal scan overlay text centered
        scanCtx.fillStyle = "rgba(107, 122, 153, 0.4)";
        scanCtx.font = "10px 'Share Tech Mono'";
        scanCtx.textAlign = "center";
        scanCtx.fillText("AWAITING LIDAR STREAM...", centerX, centerY + 30);
    }

    // Request next frame
    requestAnimationFrame(drawLidarScan);
}

// Start rendering loop when image loads
cameraImage.onload = () => {
    drawTacticalHUD();
};
// Fallback if image caching is immediate
if (cameraImage.complete) {
    drawTacticalHUD();
}

// Start LIDAR rendering loop immediately
requestAnimationFrame(drawLidarScan);

// ─── Keyboard Teleop / Manual Driving ───
const activeKeys = new Set();
let teleopInterval = null;
let keyboardTargetHeading = null;
let currentTeleopSpeed = 0.0;
let handbrakeActive = false;

const DRIVE_KEYS = ["KeyW", "KeyS", "KeyA", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "Space"];

function handleTeleopTick() {
    const maxSpeed = parseFloat(document.getElementById("ctrl-speed")?.value || "5.0");
    const throttle = parseFloat(document.getElementById("ctrl-throttle")?.value || "0.5");

    let driveForward = activeKeys.has("KeyW") || activeKeys.has("ArrowUp");
    let driveBackward = activeKeys.has("KeyS") || activeKeys.has("ArrowDown");
    let steerLeft = activeKeys.has("KeyA") || activeKeys.has("ArrowLeft");
    let steerRight = activeKeys.has("KeyD") || activeKeys.has("ArrowRight");
    let handbrake = activeKeys.has("Space");

    handbrakeActive = handbrake;

    // Check if we need to shut down the loop when no keys are pressed and speed is 0
    if (activeKeys.size === 0 && currentTeleopSpeed === 0.0) {
        if (teleopInterval) {
            clearInterval(teleopInterval);
            teleopInterval = null;
        }
        sendCommand({
            action: "stop",
            speed_kmh: 0.0,
            heading_deg: keyboardTargetHeading !== null ? keyboardTargetHeading : 0.0,
            throttle_pct: 0.0,
            source: "web_gcs_teleop",
            timestamp_ms: Date.now()
        });
        keyboardTargetHeading = null;
        return;
    }

    let active = false;

    // Throttle / Brake / Drift logic
    if (handbrake) {
        // Need for Speed drift physics: decelerate smoothly, do not drop to 0 instantly
        currentTeleopSpeed = Math.max(0.5, currentTeleopSpeed - 0.15);
        active = true;
    } else {
        if (driveForward) {
            // Smooth acceleration: add 0.4 km/h per tick (at 50ms, takes 0.5s to reach 4 km/h)
            currentTeleopSpeed = Math.min(maxSpeed, currentTeleopSpeed + 0.4);
            active = true;
        } else if (driveBackward) {
            // Smooth braking: subtract 0.8 km/h per tick (decelerates twice as fast as acceleration)
            currentTeleopSpeed = Math.max(0.0, currentTeleopSpeed - 0.8);
            active = true;
        } else {
            // Coasting deceleration: slow down naturally by 0.2 km/h per tick
            if (currentTeleopSpeed > 0.0) {
                currentTeleopSpeed = Math.max(0.0, currentTeleopSpeed - 0.2);
                active = true;
            }
        }
    }

    // Dynamically get the current actual heading of the rover
    const currentActualHeading = (latestTelemetryPayload?.Navigation?.heading_deg !== undefined)
        ? latestTelemetryPayload.Navigation.heading_deg
        : parseFloat(document.getElementById("ctrl-heading")?.value || "0.0");

    // If we are not actively steering, snap target heading immediately to actual heading (auto-centering)
    if (!steerLeft && !steerRight) {
        keyboardTargetHeading = currentActualHeading;
    } else {
        // We are steering! If keyboardTargetHeading was null, initialize it to actual heading
        if (keyboardTargetHeading === null) {
            keyboardTargetHeading = currentActualHeading;
        }
        
        // Smooth steering: turn target heading continuously when steer keys are held
        // Boost steering speed if handbrake is engaged (drifting)
        const steerRate = handbrake ? 5.0 : 2.5;
        if (steerLeft) {
            keyboardTargetHeading -= steerRate;
            active = true;
        } else if (steerRight) {
            keyboardTargetHeading += steerRate;
            active = true;
        }
        keyboardTargetHeading = (keyboardTargetHeading + 360.0) % 360.0;
    }

    // Send command if active (or speed is positive / handbrake is engaged)
    if (active || currentTeleopSpeed > 0.0 || handbrake) {
        // Update UI sliders for visual feedback
        const headingSlider = document.getElementById("ctrl-heading");
        if (headingSlider) {
            headingSlider.value = Math.round(keyboardTargetHeading);
        }
        const headingBubble = document.getElementById("heading-bubble");
        if (headingBubble) {
            headingBubble.textContent = Math.round(keyboardTargetHeading) + "°";
        }

        const isDrifting = handbrake && currentTeleopSpeed > 0.1;
        sendCommand({
            action: isDrifting ? "drive" : (handbrake ? "stop" : "drive"),
            speed_kmh: currentTeleopSpeed,
            heading_deg: keyboardTargetHeading,
            throttle_pct: handbrake ? 0.0 : throttle,
            source: "web_gcs_teleop",
            timestamp_ms: Date.now()
        });
    }
}

window.addEventListener("keydown", (e) => {
    // Ignore keystrokes if the user is typing in forms/input fields
    if (document.activeElement.tagName === "INPUT" || document.activeElement.tagName === "TEXTAREA") {
        return;
    }

    if (DRIVE_KEYS.includes(e.code)) {
        e.preventDefault();
        
        if (!activeKeys.has(e.code)) {
            activeKeys.add(e.code);
            
            if (!teleopInterval) {
                // Launch loop immediately and run at 50ms (20 Hz) for responsive NFS-like controls
                handleTeleopTick();
                teleopInterval = setInterval(handleTeleopTick, 50);
            }
        }
    }
});

window.addEventListener("keyup", (e) => {
    if (DRIVE_KEYS.includes(e.code)) {
        activeKeys.delete(e.code);
        
        // Handbrake reset immediately on keyup
        if (e.code === "Space") {
            handbrakeActive = false;
        }

        // The interval is NOT cleared here anymore.
        // Instead, handleTeleopTick will smoothly decelerate/coast speed to 0.0
        // and then clear itself automatically.
    }
});


// ─── PS5 / Gamepad Controller Support ───
let gamepadInterval = null;
let gamepadActive = false;
let prevGamepadButtons = {};
let gamepadTargetHeading = null;
let gamepadLastSentActive = false;
let smoothedLeftX = 0.0;
let smoothedRightY = 0.0;

function handleGamepad() {
    const gamepads = navigator.getGamepads();
    if (!gamepads) return;

    let gp = null;
    for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i] && gamepads[i].connected) {
            gp = gamepads[i];
            break;
        }
    }
    if (!gp) {
        if (gamepadActive) {
            gamepadActive = false;
            if (gamepadInterval) {
                clearInterval(gamepadInterval);
                gamepadInterval = null;
            }
            sendCommand({
                action: "stop",
                speed_kmh: 0.0,
                heading_deg: 0.0,
                throttle_pct: 0.0,
                source: "web_gcs_gamepad",
                timestamp_ms: Date.now()
            });
            const resultLbl = document.getElementById("command-result-lbl");
            if (resultLbl) {
                resultLbl.textContent = "🎮 GAMEPAD DISCONNECTED";
                resultLbl.style.color = "var(--amber)";
            }
        }
        return;
    }

    if (!gamepadActive) {
        gamepadActive = true;
        const resultLbl = document.getElementById("command-result-lbl");
        if (resultLbl) {
            resultLbl.textContent = "🎮 GAMEPAD CONNECTED";
            resultLbl.style.color = "var(--green-op)";
        }
        if (!gamepadInterval) {
            gamepadInterval = setInterval(handleGamepadTick, 50);
        }
    }
}

function handleGamepadTick() {
    const gamepads = navigator.getGamepads();
    let gp = null;
    for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i] && gamepads[i].connected) {
            gp = gamepads[i];
            break;
        }
    }
    if (!gp) return;

    // PS5 DualSense / W3C standard mapping:
    // axes[0]: Left stick X (-1 left, +1 right) -> steering
    // buttons[7]: R2 analog trigger (0.0 to 1.0) -> throttle
    // buttons[6]: L2 analog trigger (0.0 to 1.0) -> brake
    // buttons[1]: Circle button -> handbrake
    // buttons[14]: Touchpad click -> estop action

    const DEADZONE = 0.15;
    const gpLeftX = Math.abs(gp.axes[0]) > DEADZONE ? gp.axes[0] : 0;
    const r2 = gp.buttons[7] ? gp.buttons[7].value : 0.0;
    const l2 = gp.buttons[6] ? gp.buttons[6].value : 0.0;
    const handbrake = gp.buttons[1] ? gp.buttons[1].pressed : false;

    // Apply exponential smoothing for buttery Left stick X steering
    const alpha = 0.35;
    smoothedLeftX = alpha * gpLeftX + (1 - alpha) * smoothedLeftX;
    const leftX = Math.abs(smoothedLeftX) > 0.05 ? smoothedLeftX : 0;

    handbrakeActive = handbrake;

    const maxSpeed = parseFloat(document.getElementById("ctrl-speed")?.value || "5.0");
    const defaultThrottle = parseFloat(document.getElementById("ctrl-throttle")?.value || "0.5");

    // Need for Speed physics
    if (handbrake) {
        // Need for Speed drift physics: decelerate smoothly, do not drop to 0 instantly
        currentTeleopSpeed = Math.max(0.5, currentTeleopSpeed - 0.15);
    } else {
        if (r2 > 0.1) {
            // Smooth acceleration based on trigger pressure
            currentTeleopSpeed = Math.min(maxSpeed, currentTeleopSpeed + r2 * 0.4);
        } else if (l2 > 0.1) {
            // Smooth braking based on trigger pressure
            currentTeleopSpeed = Math.max(0.0, currentTeleopSpeed - l2 * 0.8);
        } else {
            // Coasting
            if (currentTeleopSpeed > 0.0) {
                currentTeleopSpeed = Math.max(0.0, currentTeleopSpeed - 0.2);
            }
        }
    }

    // Dynamically get the current actual heading of the rover
    const currentActualHeading = (latestTelemetryPayload?.Navigation?.heading_deg !== undefined)
        ? latestTelemetryPayload.Navigation.heading_deg
        : parseFloat(document.getElementById("ctrl-heading")?.value || "0.0");

    // If Left Joystick is centered, snap target heading immediately to actual heading (auto-centering)
    if (Math.abs(leftX) <= 0.05) {
        gamepadTargetHeading = currentActualHeading;
    } else {
        if (gamepadTargetHeading === null) {
            gamepadTargetHeading = currentActualHeading;
        }
        // Steering turn rate (dt = 50ms)
        // Boost steering speed if handbrake is engaged (drifting)
        const steerRate = handbrake ? 5.0 : 2.5;
        gamepadTargetHeading += leftX * steerRate; // Turn smoothly by up to 100 deg/sec
        gamepadTargetHeading = (gamepadTargetHeading + 360.0) % 360.0;
    }

    const activeThrottle = r2 > 0.1 ? Math.max(defaultThrottle, r2) : defaultThrottle;
    const hasInput = Math.abs(leftX) > 0.05 || r2 > 0.1 || l2 > 0.1 || handbrake || currentTeleopSpeed > 0.0;

    if (hasInput) {
        // Update UI sliders for visual feedback
        const headingSlider = document.getElementById("ctrl-heading");
        if (headingSlider) {
            headingSlider.value = Math.round(gamepadTargetHeading);
        }
        const headingBubble = document.getElementById("heading-bubble");
        if (headingBubble) {
            headingBubble.textContent = Math.round(gamepadTargetHeading) + "°";
        }
        const throttleInput = document.getElementById("ctrl-throttle");
        if (throttleInput) {
            throttleInput.value = Math.round(activeThrottle * 100) / 100;
        }

        const touchpadPressed = gp.buttons[14] ? gp.buttons[14].pressed : false;
        if (touchpadPressed && !prevGamepadButtons[14]) {
            sendCommand({
                action: "estop",
                speed_kmh: 0.0,
                heading_deg: gamepadTargetHeading,
                throttle_pct: 0.0,
                estop_triggered: true,
                source: "web_gcs_gamepad",
                timestamp_ms: Date.now()
            });
            const resultLbl = document.getElementById("command-result-lbl");
            if (resultLbl) {
                resultLbl.textContent = "🎮 GAMEPAD ESTOP!";
                resultLbl.style.color = "var(--red-alert)";
            }
        } else {
            const isDrifting = handbrake && currentTeleopSpeed > 0.1;
            sendCommand({
                action: isDrifting ? "drive" : (handbrake ? "stop" : "drive"),
                speed_kmh: Math.round(currentTeleopSpeed * 100) / 100,
                heading_deg: gamepadTargetHeading,
                throttle_pct: Math.round((handbrake ? 0.0 : activeThrottle) * 100) / 100,
                source: "web_gcs_gamepad",
                timestamp_ms: Date.now()
            });
        }
        gamepadLastSentActive = true;
    } else {
        if (gamepadLastSentActive) {
            sendCommand({
                action: "stop",
                speed_kmh: 0.0,
                heading_deg: gamepadTargetHeading,
                throttle_pct: 0.0,
                source: "web_gcs_gamepad",
                timestamp_ms: Date.now()
            });
            gamepadLastSentActive = false;
            gamepadTargetHeading = null;
        }
    }

    // Track button state for edge detection
    for (let i = 0; i < gp.buttons.length; i++) {
        prevGamepadButtons[i] = gp.buttons[i].pressed;
    }
}

// Poll for gamepad connection/disconnection
setInterval(handleGamepad, 1000);

window.addEventListener("gamepadconnected", (e) => {
    const gp = e.gamepad;
    const resultLbl = document.getElementById("command-result-lbl");
    if (resultLbl) {
        resultLbl.textContent = `🎮 ${gp.id} CONNECTED`;
        resultLbl.style.color = "var(--green-op)";
    }
    handleGamepad();
});

window.addEventListener("gamepaddisconnected", () => {
    gamepadActive = false;
    if (gamepadInterval) {
        clearInterval(gamepadInterval);
        gamepadInterval = null;
    }
    const resultLbl = document.getElementById("command-result-lbl");
    if (resultLbl) {
        resultLbl.textContent = "🎮 GAMEPAD DISCONNECTED";
        resultLbl.style.color = "var(--amber)";
    }
});

// ═══════════════════════════════════════════════════
// METADATA-DRIVEN TELEMETRY & WIDGET FACTORY SYSTEM
// ═══════════════════════════════════════════════════

// Inject toggle switch styling dynamically
const style = document.createElement("style");
style.textContent = `
    .switch-slider:before {
        position: absolute;
        content: "";
        height: 12px;
        width: 12px;
        left: 2px;
        bottom: 2px;
        background-color: var(--text-secondary);
        border-radius: 50%;
        transition: .2s;
    }
    input:checked + .switch-slider:before {
        transform: translateX(16px);
        background-color: var(--bg-void);
    }
    .topic-row {
        transition: border-color 0.2s ease, background 0.2s ease;
    }
    .topic-row:hover {
        border-color: rgba(0, 212, 255, 0.45) !important;
        background: rgba(0, 212, 255, 0.03) !important;
    }
    .widget-panel {
        margin-bottom: 0 !important;
        animation: widget-mount 0.25s ease-out;
    }
    @keyframes widget-mount {
        from { opacity: 0; transform: translateY(8px); }
        to { opacity: 1; transform: translateY(0); }
    }
`;
document.head.appendChild(style);

// Helper to flatten nested objects recursively
function flattenObject(ob) {
    var toReturn = {};
    for (var i in ob) {
        if (!ob.hasOwnProperty(i)) continue;
        if ((typeof ob[i]) == 'object' && ob[i] !== null && !Array.isArray(ob[i])) {
            var flatObject = flattenObject(ob[i]);
            for (var x in flatObject) {
                if (!flatObject.hasOwnProperty(x)) continue;
                toReturn[i + '.' + x] = flatObject[x];
            }
        } else {
            toReturn[i] = ob[i];
        }
    }
    return toReturn;
}

// ─── Custom Widget Class Implementations ───

class GenericFieldTable {
    constructor(topic) {
        this.topic = topic;
        this.element = document.createElement("div");
        this.element.className = "panel panel-cyan widget-panel";
        this.element.id = `widget-${topic.name.replace(/\//g, "-")}`;
        this.element.innerHTML = `
            <div class="ph">
                <div class="ph-title" style="overflow-wrap: anywhere;">${topic.name}</div>
                <span class="ph-badge" style="font-size: 8px;">${topic.type.split("/").pop()}</span>
            </div>
            <div class="pb" style="padding: 10px; max-height: 240px; overflow-y: auto;">
                <table class="setup-table" style="margin: 0; width: 100%;">
                    <tbody class="widget-tbody font-mono" style="font-size: 11px;">
                        <!-- Dynamic fields populated from schema -->
                    </tbody>
                </table>
            </div>
        `;
        this.tbody = this.element.querySelector(".widget-tbody");
        this.initFields(topic.fields);
    }
    
    initFields(fields, prefix = "") {
        if (!fields || fields.length === 0) {
            const tr = document.createElement("tr");
            tr.innerHTML = `
                <td style="color: var(--text-secondary); width: 50%;">data</td>
                <td class="dv font-mono" id="val-${this.topic.name.replace(/\//g, "-")}-data">--</td>
            `;
            this.tbody.appendChild(tr);
            return;
        }
        fields.forEach(f => {
            const key = prefix ? `${prefix}.${f.name}` : f.name;
            if (f.fields && f.fields.length > 0) {
                this.initFields(f.fields, key);
            } else {
                const tr = document.createElement("tr");
                tr.innerHTML = `
                    <td style="color: var(--text-secondary); width: 50%;">${key}</td>
                    <td class="dv font-mono" id="val-${this.topic.name.replace(/\//g, "-")}-${key.replace(/\./g, "-")}">--</td>
                `;
                this.tbody.appendChild(tr);
            }
        });
    }
    
    update(data) {
        const flat = flattenObject(data);
        if (typeof data !== "object" || data === null) {
            const cell = document.getElementById(`val-${this.topic.name.replace(/\//g, "-")}-data`);
            if (cell) {
                cell.textContent = data;
                cell.className = "dv ok";
            }
            return;
        }
        for (let key in flat) {
            const cell = document.getElementById(`val-${this.topic.name.replace(/\//g, "-")}-${key.replace(/\./g, "-")}`);
            if (cell) {
                let val = flat[key];
                if (typeof val === "number") {
                    val = Number.isInteger(val) ? val : val.toFixed(3);
                }
                cell.textContent = val;
                cell.className = "dv ok";
            }
        }
    }
}

class BatteryWidget {
    constructor(topic) {
        this.topic = topic;
        this.element = document.createElement("div");
        this.element.className = "panel panel-amber widget-panel";
        this.element.id = `widget-${topic.name.replace(/\//g, "-")}`;
        this.element.innerHTML = `
            <div class="ph">
                <div class="ph-title">${topic.name}</div>
                <span class="ph-badge">BATTERY</span>
            </div>
            <div class="pb" style="padding: 15px; display: flex; flex-direction: column; gap: 10px;">
                <div style="display: flex; justify-content: space-between; align-items: center;">
                    <span class="stat-lbl">VOLTAGE</span>
                    <span class="dv font-mono" id="bat-volt">-- V</span>
                </div>
                <div style="display: flex; justify-content: space-between; align-items: center;">
                    <span class="stat-lbl">PERCENTAGE</span>
                    <span class="dv font-mono" id="bat-pct">--%</span>
                </div>
                <div class="bat-wrap" style="width: 100%; height: 18px; border: 1px solid var(--border-dim); border-radius: 3px; padding: 2px;">
                    <div class="bat-fill" id="bat-fill-bar" style="width: 0%; height: 100%; background: var(--amber); transition: width 0.3s ease;"></div>
                </div>
            </div>
        `;
    }
    update(data) {
        const volt = data.voltage !== undefined ? data.voltage : (data.voltage_v !== undefined ? data.voltage_v : 0.0);
        const pct = data.percentage !== undefined ? data.percentage : 0.0;
        const pctVal = pct <= 1.0 ? pct * 100.0 : pct;
        
        const vCell = this.element.querySelector("#bat-volt");
        const pCell = this.element.querySelector("#bat-pct");
        const bar = this.element.querySelector("#bat-fill-bar");
        
        if (vCell) vCell.textContent = `${volt.toFixed(2)} V`;
        if (pCell) pCell.textContent = `${pctVal.toFixed(1)}%`;
        if (bar) {
            bar.style.width = `${pctVal}%`;
            if (pctVal > 50) bar.style.backgroundColor = "var(--green)";
            else if (pctVal > 20) bar.style.backgroundColor = "var(--amber)";
            else bar.style.backgroundColor = "var(--red-alert)";
        }
    }
}

class GpsWidget {
    constructor(topic) {
        this.topic = topic;
        this.element = document.createElement("div");
        this.element.className = "panel panel-cyan widget-panel";
        this.element.id = `widget-${topic.name.replace(/\//g, "-")}`;
        this.element.innerHTML = `
            <div class="ph">
                <div class="ph-title">${topic.name}</div>
                <span class="ph-badge">GPS FIX</span>
            </div>
            <div class="pb" style="padding: 15px; display: flex; flex-direction: column; gap: 8px;">
                <div style="display: flex; justify-content: space-between;">
                    <span class="stat-lbl">LATITUDE</span>
                    <span class="dv font-mono" id="gps-lat">--</span>
                </div>
                <div style="display: flex; justify-content: space-between;">
                    <span class="stat-lbl">LONGITUDE</span>
                    <span class="dv font-mono" id="gps-lon">--</span>
                </div>
                <div style="display: flex; justify-content: space-between;">
                    <span class="stat-lbl">ALTITUDE</span>
                    <span class="dv font-mono" id="gps-alt">-- m</span>
                </div>
            </div>
        `;
    }
    update(data) {
        const lat = data.latitude !== undefined ? data.latitude : 0.0;
        const lon = data.longitude !== undefined ? data.longitude : 0.0;
        const alt = data.altitude !== undefined ? data.altitude : (data.altitude_m !== undefined ? data.altitude_m : 0.0);
        
        this.element.querySelector("#gps-lat").textContent = lat.toFixed(6);
        this.element.querySelector("#gps-lon").textContent = lon.toFixed(6);
        this.element.querySelector("#gps-alt").textContent = alt.toFixed(1);
    }
}

class CameraViewer {
    constructor(topic) {
        this.topic = topic;
        this.element = document.createElement("div");
        this.element.className = "panel panel-green widget-panel";
        this.element.id = `widget-${topic.name.replace(/\//g, "-")}`;
        
        // Serve using the direct HTTP stream route we set up in Flask
        const streamUrl = `/api/topics${topic.name}/stream`;
        this.element.innerHTML = `
            <div class="ph">
                <div class="ph-title">${topic.name}</div>
                <span class="ph-badge">MJPEG STREAM</span>
            </div>
            <div class="pb" style="padding: 0; background: #000; display: flex; justify-content: center; align-items: center; min-height: 200px;">
                <img src="${streamUrl}" style="width: 100%; height: auto; display: block; border-bottom-left-radius: 4px; border-bottom-right-radius: 4px;" alt="Camera Stream">
            </div>
        `;
    }
    update(data) {}
}

class ScanViewer {
    constructor(topic) {
        this.topic = topic;
        this.element = document.createElement("div");
        this.element.className = "panel panel-cyan widget-panel";
        this.element.id = `widget-${topic.name.replace(/\//g, "-")}`;
        this.element.innerHTML = `
            <div class="ph">
                <div class="ph-title">${topic.name}</div>
                <span class="ph-badge">LIDAR SCAN</span>
            </div>
            <div class="pb" style="padding: 8px; background: #020408; display: flex; justify-content: center; align-items: center; flex-direction: column;">
                <canvas class="widget-scan-canvas" width="300" height="200" style="max-width: 100%; height: auto; border: 1px solid rgba(0,212,255,0.15); border-radius: 4px; background: #010306;"></canvas>
                <div style="font-size: 8px; font-family: 'Share Tech Mono'; color: var(--cyan); margin-top: 4px;" class="widget-scan-desc">
                    RANGES: -- POINTS
                </div>
            </div>
        `;
        this.canvas = this.element.querySelector(".widget-scan-canvas");
        this.ctx = this.canvas.getContext("2d");
        this.desc = this.element.querySelector(".widget-scan-desc");
    }
    update(data) {
        const ranges = data.ranges || [];
        this.desc.textContent = `RANGES: ${ranges.length} POINTS | MIN: ${data.range_min || 0.12}m`;
        
        const ctx = this.ctx;
        const w = this.canvas.width;
        const h = this.canvas.height;
        ctx.clearRect(0, 0, w, h);
        
        ctx.strokeStyle = "rgba(0, 212, 255, 0.08)";
        ctx.lineWidth = 1;
        for (let r = 30; r < w/2; r += 30) {
            ctx.beginPath();
            ctx.arc(w/2, h - 20, r, 0, 2 * Math.PI);
            ctx.stroke();
        }
        
        ctx.beginPath();
        ctx.moveTo(w/2, 0);
        ctx.lineTo(w/2, h);
        ctx.moveTo(0, h - 20);
        ctx.lineTo(w, h - 20);
        ctx.stroke();
        
        if (ranges.length === 0) return;
        ctx.fillStyle = "var(--cyan)";
        
        const angleMin = data.angle_min !== undefined && data.angle_min !== null ? data.angle_min : (data.angle_min_rad !== undefined ? data.angle_min_rad : -Math.PI);
        const angleInc = data.angle_increment !== undefined && data.angle_increment !== null ? data.angle_increment : (2 * Math.PI / ranges.length);
        const rangeMax = data.range_max !== undefined && data.range_max !== null ? data.range_max : (data.range_max_m !== undefined ? data.range_max_m : 12.0);
        const rangeMin = data.range_min !== undefined && data.range_min !== null ? data.range_min : 0.12;
        
        const scale = (w / 2 - 20) / rangeMax;
        const cx = w / 2;
        const cy = h - 20;
        
        for (let i = 0; i < ranges.length; i++) {
            const r = ranges[i];
            if (r <= rangeMin || r >= rangeMax) continue;
            
            const angle = angleMin + i * angleInc;
            const px = cx - (r * scale * Math.sin(angle));
            const py = cy - (r * scale * Math.cos(angle));
            
            ctx.fillRect(px - 1, py - 1, 2, 2);
        }
    }
}

class StatusWidget {
    constructor(topic) {
        this.topic = topic;
        this.element = document.createElement("div");
        this.element.className = "panel panel-blue widget-panel";
        this.element.id = `widget-${topic.name.replace(/\//g, "-")}`;
        this.element.innerHTML = `
            <div class="ph">
                <div class="ph-title">${topic.name}</div>
                <span class="ph-badge">STATUS</span>
            </div>
            <div class="pb" style="padding: 15px; display: flex; align-items: center; justify-content: space-between;">
                <span class="stat-lbl" style="margin-bottom:0;">STATE</span>
                <div style="display:flex; align-items:center; gap:8px;">
                    <span class="chip-dot d-offline" id="status-dot"></span>
                    <span class="dv err font-mono" id="status-lbl">UNKNOWN</span>
                </div>
            </div>
        `;
    }
    update(data) {
        const dot = this.element.querySelector("#status-dot");
        const lbl = this.element.querySelector("#status-lbl");
        
        let state = false;
        let labelStr = "OFFLINE";
        
        if (typeof data === "boolean") {
            state = data;
            labelStr = state ? "ACTIVE" : "INACTIVE";
        } else if (typeof data === "object" && data !== null) {
            if (data.data !== undefined) {
                state = !!data.data;
                labelStr = state ? "TRUE" : "FALSE";
            } else if (data.transition !== undefined && data.transition.label !== undefined) {
                state = true;
                labelStr = data.transition.label.toUpperCase();
            } else {
                state = true;
                labelStr = "CONNECTED";
            }
        }
        
        if (dot) dot.className = state ? "chip-dot d-online" : "chip-dot d-offline";
        if (lbl) {
            lbl.textContent = labelStr;
            lbl.className = state ? "dv ok font-mono" : "dv err font-mono";
        }
    }
}

class GaugeWidget {
    constructor(topic) {
        this.topic = topic;
        this.element = document.createElement("div");
        this.element.className = "panel panel-cyan widget-panel";
        this.element.id = `widget-${topic.name.replace(/\//g, "-")}`;
        this.element.innerHTML = `
            <div class="ph">
                <div class="ph-title">${topic.name}</div>
                <span class="ph-badge">GAUGE</span>
            </div>
            <div class="pb" style="padding: 15px; display: flex; flex-direction: column; gap: 8px;">
                <div style="display: flex; justify-content: space-between; align-items: center;">
                    <span class="stat-lbl" id="gauge-lbl">VALUE</span>
                    <span class="dv font-mono" id="gauge-val">--</span>
                </div>
                <div style="width: 100%; height: 6px; background: var(--bg-void); border-radius: 3px; overflow: hidden; border: 1px solid var(--border-dim);">
                    <div id="gauge-fill" style="width: 0%; height: 100%; background: var(--cyan); transition: width 0.2s ease;"></div>
                </div>
            </div>
        `;
    }
    update(data) {
        let val = 0.0;
        if (typeof data === "number") val = data;
        else if (typeof data === "object" && data !== null && data.data !== undefined) val = data.data;
        
        const limit = 100.0;
        const displayVal = typeof val === "number" ? val.toFixed(2) : val;
        const fillPct = Math.max(0, Math.min(100, (val / limit) * 100));
        
        this.element.querySelector("#gauge-val").textContent = displayVal;
        this.element.querySelector("#gauge-fill").style.width = `${fillPct}%`;
    }
}

class TextPanel {
    constructor(topic) {
        this.topic = topic;
        this.element = document.createElement("div");
        this.element.className = "panel panel-blue widget-panel";
        this.element.id = `widget-${topic.name.replace(/\//g, "-")}`;
        this.element.innerHTML = `
            <div class="ph">
                <div class="ph-title">${topic.name}</div>
                <span class="ph-badge">TEXT</span>
            </div>
            <div class="pb" style="padding: 12px;">
                <pre style="margin: 0; padding: 6px; background: var(--bg-void); border: 1px solid var(--border-dim); font-family: var(--font-mono); font-size: 10px; color: var(--cyan); line-height: 1.4; word-break: break-all; white-space: pre-wrap;" id="text-box">Awaiting data...</pre>
            </div>
        `;
    }
    update(data) {
        let text = "";
        if (typeof data === "string") text = data;
        else if (typeof data === "object" && data !== null) {
            if (data.data !== undefined) text = data.data;
            else text = JSON.stringify(data, null, 2);
        }
        this.element.querySelector("#text-box").textContent = text;
    }
}


// ─── Component Widget Registry Mapping ───

const WIDGET_MAP = {
    "sensor_msgs/msg/BatteryState": BatteryWidget,
    "sensor_msgs/msg/NavSatFix": GpsWidget,
    "sensor_msgs/msg/LaserScan": ScanViewer,
    "sensor_msgs/msg/CompressedImage": CameraViewer,
    "std_msgs/msg/Bool": StatusWidget,
    "std_msgs/msg/Float64": GaugeWidget,
    "std_msgs/msg/String": TextPanel,
    "lifecycle_msgs/msg/TransitionEvent": StatusWidget,
};

// ─── Socket.IO Handler and lazy mount execution ───

let socket;
if (typeof io !== "undefined") {
    socket = io();
} else {
    console.warn("Socket.IO client library not loaded. Falling back to mock socket.");
    socket = {
        emit: (event, data) => console.log("[Mock Socket] emit:", event, data),
        on: (event, cb) => console.log("[Mock Socket] register on:", event)
    };
}

const mountedWidgets = {};

function mountWidget(topic) {
    if (mountedWidgets[topic.name]) return;
    
    // Hide empty state indicator
    const emptyState = document.getElementById("widgets-empty-state");
    if (emptyState) emptyState.style.display = "none";
    
    const WidgetClass = WIDGET_MAP[topic.type] || GenericFieldTable;
    const widget = new WidgetClass(topic);
    
    document.getElementById("active-widgets-grid").appendChild(widget.element);
    mountedWidgets[topic.name] = widget;
    
    // Join room (lazy subscription)
    socket.emit("join_topic", { topic: topic.name });
}

function unmountWidget(topicName) {
    const widget = mountedWidgets[topicName];
    if (!widget) return;
    
    // Leave room
    socket.emit("leave_topic", { topic: topicName });
    
    widget.element.remove();
    delete mountedWidgets[topicName];
    
    // If no widgets are left, show empty state
    if (Object.keys(mountedWidgets).length === 0) {
        const emptyState = document.getElementById("widgets-empty-state");
        if (emptyState) emptyState.style.display = "block";
    }
}

// ─── API TopicStore Loader ───

async function initTopicsTab() {
    const listContainer = document.getElementById("topics-sidebar-list");
    const countBadge = document.getElementById("topics-count-badge");
    if (!listContainer) return;
    
    try {
        const res = await fetch("/api/topics");
        const topics = await res.json();
        
        countBadge.textContent = `${topics.length} topics`;
        listContainer.innerHTML = "";
        
        topics.forEach(topic => {
            const row = document.createElement("div");
            row.className = "topic-row";
            row.style.display = "flex";
            row.style.flexDirection = "column";
            row.style.gap = "4px";
            row.style.padding = "8px";
            row.style.background = "var(--bg-panel)";
            row.style.border = "1px solid var(--border-dim)";
            row.style.borderRadius = "4px";
            
            const isError = topic.connection_state.startsWith("error");
            const statusClass = topic.connection_state === "connected" ? "ok" : (isError ? "err" : "warn");
            const statusText = topic.connection_state.toUpperCase();
            
            row.innerHTML = `
                <div style="display: flex; justify-content: space-between; align-items: start;">
                    <div style="font-family: var(--font-mono); font-size: 11px; font-weight: bold; color: var(--cyan); overflow-wrap: anywhere; max-width: 230px;">
                        ${topic.name}
                    </div>
                    <label class="switch-container" style="display: inline-block; position: relative; width: 34px; height: 18px; margin-bottom:0;">
                        <input type="checkbox" class="topic-toggle-checkbox" data-topic="${topic.name}" ${isError ? "disabled" : ""} style="opacity: 0; width: 0; height: 0;">
                        <span class="switch-slider" style="position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: rgba(255,255,255,0.08); border: 1px solid var(--border-dim); border-radius: 9px; transition: .2s;"></span>
                    </label>
                </div>
                <div style="display: flex; justify-content: space-between; font-size: 9px; color: var(--text-secondary); font-family: var(--font-mono);">
                    <span>${topic.type.split("/").pop()}</span>
                    <span class="dv ${statusClass}">${statusText}</span>
                </div>
            `;
            
            const checkbox = row.querySelector(".topic-toggle-checkbox");
            const slider = row.querySelector(".switch-slider");
            checkbox.addEventListener("change", (e) => {
                if (e.target.checked) {
                    slider.style.backgroundColor = "var(--green)";
                    slider.style.borderColor = "var(--green)";
                    mountWidget(topic);
                } else {
                    slider.style.backgroundColor = "rgba(255,255,255,0.08)";
                    slider.style.borderColor = "var(--border-dim)";
                    unmountWidget(topic.name);
                }
            });
            
            listContainer.appendChild(row);
        });
    } catch (err) {
        console.error("Failed to load topic registry:", err);
    }
}

// Connect the Socket.IO listener callback
socket.on("topic_update", (payload) => {
    const widget = mountedWidgets[payload.topic];
    if (widget) {
        widget.update(payload.data);
    }
});

// Binary MessagePack telemetry update listener
socket.on("telemetry_binary_update", (binaryData) => {
    try {
        // Decode messagepack binary payload
        const data = msgpack.decode(new Uint8Array(binaryData));
        
        // Deactivate SSE if it is running because we are successfully receiving binary updates
        if (_sseActive && eventSource) {
            console.log("🎮 Successfully upgraded to binary WebSockets. Closing SSE...");
            eventSource.close();
            eventSource = null;
            _sseActive = false;
        }
        
        // Set camera stream source dynamically if connected
        if (data.connected) {
            const camPath = (gcsTopics && gcsTopics["image_recognition"] && gcsTopics["image_recognition"].path) ? gcsTopics["image_recognition"].path : "/rgb/image_raw/compressed";
            const streamUrl = `/api/topics${camPath}/stream`;
            if (!cameraImage.src.endsWith(streamUrl)) {
                cameraImage.src = streamUrl;
            }
        } else {
            if (!cameraImage.src.endsWith("rover_camera_feed.jpg")) {
                cameraImage.src = "rover_camera_feed.jpg";
            }
        }
        
        // Update the GCS dashboard
        updateDashboard(data.telemetry, data.connected, data.peers, data.simulation_mode);
    } catch (err) {
        console.error("Failed to decode binary telemetry payload:", err);
    }
});

// Handle WebSocket connection events to coordinate SSE fallback
if (typeof io !== "undefined" && socket) {
    socket.on("connect", () => {
        console.log("🎮 WebSocket connection established.");
    });
    
    socket.on("disconnect", () => {
        console.warn("🔌 WebSocket disconnected. Reverting to Server-Sent Events (SSE) telemetry fallback...");
        connectSSE();
    });
}

// Trigger registry fetch on boot
document.addEventListener("DOMContentLoaded", () => {
    initTopicsTab();
});

