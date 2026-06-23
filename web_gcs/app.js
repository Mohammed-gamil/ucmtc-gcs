// app.js — UCMTC Tactical GCS

// ─── DOM Element Cache ───
const telemetryStateLbl = document.getElementById("telemetry-state-lbl");
const safetyModeLbl     = document.getElementById("safety-mode-lbl");
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
const navDistance = document.getElementById("nav-distance");
const navPosition = document.getElementById("nav-position");
const navWaypoint = document.getElementById("nav-waypoint");

// Compute
const cpuLbl    = document.getElementById("cpu-lbl");
const cpuFill   = document.getElementById("cpu-fill");
const gpuLbl    = document.getElementById("gpu-lbl");
const gpuFill   = document.getElementById("gpu-fill");
const ramLbl    = document.getElementById("ram-lbl");
const ramFill   = document.getElementById("ram-fill");
const tempLbl   = document.getElementById("temp-lbl");
const uptimeLbl = document.getElementById("uptime-lbl");

// ROS Nodes
const nodeLane   = document.getElementById("node-lane");
const nodeAvoid  = document.getElementById("node-avoid");
const nodeNav    = document.getElementById("node-nav");
const nodeVision = document.getElementById("node-vision");
const nodeMotor  = document.getElementById("node-motor");
const rosoutLbl  = document.getElementById("rosout-lbl");

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

// Configure Real-time Chart.js
const ctx = document.getElementById("realtime-chart").getContext("2d");
const maxDataPoints = 30;
const chartLabels = Array(maxDataPoints).fill("");
const cpuData = Array(maxDataPoints).fill(0);
const tempData = Array(maxDataPoints).fill(0);
const lossData = Array(maxDataPoints).fill(0);

const telemetryChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: chartLabels,
        datasets: [
            {
                label: 'CPU %',
                data: cpuData,
                borderColor: '#a855f7',
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                tension: 0.2
            },
            {
                label: 'Temp °C',
                data: tempData,
                borderColor: '#3b82f6',
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                tension: 0.2
            },
            {
                label: 'Loss %',
                data: lossData,
                borderColor: '#ef4444',
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                tension: 0.2
            }
        ]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
            legend: {
                display: true,
                position: 'top',
                labels: {
                    color: '#9ca3af',
                    font: { size: 9, family: 'Space Grotesk' },
                    boxWidth: 8,
                    padding: 6
                }
            }
        },
        scales: {
            x: { display: false },
            y: {
                min: 0,
                max: 100,
                grid: { color: 'rgba(255, 255, 255, 0.04)' },
                ticks: {
                    color: '#6b7280',
                    font: { size: 8 }
                }
            }
        }
    }
});

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

function renderPeerList(peers) {
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
        peerAddResult.style.color = "var(--neon-red)";
        return;
    }

    peerAddResult.textContent = "⏳ Connecting to peer...";
    peerAddResult.style.color = "var(--neon-mesh)";

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
            peerAddResult.style.color = "var(--neon-green)";
            // Clear form
            document.getElementById("peer-id-input").value = "";
            document.getElementById("peer-ip-input").value = "";
            document.getElementById("peer-port-input").value = "8090";
            document.getElementById("peer-team-input").value = "";
        } else {
            peerAddResult.textContent = `❌ ${result.message}`;
            peerAddResult.style.color = "var(--neon-red)";
        }
    } catch (err) {
        peerAddResult.textContent = `❌ Network error: ${err.message}`;
        peerAddResult.style.color = "var(--neon-red)";
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
            peerAddResult.style.color = "var(--neon-amber)";
        }
    } catch (err) {
        console.error("Failed to remove peer:", err);
    }
}

// Update DOM elements from telemetry payload
function updateDashboard(payload, connected, peers) {
    if (!connected || !payload) {
        telemetryStateLbl.textContent = "OFFLINE";
        connectionStatusDot.className = "chip-dot d-offline";
        safetyModeLbl.textContent = "ESTOP";
        safetyStatusDot.className = "chip-dot d-offline";
        alertBanner.className = "alert-banner alert-mode";
        alertContent.innerHTML = "🔴 CRITICAL: TELEMETRY DISCONNECTED — Heartbeat stream offline!";
        if (telemPulse) telemPulse.className = "telemetry-pulse disconnected";
        if (signalStrength) signalStrength.className = "signal-bars";
        return;
    }


    // Normal connection
    telemetryStateLbl.textContent = "CONNECTED";
    connectionStatusDot.className = "chip-dot d-online";
    if (telemPulse) telemPulse.className = "telemetry-pulse";


    const nav    = payload.Navigation;
    const safety = payload.Safety;
    const vision = payload.Vision;
    const jetson = payload.Jetson;
    const comm   = payload.Communication;
    const ros    = payload.ROS;

    // Safety state dot
    if (safety.estop_triggered || safety.collision_detected) {
        safetyModeLbl.textContent = "ALERT";
        safetyStatusDot.className = "chip-dot d-offline";
    } else {
        safetyModeLbl.textContent = safety.mode.toUpperCase();
        safetyStatusDot.className = `chip-dot ${safety.mode === 'monitoring' ? 'd-online' : 'd-idle'}`;
    }

    // Top Header Readouts
    rttLbl.textContent = comm.rtt_ms + " ms";
    batPctLbl.textContent = jetson.bat_pct.toFixed(1) + "%";
    batBar.style.width = jetson.bat_pct + "%";
    if (jetson.bat_pct < 20) {
        batBar.style.backgroundColor = "var(--neon-red)";
    } else if (jetson.bat_pct < 50) {
        batBar.style.backgroundColor = "var(--neon-amber)";
    } else {
        batBar.style.backgroundColor = "var(--neon-green)";
    }

    // Subsystem alerts analysis
    const warnings = [];
    if (safety.estop_triggered) warnings.push("EMERGENCY STOP TRIGGERED");
    if (safety.collision_detected) warnings.push("COLLISION DETECTED");
    if (safety.border_crossed) warnings.push("GEOFENCE BORDER BREACHED");
    if (jetson.temp_c > 75) warnings.push(`CPU TEMP CRITICAL (${jetson.temp_c}°C)`);
    if (jetson.bat_pct < 15) warnings.push("LOW BATTERY DETECTED");
    if (comm.packet_loss_pct > 8) warnings.push(`HIGH PACKET LOSS (${comm.packet_loss_pct}%)`);
    if (comm.rtt_ms > 150) warnings.push("HIGH LATENCY DELAY");

    const deadNodes = [];
    if (!ros.node_lane_det) deadNodes.push("Lane");
    if (!ros.node_obs_avoid) deadNodes.push("Safety");
    if (!ros.node_wp_nav) deadNodes.push("Nav");
    if (!ros.node_motor_ctrl) deadNodes.push("Motor");
    if (deadNodes.length > 0) warnings.push(`ROS NODE FAIL (${deadNodes.join(",")})`);

    if (warnings.length > 0) {
        alertBanner.className = "alert-banner alert-mode";
        alertContent.innerHTML = "⚠️ ALERT: " + warnings.join(" | ");
    } else {
        alertBanner.className = "alert-banner";
        alertContent.innerHTML = "🟢 SYSTEM NOMINAL — All metrics within safe operating parameters";
    }

    // Compass needle rotation (SVG group)
    if (compassNeedle) {
        compassNeedle.style.transform = `rotate(${nav.heading_deg}deg)`;
    }
    headingDialVal.textContent = nav.heading_deg.toFixed(1) + "°";
    headingCardinal.textContent = getCardinalDirection(nav.heading_deg);

    // Navigation kinematics
    navSpeed.textContent = nav.speed_kmh.toFixed(2) + " km/h";
    navDistance.textContent = nav.dist_traveled_m.toFixed(1) + " m";
    navPosition.textContent = `${nav.pos_lat.toFixed(6)}, ${nav.pos_lon.toFixed(6)}`;
    navWaypoint.textContent = `WP ${nav.wp_current} (${nav.wp_status.toUpperCase()}) (err: ${nav.wp_error_m.toFixed(2)}m)`;

    // Onboard Compute Bars
    cpuLbl.textContent = jetson.cpu_pct.toFixed(1) + "%";
    cpuFill.style.width = jetson.cpu_pct + "%";
    gpuLbl.textContent = jetson.gpu_pct.toFixed(1) + "%";
    gpuFill.style.width = jetson.gpu_pct + "%";
    ramLbl.textContent = jetson.ram_pct.toFixed(1) + "%";
    ramFill.style.width = jetson.ram_pct + "%";
    tempLbl.textContent = jetson.temp_c.toFixed(1) + " °C";
    uptimeLbl.textContent = jetson.uptime_sec + " sec";

    // Chart.js updates (shift metrics arrays)
    cpuData.push(jetson.cpu_pct);
    cpuData.shift();
    tempData.push(jetson.temp_c);
    tempData.shift();
    lossData.push(comm.packet_loss_pct * 10); // scale up for visual visibility
    lossData.shift();
    telemetryChart.update('none'); // update without animations for efficiency

    // ROS node grid (new class names)
    const updateNodeStatus = (elem, alive) => {
        const dot = elem.querySelector(".ndot");
        if (alive) {
            dot.className = "ndot on";
            elem.className = "node-pill alive";
        } else {
            dot.className = "ndot off";
            elem.className = "node-pill dead";
        }
    };
    updateNodeStatus(nodeLane,   ros.node_lane_det);
    updateNodeStatus(nodeAvoid,  ros.node_obs_avoid);
    updateNodeStatus(nodeNav,    ros.node_wp_nav);
    updateNodeStatus(nodeVision, ros.node_img_recog);
    updateNodeStatus(nodeMotor,  ros.node_motor_ctrl);
    rosoutLbl.textContent = ros.rosout_last;

    // Update Setup Tab ROS Subsystem Checklist
    const setNodeRow = (dotId, lblId, hbId, alive) => {
        const dotElem = document.getElementById(dotId);
        const lblElem = document.getElementById(lblId);
        const hbElem = document.getElementById(hbId);
        if (dotElem && lblElem && hbElem) {
            if (alive) {
                dotElem.className = "chip-dot d-online";
                lblElem.textContent = "ONLINE";
                lblElem.className = "dv ok";
                hbElem.textContent = jetson.uptime_sec ? `${jetson.uptime_sec}s` : "active";
            } else {
                dotElem.className = "chip-dot d-offline";
                lblElem.textContent = "OFFLINE";
                lblElem.className = "dv err";
                hbElem.textContent = "--";
            }
        }
    };
    setNodeRow("setup-node-motor-dot", "setup-node-motor-lbl", "setup-node-motor-heartbeat", ros.node_motor_ctrl);
    setNodeRow("setup-node-lane-dot", "setup-node-lane-lbl", "setup-node-lane-heartbeat", ros.node_lane_det);
    setNodeRow("setup-node-avoid-dot", "setup-node-avoid-lbl", "setup-node-avoid-heartbeat", ros.node_obs_avoid);
    setNodeRow("setup-node-nav-dot", "setup-node-nav-lbl", "setup-node-nav-heartbeat", ros.node_wp_nav);
    setNodeRow("setup-node-vision-dot", "setup-node-vision-lbl", "setup-node-vision-heartbeat", ros.node_img_recog);
    setNodeRow("setup-node-telem-dot", "setup-node-telem-lbl", "setup-node-telem-heartbeat", true);

    // Safety Diagnostics
    const setBoolCell = (elem, val, invert = false) => {
        const check = invert ? !val : val;
        if (check) {
            elem.textContent = "CLEAR";
            elem.className = "dv ok";
        } else {
            elem.textContent = invert ? "ARMED" : "DETECTED";
            elem.className = "dv err";
        }
    };


    // E-stops are armed when false (inverted logic)
    setBoolCell(safeEstopMech, !safety.estop_mech_armed, true);
    setBoolCell(safeEstopWire, !safety.estop_wire_armed, true);
    setBoolCell(safeEstopTrig, !safety.estop_triggered, true);

    setBoolCell(safeBlocked,      !safety.is_blocked);
    setBoolCell(safeTouched,      !safety.obstacle_touched);
    setBoolCell(safeCollision,    !safety.collision_detected);
    setBoolCell(safeBorderCrossed, !safety.border_crossed);
    setBoolCell(safeBorderPartial, !safety.border_partial);

    // Vision Metrics
    const confOffset = 251.2 - (vision.img_confidence * 251.2);
    confDial.style.strokeDasharray = `${vision.img_confidence * 251.2}, 251.2`;
    visionConfPct.textContent = Math.round(vision.img_confidence * 100) + "%";
    visionStatusLbl.textContent = vision.img_task_status.toUpperCase();
    
    visionLane.textContent = vision.lane_detected ? "YES" : "NO";
    visionLane.className = vision.lane_detected ? "dv ok" : "dv err";
    visionObstacles.textContent = vision.obstacles_count + " detected";
    visionLaser.textContent = vision.laser_active ? "EMITTING" : "STANDBY";
    visionLaser.className = vision.laser_active ? "dv ok" : "dv";
    visionFps.textContent = vision.fps_vision.toFixed(1) + " FPS";
    visionUptime.textContent = vision.img_elapsed_sec + " sec";

    // Link Details
    linkRssi.textContent = comm.channel_rssi + " dBm";
    linkRssi.className = comm.channel_rssi < -75 ? "dv rssi-val err" : "dv rssi-val ok";
    linkLoss.textContent = comm.packet_loss_pct.toFixed(1) + " %";
    linkFps.textContent = comm.stream_fps.toFixed(1) + " FPS";
    linkHeartbeat.textContent = "#" + comm.heartbeat_seq;

    // Update signal strength bars
    if (signalStrength) {
        if (comm.channel_rssi >= -60) {
            signalStrength.className = "signal-bars signal-good";
        } else if (comm.channel_rssi >= -75) {
            signalStrength.className = "signal-bars signal-fair";
        } else {
            signalStrength.className = "signal-bars signal-bad";
        }
    }


    // Update peer list from SSE data
    if (peers) {
        renderPeerList(peers);
    }
}

// POST dispatch command
async function sendCommand(command) {
    commandResultLbl.textContent = "Sending command...";
    commandResultLbl.style.color = "var(--text-secondary)";
    try {
        const response = await fetch("/command", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(command)
        });
        const res = await response.json();
        if (res.status === "success") {
            commandResultLbl.textContent = `Success: Dispatched ${command.action.toUpperCase()} command`;
            commandResultLbl.style.color = "var(--neon-green)";
        } else {
            commandResultLbl.textContent = `Error: ${res.message}`;
            commandResultLbl.style.color = "var(--neon-red)";
        }
    } catch (err) {
        commandResultLbl.textContent = `Network Error: ${err.message}`;
        commandResultLbl.style.color = "var(--neon-red)";
    }
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

if (btnApplyNetwork) {
    btnApplyNetwork.addEventListener("click", () => {
        const domain = setupDomainInput ? setupDomainInput.value : "0";
        const discovery = setupDiscoverySelect ? setupDiscoverySelect.value : "SUBNET";
        const peerId = setupPeerIdInput ? setupPeerIdInput.value : "gcs-operator";
        const localIp = setupIpInput ? setupIpInput.value : "127.0.0.1";

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
    });
}

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

        appendDiagLog(` - Motor Control Node: ${nodeMotorCtrl ? 'ONLINE' : 'OFFLINE (Critical)'}`);
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


// Connect to EventSource stream
const eventSource = new EventSource("/events");

eventSource.onmessage = (event) => {
    try {
        const data = JSON.parse(event.data);
        updateDashboard(data.telemetry, data.connected, data.peers);
    } catch (e) {
        console.error("Failed to parse event message:", e);
    }
};

eventSource.onerror = (err) => {
    console.error("SSE stream error:", err);
    updateDashboard(null, false, null);
};

// ═══════════════════════════════════════════════════
// TACTICAL VIDEO HUD STREAM SIMULATION
// ═══════════════════════════════════════════════════

const videoCanvas = document.getElementById("video-hud-canvas");
const videoCtx = videoCanvas?.getContext("2d");

// Create background optical frame image
const cameraImage = new Image();
cameraImage.src = "rover_camera_feed.jpg";

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

function drawTacticalHUD() {
    if (!videoCanvas || !videoCtx) return;

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

    // Check if GCS is connected and telemetry is flowing
    const isConnected = telemetryStateLbl.textContent === "CONNECTED";

    if (isConnected && cameraImage.complete) {
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
        // Simulate slight lock-on shaking
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
        videoCtx.fillText("[TRG_A: WP_03]", boxX, boxY - 5);

        // Draw Target Bounding Box 2 (Obstacle Avoidance)
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

        // Draw Dynamic HUD artificial horizon ladder (moves with heading/simulated pitch)
        const heading = parseFloat(document.getElementById("heading-dial-val")?.textContent || "0");
        const roll = Math.sin(time / 2000) * 8; // simulated roll oscillation
        const pitch = Math.cos(time / 2500) * 10; // simulated pitch oscillation
        
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
    }

    requestAnimationFrame(drawTacticalHUD);
}

// Start rendering loop when image loads
cameraImage.onload = () => {
    drawTacticalHUD();
};
// Fallback if image caching is immediate
if (cameraImage.complete) {
    drawTacticalHUD();
}
