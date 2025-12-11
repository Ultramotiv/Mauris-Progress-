# =====================================================================
# FAIRINO ROBOT - ULTRA SMOOTH CONTROL WITH WINDOW CLEANING GAME
# → 1 Lap = +90° → -90° → back to +90° (round trip)
# → Switches IMMEDIATELY after ONE full lap
# → MAX JOINT VELOCITY: 60°/sec (buttery smooth)
# → Web-based game shows robot cleaning a dusty window!
# =====================================================================
# CHANGE ONLY THESE 3 LINES
ARC1_RADIUS = 300
ARC2_RADIUS = 200
ARC3_RADIUS = 100
# =====================================================================

import sys
import time
import math
import numpy as np
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import webbrowser
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
robot = Robot.RPC('192.168.58.2')

X_MOVEMENT = 300.0
ARC_POINTS = 1200
SERVO_RATE = 0.008

# ULTRA-SMOOTH TUNING - MAXIMUM STABILITY
FORCE_SCALE = 0.05
FILTER_ALPHA = 0.12
DEADZONE = [4.0, 4.0, 4.5, 2.5, 2.5, 2.5]
FORCE_THRESHOLD = 1.5
MAX_STEP = 1.2
MAX_JOINT_VEL_DEG = 60.0

VELOCITY_SMOOTHING = 0.82
POSITION_SMOOTHING = 0.28
JOINT_TARGET_SMOOTHING = 0.25

center_x = center_y = 0.0
baseline = [0.0] * 6
filtered = [0.0] * 6

arcs = {
    ARC1_RADIUS: {"p": [], "j": [], "name": f"Arc1 {ARC1_RADIUS}mm"},
    ARC2_RADIUS: {"p": [], "j": [], "name": f"Arc2 {ARC2_RADIUS}mm"},
    ARC3_RADIUS: {"p": [], "j": [], "name": f"Arc3 {ARC3_RADIUS}mm"}
}

sequence = [ARC1_RADIUS, ARC2_RADIUS, ARC3_RADIUS]
current_index = 0
current_radius = sequence[current_index]

current_points = current_joints = []
pos = 0.0
smoothed_pos = 0.0
vel = 0.0
prev_joints = None
smoothed_target = None

lap_counter = {ARC1_RADIUS: 0, ARC2_RADIUS: 0, ARC3_RADIUS: 0}
has_passed_bottom = False
lap_detected_this_cycle = False
transition_lock = False

# Game state variables
game_state = {
    "robot_x": 0,
    "robot_y": 0,
    "current_arc": ARC1_RADIUS,
    "arc1_laps": 0,
    "arc2_laps": 0,
    "arc3_laps": 0,
    "velocity": 0,
    "force": 0
}

# HTML for the enhanced rainy window cleaning game
HTML_GAME = """<!DOCTYPE html>
<html>
<head>
    <title>Robot Window Cleaner - Rainy Day</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(to bottom, #2c3e50 0%, #34495e 100%);
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            min-height: 100vh;
            padding: 20px;
            overflow: hidden;
        }
        h1 {
            color: #ecf0f1;
            text-align: center;
            margin-bottom: 20px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.5);
            font-size: 2em;
            letter-spacing: 2px;
        }
        #gameContainer {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 20px;
            padding: 25px;
            box-shadow: 0 15px 50px rgba(0,0,0,0.5);
            backdrop-filter: blur(10px);
            border: 2px solid rgba(255, 255, 255, 0.2);
        }
        #canvasWrapper {
            position: relative;
            display: inline-block;
            border-radius: 15px;
            overflow: hidden;
            box-shadow: inset 0 0 50px rgba(0,0,0,0.3);
        }
        canvas {
            display: block;
            border: 5px solid #34495e;
            border-radius: 15px;
        }
        #rainCanvas {
            position: absolute;
            top: 0;
            left: 0;
            pointer-events: none;
            z-index: 10;
        }
        #stats {
            margin-top: 25px;
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 20px;
        }
        .stat {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 20px;
            border-radius: 15px;
            text-align: center;
            color: white;
            box-shadow: 0 5px 15px rgba(0,0,0,0.3);
            transition: transform 0.3s ease, box-shadow 0.3s ease;
        }
        .stat:hover {
            transform: translateY(-5px);
            box-shadow: 0 8px 20px rgba(0,0,0,0.4);
        }
        .stat-label {
            font-size: 13px;
            opacity: 0.95;
            margin-bottom: 8px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .stat-value {
            font-size: 28px;
            font-weight: bold;
            text-shadow: 1px 1px 2px rgba(0,0,0,0.3);
        }
        #info {
            margin-top: 20px;
            text-align: center;
            color: #ecf0f1;
            font-size: 15px;
            padding: 15px;
            background: rgba(0,0,0,0.3);
            border-radius: 10px;
            border: 1px solid rgba(255,255,255,0.1);
        }
        .pulse {
            animation: pulse 2s infinite;
        }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.7; }
        }
    </style>
</head>
<body>
    <h1>🤖 Robotic Window Cleaner - Rainy Day Mode 🌧️</h1>
    <div id="gameContainer">
        <div id="canvasWrapper">
            <canvas id="gameCanvas" width="900" height="700"></canvas>
            <canvas id="rainCanvas" width="900" height="700"></canvas>
        </div>
        <div id="stats">
            <div class="stat">
                <div class="stat-label">🔴 Arc 1 (300mm)</div>
                <div class="stat-value" id="arc1">0 laps</div>
            </div>
            <div class="stat">
                <div class="stat-label">🟢 Arc 2 (200mm)</div>
                <div class="stat-value" id="arc2">0 laps</div>
            </div>
            <div class="stat">
                <div class="stat-label">🔵 Arc 3 (100mm)</div>
                <div class="stat-value" id="arc3">0 laps</div>
            </div>
        </div>
        <div id="info" class="pulse">💧 Push the robot to clean the rainy window! Each lap removes water & dirt from that arc. 💧</div>
    </div>

    <script>
        const canvas = document.getElementById('gameCanvas');
        const ctx = canvas.getContext('2d');
        const rainCanvas = document.getElementById('rainCanvas');
        const rainCtx = rainCanvas.getContext('2d');
        
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const scale = 1.0;
        
        // Raindrops array
        let raindrops = [];
        let splashes = [];
        
        // Cleaned arcs tracking
        let cleanedArcs = {
            300: 0,
            200: 0,
            100: 0
        };
        
        // Water droplets on window
        let waterDrops = [];
        
        // Initialize raindrops
        function initRain() {
            for (let i = 0; i < 150; i++) {
                raindrops.push({
                    x: Math.random() * canvas.width,
                    y: Math.random() * canvas.height,
                    length: Math.random() * 20 + 10,
                    speed: Math.random() * 3 + 4,
                    opacity: Math.random() * 0.5 + 0.3
                });
            }
        }
        
        // Initialize water drops on window
        function initWaterDrops() {
            for (let i = 0; i < 80; i++) {
                waterDrops.push({
                    x: Math.random() * canvas.width,
                    y: Math.random() * canvas.height,
                    radius: Math.random() * 8 + 3,
                    opacity: Math.random() * 0.4 + 0.2,
                    speed: Math.random() * 0.5 + 0.2
                });
            }
        }
        
        // Animate rain
        function animateRain() {
            rainCtx.clearRect(0, 0, rainCanvas.width, rainCanvas.height);
            
            // Draw rain
            raindrops.forEach(drop => {
                rainCtx.strokeStyle = `rgba(174, 194, 224, ${drop.opacity})`;
                rainCtx.lineWidth = 1;
                rainCtx.beginPath();
                rainCtx.moveTo(drop.x, drop.y);
                rainCtx.lineTo(drop.x, drop.y + drop.length);
                rainCtx.stroke();
                
                drop.y += drop.speed;
                
                if (drop.y > canvas.height) {
                    drop.y = -drop.length;
                    drop.x = Math.random() * canvas.width;
                    
                    // Create splash
                    splashes.push({
                        x: drop.x,
                        y: canvas.height - 5,
                        radius: 0,
                        maxRadius: 15,
                        opacity: 0.6
                    });
                }
            });
            
            // Draw splashes
            splashes = splashes.filter(splash => {
                splash.radius += 1.5;
                splash.opacity -= 0.03;
                
                if (splash.opacity > 0) {
                    rainCtx.strokeStyle = `rgba(174, 194, 224, ${splash.opacity})`;
                    rainCtx.lineWidth = 2;
                    rainCtx.beginPath();
                    rainCtx.arc(splash.x, splash.y, splash.radius, 0, Math.PI, true);
                    rainCtx.stroke();
                    return true;
                }
                return false;
            });
            
            requestAnimationFrame(animateRain);
        }
        
        // Draw window with dirt and water
        function drawWindow() {
            // Background - dark rainy sky outside
            const gradient = ctx.createLinearGradient(0, 0, 0, canvas.height);
            gradient.addColorStop(0, '#34495e');
            gradient.addColorStop(1, '#2c3e50');
            ctx.fillStyle = gradient;
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            
            // Draw window frame
            ctx.strokeStyle = '#1a1a1a';
            ctx.lineWidth = 20;
            ctx.strokeRect(10, 10, canvas.width - 20, canvas.height - 20);
            
            // Inner frame detail
            ctx.strokeStyle = '#2c2c2c';
            ctx.lineWidth = 8;
            ctx.strokeRect(25, 25, canvas.width - 50, canvas.height - 50);
            
            // Draw water droplets that haven't been cleaned
            waterDrops.forEach(drop => {
                const inCleanedZone = isInCleanedZone(drop.x, drop.y);
                if (!inCleanedZone) {
                    // Water drop
                    ctx.fillStyle = `rgba(100, 149, 237, ${drop.opacity})`;
                    ctx.beginPath();
                    ctx.arc(drop.x, drop.y, drop.radius, 0, Math.PI * 2);
                    ctx.fill();
                    
                    // Highlight
                    ctx.fillStyle = `rgba(255, 255, 255, ${drop.opacity * 0.6})`;
                    ctx.beginPath();
                    ctx.arc(drop.x - drop.radius * 0.3, drop.y - drop.radius * 0.3, drop.radius * 0.3, 0, Math.PI * 2);
                    ctx.fill();
                    
                    drop.y += drop.speed;
                    if (drop.y > canvas.height) {
                        drop.y = 0;
                        drop.x = Math.random() * canvas.width;
                    }
                }
            });
            
            // Draw center point
            ctx.fillStyle = '#555';
            ctx.beginPath();
            ctx.arc(centerX, centerY, 10, 0, Math.PI * 2);
            ctx.fill();
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 2;
            ctx.stroke();
            
            // Draw arcs
            drawArc(300, cleanedArcs[300], '#e74c3c');
            drawArc(200, cleanedArcs[200], '#2ecc71');
            drawArc(100, cleanedArcs[100], '#3498db');
        }
        
        function isInCleanedZone(x, y) {
            const dx = x - centerX;
            const dy = y - centerY;
            const dist = Math.sqrt(dx * dx + dy * dy);
            const angle = Math.atan2(dy, dx);
            const angleDeg = angle * 180 / Math.PI;
            
            // Check if in arc range (90° to -90°, left semicircle)
            if (angleDeg < -90 || angleDeg > 90) return false;
            
            // Check each arc
            for (let radius of [300, 200, 100]) {
                const r = radius * scale;
                if (Math.abs(dist - r) < 20 && cleanedArcs[radius] >= 2) {
                    return true;
                }
            }
            return false;
        }
        
        function drawArc(radius, laps, color) {
            const r = radius * scale;
            const cleanness = Math.min(laps / 2.0, 1.0);        // 2 laps = fully crystal clean
            const dirtOpacity = 0.75 * (1 - cleanness);       // dirt fades fast

            // 1. Dirty/wet base layer (disappears as laps increase)
            if (dirtOpacity > 0.05) {
                ctx.globalAlpha = dirtOpacity;
                ctx.strokeStyle = 'rgba(80, 60, 40, 0.7)';
                ctx.lineWidth = 38;
                ctx.beginPath();
                ctx.arc(centerX, centerY, r, -Math.PI * 0.5, Math.PI * 0.5);
                ctx.stroke();

                ctx.strokeStyle = 'rgba(100, 149, 237, 0.5)';
                ctx.lineWidth = 32;
                ctx.beginPath();
                ctx.arc(centerX, centerY, r, -Math.PI * 0.5, Math.PI * 0.5);
                ctx.stroke();
                ctx.globalAlpha = 1.0;
            }

            // 2. Clean glass shine (appears as you clean)
            if (cleanness > 0) {
                ctx.globalAlpha = cleanness;
                ctx.strokeStyle = 'rgba(220, 240, 255, 0.9)';
                ctx.lineWidth = 40;
                ctx.shadowColor = '#ffffff';
                ctx.shadowBlur = 20;
                ctx.beginPath();
                ctx.arc(centerX, centerY, r, -Math.PI * 0.5, Math.PI * 0.5);
                ctx.stroke();
                ctx.shadowBlur = 0;
                ctx.globalAlpha = 1.0;

                // Sparkling highlight when ≥2 laps
                if (laps >= 2) {
                    ctx.strokeStyle = 'rgba(255, 255, 255, 0.7)';
                    ctx.lineWidth = 4;
                    ctx.setLineDash([15, 15]);
                    ctx.beginPath();
                    ctx.arc(centerX, centerY, r + 8, -Math.PI * 0.5, Math.PI * 0.5);
                    ctx.stroke();
                    ctx.setLineDash([]);
                }
            }

            // 3. Colored outline (always visible, brighter when clean)
            ctx.strokeStyle = laps > 0 ? color : '#555';
            ctx.lineWidth = 5;
            ctx.shadowColor = laps > 0 ? color : 'transparent';
            ctx.shadowBlur = laps > 0 ? 15 : 0;
            ctx.beginPath();
            ctx.arc(centerX, centerY, r, -Math.PI * 0.5, Math.PI * 0.5);
            ctx.stroke();
            ctx.shadowBlur = 0;

            // Extra sparkles when fully clean
            if (laps >= 2) {
                for (let i = 0; i < 12; i++) {
                    const angle = -Math.PI * 0.5 + (Math.PI * i / 11);
                    const x = centerX + r * Math.cos(angle);
                    const y = centerY + r * Math.sin(angle);
                    drawSparkle(x, y, cleanness);
                }
            }
        }
        
        function drawSparkle(x, y, intensity) {
            ctx.save();
            ctx.globalAlpha = intensity;
            
            // Star shape
            ctx.fillStyle = '#FFD700';
            ctx.shadowColor = 'white';
            ctx.shadowBlur = 8;
            ctx.beginPath();
            for (let i = 0; i < 4; i++) {
                const angle = (Math.PI / 2) * i;
                const x1 = x + Math.cos(angle) * 8;
                const y1 = y + Math.sin(angle) * 8;
                const x2 = x + Math.cos(angle + Math.PI / 4) * 3;
                const y2 = y + Math.sin(angle + Math.PI / 4) * 3;
                if (i === 0) ctx.moveTo(x1, y1);
                else ctx.lineTo(x1, y1);
                ctx.lineTo(x2, y2);
            }
            ctx.closePath();
            ctx.fill();
            
            ctx.restore();
        }
        
        function drawRobot(x, y, currentArc) {
            const screenX = centerX + x * scale;
            const screenY = centerY - y * scale;
            
            // Robot arm shadow
            ctx.fillStyle = 'rgba(0, 0, 0, 0.3)';
            ctx.beginPath();
            ctx.arc(screenX + 5, screenY + 5, 28, 0, Math.PI * 2);
            ctx.fill();
            
            // Robot arm body
            const gradient = ctx.createRadialGradient(screenX, screenY, 5, screenX, screenY, 25);
            gradient.addColorStop(0, '#e0e0e0');
            gradient.addColorStop(1, '#9e9e9e');
            ctx.fillStyle = gradient;
            ctx.beginPath();
            ctx.arc(screenX, screenY, 25, 0, Math.PI * 2);
            ctx.fill();
            
            // Arm details
            ctx.strokeStyle = '#616161';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.arc(screenX, screenY, 25, 0, Math.PI * 2);
            ctx.stroke();
            
            // Cleaning sponge
            const spongeGradient = ctx.createRadialGradient(screenX, screenY, 0, screenX, screenY, 15);
            spongeGradient.addColorStop(0, '#FFD54F');
            spongeGradient.addColorStop(1, '#FFA726');
            ctx.fillStyle = spongeGradient;
            ctx.beginPath();
            ctx.arc(screenX, screenY, 15, 0, Math.PI * 2);
            ctx.fill();
            
            // Sponge texture
            ctx.fillStyle = '#FF8A65';
            for (let i = 0; i < 6; i++) {
                const angle = (Math.PI * 2 * i) / 6;
                const dx = Math.cos(angle) * 8;
                const dy = Math.sin(angle) * 8;
                ctx.beginPath();
                ctx.arc(screenX + dx, screenY + dy, 2, 0, Math.PI * 2);
                ctx.fill();
            }
            
            // Soap bubbles
            for (let i = 0; i < 5; i++) {
                const angle = Math.random() * Math.PI * 2;
                const distance = Math.random() * 40 + 30;
                const bx = screenX + Math.cos(angle) * distance;
                const by = screenY + Math.sin(angle) * distance;
                const size = Math.random() * 8 + 4;
                
                const bubbleGradient = ctx.createRadialGradient(bx, by, 0, bx, by, size);
                bubbleGradient.addColorStop(0, 'rgba(255, 255, 255, 0.8)');
                bubbleGradient.addColorStop(0.5, 'rgba(173, 216, 230, 0.6)');
                bubbleGradient.addColorStop(1, 'rgba(135, 206, 250, 0.3)');
                ctx.fillStyle = bubbleGradient;
                ctx.beginPath();
                ctx.arc(bx, by, size, 0, Math.PI * 2);
                ctx.fill();
                
                // Bubble highlight
                ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
                ctx.beginPath();
                ctx.arc(bx - size * 0.3, by - size * 0.3, size * 0.3, 0, Math.PI * 2);
                ctx.fill();
            }
            
            // Arc indicator light
            const colors = {300: '#e74c3c', 200: '#2ecc71', 100: '#3498db'};
            ctx.fillStyle = colors[currentArc] || '#666';
            ctx.shadowColor = colors[currentArc] || '#666';
            ctx.shadowBlur = 15;
            ctx.beginPath();
            ctx.arc(screenX, screenY - 40, 10, 0, Math.PI * 2);
            ctx.fill();
            ctx.shadowBlur = 0;
        }
        
        function updateGame() {
            fetch('/state')
                .then(response => response.json())
                .then(data => {
                    cleanedArcs[300] = data.arc1_laps;
                    cleanedArcs[200] = data.arc2_laps;
                    cleanedArcs[100] = data.arc3_laps;
                    
                    document.getElementById('arc1').textContent = data.arc1_laps + ' laps';
                    document.getElementById('arc2').textContent = data.arc2_laps + ' laps';
                    document.getElementById('arc3').textContent = data.arc3_laps + ' laps';
                    
                    drawWindow();
                    drawRobot(data.robot_x, data.robot_y, data.current_arc);
                })
                .catch(err => console.error('Error fetching state:', err));
        }
        
        // Initialize
        initRain();
        initWaterDrops();
        drawWindow();
        drawRobot(0, 300, 300);
        
        // Start animations
        animateRain();
        setInterval(updateGame, 100);
    </script>
</body>
</html>"""

class GameHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Suppress logging
    
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_GAME.encode())
        elif self.path == '/state':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(game_state).encode())

def start_web_server():
    server = HTTPServer(('0.0.0.0', 8080), GameHandler)
    print("\n🎮 Game Server Started!")
    print("Open browser to: http://localhost:8080")
    print("Or from another device: http://<robot-ip>:8080\n")
    server.serve_forever()

def get_forces():
    global filtered
    tcp = robot.GetActualTCPPose(flag=1)[1]
    x, y = tcp[0], tcp[1]
    rx, ry, rz = map(math.radians, tcp[3:6])
    ret = robot.FT_GetForceTorqueRCS()
    if ret[0] != 0: return 0.0
    raw = [ret[1][i] - baseline[i] for i in range(6)]
    
    for i in range(6):
        v = raw[i]
        av = abs(v)
        if av < DEADZONE[i % 6]:
            v = v * (av / DEADZONE[i % 6]) ** 3
        else:
            v = v - DEADZONE[i % 6] if v > 0 else v + DEADZONE[i % 6]
        filtered[i] = FILTER_ALPHA * v + (1 - FILTER_ALPHA) * filtered[i]
    
    c, s = math.cos, math.sin
    R = np.array([
        [c(rz)*c(ry), c(rz)*s(ry)*s(rx) - s(rz)*c(rx), c(rz)*s(ry)*c(rx) + s(rz)*s(rx)],
        [s(rz)*c(ry), s(rz)*s(ry)*s(rx) + c(rz)*c(rx), s(rz)*s(ry)*c(rx) - c(rz)*s(rx)],
        [-s(ry),      c(ry)*s(rx),                     c(ry)*c(rx)]
    ])
    f_world = R @ np.array(filtered[:3])
    fx, fy = f_world[0], f_world[1]
    dx = x - center_x
    dy = y - center_y
    mag = math.hypot(dx, dy)
    if mag < 10: return 0.0
    tx = +dy / mag
    ty = -dx / mag
    return fx * tx + fy * ty

def ema(n, o, alpha=0.45): 
    return alpha * n + (1 - alpha) * o

def interp(a, b, t): 
    return [aa + t*(bb-aa) for aa, bb in zip(a, b)]

def compute_velocity_clamped_position(current_j, target_j, dt):
    new_j = []
    for i in range(6):
        velocity = (target_j[i] - current_j[i]) / dt
        clamped_velocity = np.clip(velocity, -MAX_JOINT_VEL_DEG, MAX_JOINT_VEL_DEG)
        new_j.append(current_j[i] + clamped_velocity * dt)
    return new_j

def init_ft():
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT sensor initialized")

def calibrate():
    global baseline
    print("Calibrating baseline (keep still for 3 seconds)...")
    forces = []
    for _ in range(300):
        r = robot.FT_GetForceTorqueRCS()
        if r[0] == 0: forces.append(r[1][:6])
        time.sleep(0.01)
    if forces:
        baseline = np.mean(forces, axis=0).tolist()
    print(f"Baseline calibrated")

print("\n" + "="*80)
print("FAIRINO - ULTRA SMOOTH CONTROL WITH RAINY WINDOW CLEANING GAME")
print("→ Max joint velocity: 60°/sec")
print("→ Realistic rainy window with water droplets!")
print("="*80)

# Start web server in background
server_thread = threading.Thread(target=start_web_server, daemon=True)
server_thread.start()
time.sleep(1)

# Auto-open browser
print("🌐 Opening browser...")
try:
    webbrowser.open('http://localhost:8080')
    print("✅ Browser opened! If not, manually go to: http://localhost:8080")
except:
    print("⚠️  Could not auto-open browser. Please manually open: http://localhost:8080")
time.sleep(2)

p0 = robot.GetActualTCPPose(flag=1)[1]
robot.MoveL(desc_pos=[p0[0] - X_MOVEMENT, p0[1], p0[2], p0[3], p0[4], p0[5]], tool=0, user=0, vel=5, acc=0)
time.sleep(2.0)
center = robot.GetActualTCPPose(flag=1)[1]
center_x, center_y = center[0], center[1]

robot.MoveL(desc_pos=[center_x, center_y + ARC1_RADIUS, p0[2], p0[3], p0[4], p0[5]], tool=0, user=0, vel=5, acc=0)
time.sleep(1.5)

for radius in [ARC1_RADIUS, ARC2_RADIUS, ARC3_RADIUS]:
    print(f"Generating {arcs[radius]['name']}...")
    arcs[radius]["p"] = []
    arcs[radius]["j"] = []
    for i in range(ARC_POINTS + 1):
        angle_deg = 90 - (i / ARC_POINTS) * 180
        theta = math.radians(angle_deg)
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        p = [x, y, p0[2], p0[3], p0[4], p0[5]]
        ik = robot.GetInverseKin(0, p, -1)
        j = ik[1][:6] if ik[0] == 0 else arcs[radius]["j"][-1] if arcs[radius]["j"] else [0]*6
        arcs[radius]["p"].append(p)
        arcs[radius]["j"].append(j)

current_radius = ARC1_RADIUS
current_points = arcs[current_radius]["p"]
current_joints = arcs[current_radius]["j"]
pos = 0.0
smoothed_pos = 0.0

init_joints = robot.GetActualJointPosDegree(flag=1)[1][:6]
prev_joints = init_joints
smoothed_target = init_joints[:]

init_ft()
calibrate()

print("\n" + "="*80)
print("AUTOMATIC MODE RUNNING - CLEAN THE RAINY WINDOW!")
print("="*80 + "\n")

last_print = time.time()

try:
    while True:
        if transition_lock:
            time.sleep(0.01)
            continue

        tangential_force = get_forces()
        
        # Update game state
        tcp = robot.GetActualTCPPose(flag=1)[1]
        game_state["robot_x"] = tcp[0] - center_x
        game_state["robot_y"] = tcp[1] - center_y
        game_state["current_arc"] = current_radius
        game_state["velocity"] = vel
        game_state["force"] = tangential_force

        raw_v = tangential_force * FORCE_SCALE
        if abs(tangential_force) > FORCE_THRESHOLD:
            raw_v = (tangential_force / 6.0) * MAX_STEP
        raw_v = max(-MAX_STEP, min(MAX_STEP, raw_v))
        
        vel = ema(raw_v, vel, VELOCITY_SMOOTHING)
        pos += vel
        pos = max(0.0, min(len(current_joints) - 1.0, pos))
        smoothed_pos = ema(pos, smoothed_pos, POSITION_SMOOTHING)

        i = int(smoothed_pos)
        t = smoothed_pos - i
        ni = min(i + 1, len(current_joints) - 1)
        desired_j = interp(current_joints[i], current_joints[ni], t)
        
        target_j = compute_velocity_clamped_position(prev_joints, desired_j, SERVO_RATE)
        
        for k in range(6):
            smoothed_target[k] = ema(target_j[k], smoothed_target[k], JOINT_TARGET_SMOOTHING)
        
        robot.ServoJ(smoothed_target, [0]*6, 0, 0, SERVO_RATE, 0, 0)
        prev_joints = smoothed_target[:]

        total = len(current_joints) - 1
        at_top = pos <= 80 or pos >= total - 80
        at_bottom = 0.35 < (pos / total) < 0.65

        if at_bottom and vel > 0.15 and not has_passed_bottom:
            has_passed_bottom = True
            print("   [Passed bottom - halfway done]")

        if at_top and has_passed_bottom and pos <= 80 and not lap_detected_this_cycle:
            lap_counter[current_radius] += 1
            lap_detected_this_cycle = True
            
            # Update game state with lap completion
            if current_radius == ARC1_RADIUS:
                game_state["arc1_laps"] = lap_counter[current_radius]
            elif current_radius == ARC2_RADIUS:
                game_state["arc2_laps"] = lap_counter[current_radius]
            elif current_radius == ARC3_RADIUS:
                game_state["arc3_laps"] = lap_counter[current_radius]
            
            print(f"\n>>> FULL LAP {lap_counter[current_radius]} COMPLETED on {arcs[current_radius]['name']} <<<")
            print(f"    💧 Window section cleaned! Watch the rain wash away!")

            current_index = (current_index + 1) % 3
            next_radius = sequence[current_index]
            print(f">>> SWITCHING TO {arcs[next_radius]['name']} <<<\n")

            transition_lock = True
            robot.MoveL(desc_pos=arcs[next_radius]["p"][0], tool=0, user=0, vel=5, acc=0)
            time.sleep(1.2)

            current_radius = next_radius
            current_points = arcs[current_radius]["p"]
            current_joints = arcs[current_radius]["j"]
            pos = 0.0
            smoothed_pos = 0.0
            vel = 0.0
            
            prev_joints = robot.GetActualJointPosDegree(flag=1)[1][:6]
            smoothed_target = prev_joints[:]
            
            has_passed_bottom = False
            lap_detected_this_cycle = False
            transition_lock = False
            continue

        if pos > 150:
            lap_detected_this_cycle = False

        if time.time() - last_print > 0.6:
            prog = pos / total * 100
            status = "FWD" if vel > 0.3 else "BACK" if vel < -0.3 else "STOP"
            top = " [TOP]" if at_top else ""
            bottom = " [BOTTOM]" if at_bottom else ""

            print(f"{status:4} {arcs[current_radius]['name']:>16} | Lap {lap_counter[current_radius]:>2} | "
                  f"{prog:5.1f}%{top}{bottom} | 💧 Cleaning rainy window...")
            last_print = time.time()

        time.sleep(SERVO_RATE)

except KeyboardInterrupt:
    print("\n\n" + "="*80)
    print("STOPPED - RAINY WINDOW CLEANING COMPLETE!")
    for r in sequence:
        print(f"   {arcs[r]['name']:>16} → {lap_counter[r]} lap(s)")
    print("="*80)