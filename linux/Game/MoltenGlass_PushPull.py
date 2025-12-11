import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math
import threading
import socket
import json
import webbrowser

# ============================= CONNECT TO ROBOT =============================
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")
error, version = robot.GetSDKVersion()
print(f"SDK version: {version}")

# ======================= COORDINATE TRANSFORMATION =======================
def euler_to_rotation_matrix(rx, ry, rz):
    rx_rad = math.radians(rx)
    ry_rad = math.radians(ry)
    rz_rad = math.radians(rz)
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(rx_rad), -math.sin(rx_rad)],
        [0, math.sin(rx_rad), math.cos(rx_rad)]
    ])
    Ry = np.array([
        [math.cos(ry_rad), 0, math.sin(ry_rad)],
        [0, 1, 0],
        [-math.sin(ry_rad), 0, math.cos(ry_rad)]
    ])
    Rz = np.array([
        [math.cos(rz_rad), -math.sin(rz_rad), 0],
        [math.sin(rz_rad), math.cos(rz_rad), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx

def transform_forces_to_tcp_and_world(ft_forces, tcp_orientation):
    rx, ry, rz = tcp_orientation
    R = euler_to_rotation_matrix(rx, ry, rz)
    # FT sensor → TCP mapping (corrected as per your code)
    tcp_force_vector = np.array([
        +ft_forces[0],  # FT +X → TCP +X
        +ft_forces[1],  # FT -Y → TCP +Y
        -ft_forces[2]   # FT -Z → TCP -Z
    ])
    tcp_moment_vector = np.array([
        +ft_forces[3],
        -ft_forces[4],
        -ft_forces[5]
    ])
    world_force_vector = R @ tcp_force_vector
    world_moment_vector = R @ tcp_moment_vector
    return world_force_vector.tolist()[:3]  # Return only forces

# ============================= FT SENSOR =============================
def init_ft_sensor():
    company = 24
    device = 0
    robot.FT_SetConfig(company, device)
    robot.FT_Activate(0); time.sleep(0.5)
    robot.FT_Activate(1); time.sleep(0.5)
    robot.SetLoadWeight(0, 0.0); robot.SetLoadCoord(0.0, 0.0, 0.0)
    robot.FT_SetZero(0); time.sleep(0.5)
    robot.FT_SetZero(1); time.sleep(0.5)
    print("FT Sensor initialized and zeroed.")

def calibrate_baseline_forces(samples=300):
    print("Calibrating gravity compensation (300 samples)...")
    forces = []
    for _ in range(samples):
        data = robot.FT_GetForceTorqueRCS()
        if data[0] == 0:
            forces.append(data[1][:3])
        time.sleep(0.005)
    if forces:
        baseline = np.mean(forces, axis=0)
        print(f"Baseline [Fx,Fy,Fz]: {baseline[0]:.3f}, {baseline[1]:.3f}, {baseline[2]:.3f} N")
        return baseline.tolist()
    return [0.0, 0.0, 0.0]

# ============================= SHUTDOWN =============================
shutdown_requested = False
def shutdown(sig, frame):
    global shutdown_requested
    print("\nShutting down...")
    shutdown_requested = True
signal.signal(signal.SIGINT, shutdown)

# ============================= KEY STATE =============================
space_pressed = False
enter_pressed = False
key_lock = threading.Lock()

# ============================= FORCE FILTER =============================
class ForceFilter:
    def __init__(self, alpha=0.7):
        self.alpha = alpha
        self.value = 0.0
    def update(self, new_val):
        self.value = self.alpha * self.value + (1 - self.alpha) * new_val
        return self.value

force_filter = ForceFilter(alpha=0.7)

# ============================= WEB SERVER =============================
class WebServer:
    def __init__(self, port=8080):
        self.port = port
        self.server_socket = None
        self.current_fy = 150.0
        self.running = False
        self.client_connected = False
        self.last_client_time = time.time()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run_server, daemon=True)
        self.thread.start()
        print(f"\nWeb server: http://localhost:{self.port}")
        time.sleep(1)
        webbrowser.open(f'http://localhost:{self.port}')

    def _run_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('', self.port))
        self.server_socket.listen(5)
        print("Web server listening...")

        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                client, _ = self.server_socket.accept()
                self.client_connected = True
                self.last_client_time = time.time()
                threading.Thread(target=self._handle_client, args=(client,), daemon=True).start()
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Server error: {e}")

    def _handle_client(self, sock):
        global space_pressed, enter_pressed
        try:
            sock.settimeout(3.0)
            request = sock.recv(4096).decode('utf-8', errors='ignore')
            if not request:
                return
            first_line = request.split('\n')[0]
            path = first_line.split(' ')[1] if len(first_line.split()) > 1 else '/'
            query = {}
            if '?' in path:
                path, q = path.split('?', 1)
                for pair in q.split('&'):
                    if '=' in pair:
                        k, v = pair.split('=', 1)
                        query[k] = v

            if path in ['/', '/index.html']:
                self._send_html(sock)
            elif path == '/api/fy':
                data = json.dumps({'fy': round(self.current_fy, 2)})
                headers = (
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: application/json\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "Cache-Control: no-cache\r\n"
                    f"Content-Length: {len(data)}\r\n\r\n{data}"
                )
                sock.sendall(headers.encode())
            elif path == '/api/key':
                space = query.get('space') == '1'
                enter = query.get('enter') == '1'
                with key_lock:
                    space_pressed = space
                    enter_pressed = enter
                sock.sendall(b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n")
            elif path == '/api/heartbeat':
                self.last_client_time = time.time()
                sock.sendall(b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n")
            else:
                sock.sendall(b"HTTP/1.1 404\r\n\r\n")
        except:
            pass
        finally:
            sock.close()

    def is_client_alive(self):
        return self.client_connected and (time.time() - self.last_client_time < 10)

    def _send_html(self, sock):
        html = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>GLASS FORGE - ULTRA SMOOTH</title>
    <style>
        *{margin:0;padding:0;box-sizing:border-box;}
        body{font-family:Arial;background:linear-gradient(180deg,#1a0a0a,#4a1010,#1a0a0a);color:#fff;overflow:hidden;height:100vh;display:flex;flex-direction:column;align-items:center;justify-content:center;}
        #header{position:absolute;top:20px;width:95%;max-width:1200px;display:flex;justify-content:space-between;z-index:10;}
        #title{font-size:48px;font-weight:bold;color:#ff6600;text-shadow:0 0 20px rgba(255,100,0,0.8);}
        #stats{display:flex;gap:30px;}
        .stat-label{font-size:14px;color:#ffaa66;}
        .stat-value{font-size:32px;font-weight:bold;}
        #score{color:#ff6600;}#match{color:#00ffff;}#strikes{color:#ffff00;}
        #gameCanvas{border:3px solid #ff6600;border-radius:10px;box-shadow:0 0 40px rgba(255,100,0,0.5);background:#000;}
        #controls{position:absolute;bottom:20px;display:flex;gap:20px;align-items:center;}
        button{padding:12px 24px;font-size:16px;font-weight:bold;border:none;border-radius:8px;background:#ff6600;color:white;cursor:pointer;transition:0.3s;}
        button:hover{background:#ff8800;transform:scale(1.05);}
        #robotPosition{background:rgba(0,0,0,0.7);padding:15px 25px;border-radius:10px;border:2px solid #ff6600;}
        #fyValue{font-size:28px;color:#00ffff;font-weight:bold;}
        #positionBar{width:300px;height:12px;background:#333;border-radius:6px;overflow:hidden;margin-top:8px;}
        #positionFill{height:100%;background:linear-gradient(90deg,#ff6600,#ffaa00);width:50%;transition:width 0.05s;}
        #forceIndicator{font-size:12px;margin-top:5px;font-weight:bold;}
        .status-ready{color:#ffaa00;}.status-playing{color:#00ff00;animation:pulse 1s infinite;}
        @keyframes pulse{0%,100%{opacity:1}50%{opacity:0.5}}
        #feedback{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);font-size:72px;font-weight:bold;text-shadow:0 0 30px currentColor;pointer-events:none;opacity:0;transition:opacity 0.3s;}
        .feedback-perfect{color:#00ff00;}.feedback-good{color:#ffff00;}.feedback-miss{color:#ff4444;}.feedback-material{color:#00ffff;}
        #overlay{position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.9);display:flex;align-items:center;justify-content:center;z-index:100;}
        .overlay-content{text-align:center;padding:40px;background:rgba(26,10,10,0.95);border-radius:20px;border:3px solid #ff6600;box-shadow:0 0 50px rgba(255,100,0,0.6);}
        .overlay-title{font-size:64px;margin-bottom:20px;text-shadow:0 0 30px currentColor;}
        #resultOverlay{position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.9);display:none;align-items:center;justify-content:center;z-index:101;}
        .result-content{text-align:center;padding:40px;background:rgba(26,10,10,0.95);border-radius:20px;border:3px solid #ff6600;box-shadow:0 0 50px rgba(255,100,0,0.6);}
        .passed{color:#00ff00;}.failed{color:#ff4444;}
        #keyHint{position:absolute;top:80px;left:50%;transform:translateX(-50%);font-size:24px;color:#ffaa66;background:rgba(0,0,0,0.6);padding:10px 20px;border-radius:10px;}
    </style>
</head>
<body>
    <div id="header">
        <div><div id="title">GLASS FORGE</div><div style="font-size:14px;color:#ffaa66;margin-top:5px;">SPACEBAR = Hammer | ENTER = Add Material</div></div>
        <div id="stats">
            <div class="stat"><div class="stat-label">Score</div><div class="stat-value" id="score">0</div></div>
            <div class="stat"><div class="stat-label">Match</div><div class="stat-value" id="match">0%</div></div>
            <div class="stat"><div class="stat-label">Strikes</div><div class="stat-value" id="strikes">0</div></div>
        </div>
    </div>
    <div id="keyHint">SPACEBAR = HAMMER | ENTER = ADD MATERIAL</div>
    <canvas id="gameCanvas" width="1200" height="600"></canvas>
    <div id="feedback"></div>
    <div id="controls">
        <button id="submitBtn">Submit Glass</button>
        <button id="resetBtn">Reset</button>
        <div id="robotPosition">
            <div style="font-size:12px;color:#ffaa66;margin-bottom:5px;">Robot Fy (0 - 300 mm)</div>
            <div id="fyValue">150.0 mm</div>
            <div id="positionBar"><div id="positionFill"></div></div>
            <div id="forceIndicator" class="status-ready">Game Ready</div>
        </div>
    </div>

    <div id="overlay">
        <div class="overlay-content">
            <div class="overlay-title" style="color:#ff6600;">Ready to Forge?</div>
            <div style="font-size:20px;color:#ffaa66;margin:20px 0;">
                Push/Pull robot freely<br>SPACEBAR = Hammer | ENTER = Add Glass
            </div>
            <button id="overlayStartBtn">Start Game</button>
        </div>
    </div>

    <div id="resultOverlay">
        <div class="result-content">
            <div class="overlay-title" id="resultTitle"></div>
            <div class="overlay-text" id="resultText"></div>
            <button id="closeResult">Close</button>
        </div>
    </div>

    <script>
        const GLASS_LENGTH_MM = 300;
        const CANVAS_WIDTH = 1200;
        const GLASS_START_X = 180;
        const GLASS_END_X = 1020;
        const PX_PER_MM = (GLASS_END_X - GLASS_START_X) / GLASS_LENGTH_MM;

        let gameState='ready', score=0, hammerBeat=0, robotFy=150;
        let glassBlob=[], targetShape=[], animationId=null;
        let lastHitTime=0, lastHitAccuracy=0;
        const SEGMENTS=30, DEFORM_AMOUNT=15, ADD_AMOUNT=5;
        const canvas=document.getElementById('gameCanvas'), ctx=canvas.getContext('2d');

        function mmToPx(mm) { return GLASS_START_X + mm * PX_PER_MM; }

        let spaceDown=false, enterDown=false;
        function sendKey(){ fetch(`/api/key?space=${spaceDown?1:0}&enter=${enterDown?1:0}`).catch(()=>{}); }
        document.addEventListener('keydown', e=>{
            if(e.code==='Space' && !spaceDown && gameState==='playing'){ spaceDown=true; sendKey(); performHammerStrike(); }
            if(e.code==='Enter' && !enterDown && gameState==='playing'){ enterDown=true; sendKey(); addMaterial(); }
        });
        document.addEventListener('keyup', e=>{
            if(e.code==='Space'){ spaceDown=false; sendKey(); }
            if(e.code==='Enter'){ enterDown=false; sendKey(); }
        });
        setInterval(()=>fetch('/api/heartbeat').catch(()=>{}),1000);

        function initGame(){
            glassBlob = Array.from({length: SEGMENTS}, (_, i) => {
                const mm = (i / (SEGMENTS - 1)) * GLASS_LENGTH_MM;
                return { mm, radius: 25 };
            });
            targetShape = Array.from({length: SEGMENTS}, (_, i) => {
                const t = i / (SEGMENTS - 1);
                let r = t < 0.2 ? 15 + (t/0.2)*5 : t < 0.5 ? 12 : t < 0.7 ? 12 + ((t-0.5)/0.2)*23 : 35 - ((t-0.7)/0.3)*15;
                const mm = t * GLASS_LENGTH_MM;
                return { mm, targetRadius: r };
            });
            score = hammerBeat = 0; updateStats();
        }

        function startGame(){
            document.getElementById('overlay').style.display='none';
            document.getElementById('forceIndicator').textContent='Game Active!';
            document.getElementById('forceIndicator').className='status-playing';
            initGame(); gameState='playing';
            if(!animationId) render();
            pollRobot(); setInterval(pollRobot, 30);
        }

        function performHammerStrike(){
            hammerBeat++;
            const idx = Math.round((robotFy / GLASS_LENGTH_MM) * (SEGMENTS - 1));
            for(let i = Math.max(0, idx-2); i <= Math.min(SEGMENTS-1, idx+2); i++){
                const dist = Math.abs(i - idx);
                const factor = 1 - dist/3;
                glassBlob[i].radius = Math.max(5, glassBlob[i].radius - DEFORM_AMOUNT * factor);
            }
            const err = Math.abs(targetShape[idx].targetRadius - glassBlob[idx].radius);
            const acc = Math.max(0, 100 - err * 3);
            lastHitAccuracy = acc; lastHitTime = Date.now();
            if(acc > 70){ score += Math.round(acc); showFeedback('PERFECT!','perfect'); }
            else if(acc > 40){ score += Math.round(acc/2); showFeedback('Good','good'); }
            else showFeedback('Miss','miss');
            updateStats();
        }

        function addMaterial(){
            const idx = Math.round((robotFy / GLASS_LENGTH_MM) * (SEGMENTS - 1));
            for(let i = Math.max(0, idx-2); i <= Math.min(SEGMENTS-1, idx+2); i++){
                const dist = Math.abs(i - idx);
                const factor = 1 - dist/3;
                glassBlob[i].radius = Math.min(35, glassBlob[i].radius + ADD_AMOUNT * factor);
            }
            showFeedback('Material Added!','material');
            updateStats();
        }

        function submitGlass(){
            const match = getMatchPercentage();
            document.getElementById('resultTitle').textContent = match > 80 ? 'PASSED!' : 'FAILED!';
            document.getElementById('resultTitle').style.color = match > 80 ? '#00ff00' : '#ff4444';
            document.getElementById('resultText').innerHTML = `Match: ${match.toFixed(1)}%<br>Score: ${score}`;
            document.getElementById('resultOverlay').style.display = 'flex';
        }

        function showFeedback(t,c){
            const f = document.getElementById('feedback');
            f.textContent = t; f.className = `feedback-${c}`; f.style.opacity = 1;
            setTimeout(() => f.style.opacity = 0, 600);
        }

        function updateStats(){
            document.getElementById('score').textContent = score;
            document.getElementById('match').textContent = getMatchPercentage().toFixed(0) + '%';
            document.getElementById('strikes').textContent = hammerBeat;
        }

        function getMatchPercentage(){
            const e = glassBlob.reduce((s, v, i) => s + Math.abs(v.radius - targetShape[i].targetRadius), 0);
            return Math.max(0, 100 - (e / (SEGMENTS * 30)) * 100);
        }

        function updateDisplay(){
            document.getElementById('fyValue').textContent = robotFy.toFixed(1) + ' mm';
            document.getElementById('positionFill').style.width = (robotFy / 300 * 100) + '%';
        }

        async function pollRobot(){
            try{
                const res = await fetch('/api/fy?t=' + Date.now(), {cache: 'no-cache'});
                if(res.ok){
                    const data = await res.json();
                    robotFy = Math.max(0, Math.min(300, data.fy));
                    updateDisplay();
                }
            }catch(e){}
        }

        function render(){
            ctx.fillStyle = '#0a0a0a'; ctx.fillRect(0,0,canvas.width,canvas.height);
            const grad = ctx.createRadialGradient(600,300,0,600,300,600);
            grad.addColorStop(0,'rgba(255,100,0,0.15)'); grad.addColorStop(1,'rgba(139,0,0,0.05)');
            ctx.fillStyle = grad; ctx.fillRect(0,0,canvas.width,canvas.height);

            const y1 = 200, y2 = 450;
            ctx.strokeStyle = '#444'; ctx.lineWidth = 8; ctx.beginPath();
            ctx.moveTo(GLASS_START_X - 50, y1); ctx.lineTo(GLASS_END_X + 20, y1); ctx.stroke();

            draw(glassBlob, y1, true);
            draw(targetShape, y2, false);

            const hx = mmToPx(robotFy);
            const drop = spaceDown ? 80 : 0;
            ctx.save(); ctx.translate(hx, 50 + drop);
            ctx.fillStyle = spaceDown ? '#ff4444' : '#666';
            if(spaceDown){ ctx.shadowBlur = 30; ctx.shadowColor = '#ff0000'; }
            ctx.fillRect(-25, -15, 50, 30); ctx.shadowBlur = 0;
            ctx.fillStyle = '#8B4513'; ctx.fillRect(-5, -60, 10, 50); ctx.restore();

            const hammerBottomY = 50 + drop - 15;
            const targetIdx = Math.round((robotFy / GLASS_LENGTH_MM) * (SEGMENTS - 1));
            const targetPointY = y2 + targetShape[targetIdx].targetRadius;
            ctx.setLineDash([6,10]); ctx.strokeStyle = '#00ff00'; ctx.lineWidth = 3; ctx.globalAlpha = 0.9;
            ctx.beginPath(); ctx.moveTo(hx, hammerBottomY); ctx.lineTo(hx, targetPointY); ctx.stroke();
            ctx.setLineDash([]); ctx.globalAlpha = 1;

            if(Date.now() - lastHitTime < 500){
                const a = 1 - (Date.now() - lastHitTime) / 500;
                ctx.globalAlpha = a;
                ctx.fillStyle = lastHitAccuracy > 70 ? '#00ff00' : lastHitAccuracy > 40 ? '#ffff00' : '#ff4444';
                ctx.beginPath(); ctx.arc(hx, y1, 40, 0, Math.PI * 2); ctx.fill();
                ctx.globalAlpha = 1;
            }

            animationId = requestAnimationFrame(render);
        }

        function draw(shape, y, isCurrent){
            ctx.save(); ctx.translate(0, y);
            if(isCurrent){
                const g = ctx.createRadialGradient(mmToPx(150), 0, 0, mmToPx(150), 0, 50);
                g.addColorStop(0,'rgba(255,150,0,0.3)'); g.addColorStop(1,'rgba(255,50,0,0)');
                ctx.fillStyle = g; ctx.fillRect(GLASS_START_X - 20, -60, GLASS_END_X - GLASS_START_X + 40, 120);
            }
            ctx.beginPath();
            shape.forEach((p, i) => {
                const px = mmToPx(p.mm);
                const py = isCurrent ? -p.radius : -p.targetRadius;
                i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
            });
            shape.slice().reverse().forEach(p => {
                const px = mmToPx(p.mm);
                const py = isCurrent ? p.radius : p.targetRadius;
                ctx.lineTo(px, py);
            });
            ctx.closePath();
            if(isCurrent){
                const g = ctx.createLinearGradient(0, -40, 0, 40);
                g.addColorStop(0, '#ff6600'); g.addColorStop(0.5, '#ffaa00'); g.addColorStop(1, '#ff4400');
                ctx.fillStyle = g; ctx.fill();
            } else {
                ctx.globalAlpha = 0.4; ctx.strokeStyle = '#00ffff'; ctx.lineWidth = 2; ctx.stroke();
                ctx.fillStyle = 'rgba(0,255,255,0.1)'; ctx.fill(); ctx.globalAlpha = 1;
            }
            ctx.restore();
        }

        document.addEventListener('DOMContentLoaded', () => {
            initGame(); render(); updateDisplay();
            document.getElementById('overlayStartBtn').onclick = startGame;
            document.getElementById('submitBtn').onclick = submitGlass;
            document.getElementById('closeResult').onclick = () => { document.getElementById('resultOverlay').style.display = 'none'; };
            document.getElementById('resetBtn').onclick = () => {
                cancelAnimationFrame(animationId); animationId = null;
                document.getElementById('overlay').style.display = 'flex';
                initGame(); render();
            };
        });
    </script>
</body>
</html>"""
        response = f"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nCache-Control: no-cache\r\nContent-Length: {len(html)}\r\n\r\n{html}"
        try:
            sock.sendall(response.encode('utf-8'))
        except:
            pass

    def update_fy(self, fy):
        self.current_fy = fy

    def stop(self):
        self.running = False
        if self.server_socket:
            self.server_socket.close()

# ============================= MAIN LOOP - EXACTLY LIKE YOUR REFERENCE CODE =============================
print("\n" + "="*70)
print("GLASS FORGE - ULTRA SMOOTH (MATCHING REFERENCE)")
print("300 mm range | No vibration | Full impedance control")
print("="*70)

init_ft_sensor()
ret, pose = robot.GetActualTCPPose(flag=1)
if ret != 0:
    print("Failed to get pose"); sys.exit(1)

y_start = pose[1]
Y_MIN_ABS = y_start - 150.0
Y_MAX_ABS = y_start + 150.0
fy_offset = y_start - 150.0
baseline = calibrate_baseline_forces(300)

web_server = WebServer()
web_server.start()

# === EXACT IMPEDANCE FROM YOUR CODE ===
M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]
rtn = robot.ForceAndJointImpedanceStartStop(1, 0, M, K, B, 100, 100)
print(f"Impedance ON (rtn: {rtn})")

# === EXACT PARAMETERS FROM YOUR CODE ===
LINEAR_VEL = 5.0
FORCE_THRESHOLD = 2.0
DEADBAND = 0.5
FORCE_TO_VELOCITY_SCALE = 10.0
UPDATE_RATE = 0.008  # 125 Hz

print(f"ROBOT RANGE: {Y_MIN_ABS:.1f} → {Y_MAX_ABS:.1f} mm")
print("FULL 300mm TRAVEL | NO PAUSE | ULTRA SMOOTH")
print("="*70)

try:
    while not web_server.client_connected and not shutdown_requested:
        time.sleep(0.1)
    print("Browser connected! Game active.\n")

    last_print = time.time()
    last_space = last_enter = False

    while not shutdown_requested:
        now = time.time()

        err, pose = robot.GetActualTCPPose()
        if err != 0:
            time.sleep(UPDATE_RATE); continue

        current_y = max(Y_MIN_ABS, min(Y_MAX_ABS, pose[1]))
        game_fy = current_y - fy_offset
        web_server.update_fy(game_fy)

        with key_lock:
            if space_pressed and not last_space:
                print("SPACEBAR to HAMMER!")
            if enter_pressed and not last_enter:
                print("ENTER to ADD MATERIAL!")
            last_space, last_enter = space_pressed, enter_pressed

        ft = robot.FT_GetForceTorqueRCS()
        if ft[0] == 0:
            raw = ft[1][:3]
            compensated = [raw[i] - baseline[i] for i in range(3)]
            for i in range(3):
                if abs(compensated[i]) < DEADBAND:
                    compensated[i] = 0.0
            world_forces = transform_forces_to_tcp_and_world(compensated + [0,0,0], pose[3:6])
            fy_world = force_filter.update(world_forces[1])

            if abs(fy_world) > FORCE_THRESHOLD:
                y_movement = fy_world * FORCE_TO_VELOCITY_SCALE
                y_target = current_y + y_movement
                y_target = max(Y_MIN_ABS, min(Y_MAX_ABS, y_target))

                robot.MoveL(
                    desc_pos=[pose[0], y_target, pose[2], pose[3], pose[4], pose[5]],
                    tool=0, user=0, joint_pos=[0.0]*7,
                    vel=LINEAR_VEL, acc=0.0, ovl=100.0,
                    blendR=-1.0, exaxis_pos=[0.0]*4,
                    search=0, offset_flag=0, offset_pos=[0.0]*6
                )

        if now - last_print > 0.2:
            print(f"Fy: {fy_world:+6.2f}N | Y: {current_y:8.1f} mm | Game: {game_fy:6.1f} mm", end="\r")
            last_print = now

        time.sleep(UPDATE_RATE)

except KeyboardInterrupt:
    pass
finally:
    print("\n\nShutting down...")
    robot.ForceAndJointImpedanceStartStop(0, 0, M, K, B, 100, 100)
    web_server.stop()
    final = robot.GetActualTCPPose(flag=1)
    if final[0] == 0:
        print(f"Final Y: {final[1][1]:.2f} mm (Delta {final[1][1] - y_start:+.2f} mm)")
    print("Thanks for forging!")
    print("="*70)