# ROBOT JOINT 6 → 3D FIRST-PERSON DRIVER (ULTRA SMOOTH VERSION)

import sys
import time
import signal
import threading
import numpy as np
import os
import json
import webbrowser
import socket
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn

sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

# ====================== GLOBAL SHARED DATA ======================
shared_joint6_angle = 0.0
shared_force_data = {'mz': 0.0, 'velocity': 0.0}
running = True
lock = threading.Lock()

# ====================== ROBOT PARAMETERS (ULTRA SMOOTH & RESPONSIVE!) ======================
M = [3.0, 3.0, 2.0, 3.0, 3.0, 0.8]  # Lower mass = more responsive
B = [2.5, 2.5, 2.5, 3.0, 3.0, 1.2]  # Lower damping = smoother, easier rotation
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # No spring stiffness
force_to_deg = 15.0  # Higher = more sensitive, easier to move
dt = 0.008  # Control loop timing
deadband = 0.15  # Lower deadband = more responsive to small forces
gravity_compensation_samples = 100
baseline_forces = None
free_joints = [6]

JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0), 2: (-179.0, -35.0), 3: (-158.0, 158.0),
    4: (-264.0, 80.0), 5: (-170.0, 12.0), 6: (-50.0, 140.0),
}

# ====================== HTML CONTENT ======================

HTML_CONTENT = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Robot Joint 6 → SLOW HIGHWAY DRIVE</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <style>
        body { margin: 0; overflow: hidden; font-family: Arial; background: #000; }
        #info { position: absolute; top: 20px; left: 20px; color: white; font-size: 24px; font-weight: bold; background: rgba(0,0,0,0.8); padding: 15px; border-radius: 10px; z-index: 100; }
        #debug { position: absolute; top: 120px; left: 20px; color: #0f0; font-size: 14px; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; font-family: monospace; }
        #dashboard { position: absolute; bottom: 30px; left: 50%; transform: translateX(-50%); width: 450px; height: 130px; background: rgba(20,20,30,0.85); border: 3px solid #00ffff; border-radius: 15px; display: flex; justify-content: space-around; align-items: center; padding: 15px; }
        .gauge { text-align: center; color: #00ffff; }
        .gauge-value { font-size: 36px; font-weight: bold; color: #0f0; }
        .gauge-label { font-size: 14px; margin-top: 5px; }
        #gameover { 
            position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);
            color: #ff4444; font-size: 48px; font-weight: bold; text-align: center;
            background: rgba(0,0,0,0.9); padding: 30px; border-radius: 20px; display: none;
            border: 4px solid #ff0000; z-index: 200;
        }
        #score { font-size: 24px; margin-top: 10px; color: #ffff00; }
    </style>
</head>
<body>
    <div id="info">
        <div>🚗 SLOW HIGHWAY DRIVE</div>
        <div style="font-size: 18px;">Angle: <span id="joint6">0.0</span>° | Speed: 10 km/h</div>
        <div style="font-size: 18px;">Score: <span id="score">0</span>m</div>
    </div>
    <div id="debug">
        <div>Connecting...</div>
        <div>Force: <span id="force-mz">0.0</span> N·m</div>
        <div>Velocity: <span id="velocity">0.0</span> deg/s</div>
        <div style="color: yellow;">-50° → RIGHT | 140° → LEFT</div>
    </div>
    <div id="dashboard">
        <div class="gauge">
            <div class="gauge-value" id="angle-display">0°</div>
            <div class="gauge-label">JOINT 6</div>
        </div>
        <div class="gauge" style="padding: 10px 20px; background: rgba(0,0,0,0.7); border: 2px solid #0f0; border-radius: 8px;">
            <div style="font-size: 16px; color: #888;">LANE</div>
            <div id="lane" style="margin-top: 5px; font-size: 28px;">CENTER</div>
        </div>
        <div class="gauge">
            <div class="gauge-value" id="position-display">0.0</div>
            <div class="gauge-label">POSITION</div>
        </div>
    </div>
    <div id="gameover">
        <div>GAME OVER</div>
        <div style="font-size: 24px; margin-top: 20px;">Final Score: <span id="final-score">0</span>m</div>
        <div style="font-size: 18px; margin-top: 10px; color: #888;">Restarting in 3s...</div>
    </div>

    <script>
        console.log('=== SLOW HIGHWAY DRIVE - 10 KM/H ===');
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x87ceeb);
        scene.fog = new THREE.Fog(0x87ceeb, 50, 300);

        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        camera.position.set(0, 1.8, 3);

        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        document.body.appendChild(renderer.domElement);

        scene.add(new THREE.AmbientLight(0xffffff, 0.7));
        const sun = new THREE.DirectionalLight(0xffffff, 0.6);
        sun.position.set(100, 100, 50);
        scene.add(sun);

        // === HIGHWAY ===
        const laneWidth = 4.0;
        const roadWidth = laneWidth * 3;
        const roadSegments = [];

        function createRoad(z) {
            const group = new THREE.Group();
            const road = new THREE.Mesh(
                new THREE.PlaneGeometry(roadWidth + 4, 20),
                new THREE.MeshLambertMaterial({ color: 0x222222 })
            );
            road.rotation.x = -Math.PI / 2;
            group.add(road);

            for (let i = -1; i <= 1; i++) {
                const isCenter = i === 0;
                const color = isCenter ? 0xffff00 : 0xffffff;
                for (let j = 0; j < 5; j++) {
                    const dash = new THREE.Mesh(
                        new THREE.PlaneGeometry(0.15, isCenter ? 3 : 2.5),
                        new THREE.MeshBasicMaterial({ color })
                    );
                    dash.rotation.x = -Math.PI / 2;
                    dash.position.set(i * laneWidth, 0.01, -10 + j * 4);
                    group.add(dash);
                }
            }

            [-roadWidth/2 - 0.3, roadWidth/2 + 0.3].forEach(x => {
                const line = new THREE.Mesh(
                    new THREE.PlaneGeometry(0.3, 20),
                    new THREE.MeshBasicMaterial({ color: 0xffffff })
                );
                line.rotation.x = -Math.PI / 2;
                line.position.set(x, 0.02, 0);
                group.add(line);
            });

            [-roadWidth/2 - 10, roadWidth/2 + 10].forEach(x => {
                const grass = new THREE.Mesh(
                    new THREE.PlaneGeometry(20, 20),
                    new THREE.MeshLambertMaterial({ color: 0x2d5a2d })
                );
                grass.rotation.x = -Math.PI / 2;
                grass.position.set(x, -0.1, 0);
                group.add(grass);
            });

            group.position.z = z;
            return group;
        }

        for (let i = 0; i < 40; i++) {
            const segment = createRoad(-i * 20);
            roadSegments.push(segment);
            scene.add(segment);
        }

        // === PLAYER CAR (RED) ===
        const playerCar = new THREE.Group();
        const playerBody = new THREE.Mesh(
            new THREE.BoxGeometry(1.9, 0.8, 4.0),
            new THREE.MeshLambertMaterial({ color: 0xcc0000 })
        );
        playerBody.position.y = 0.4;
        playerCar.add(playerBody);

        const playerRoof = new THREE.Mesh(
            new THREE.BoxGeometry(1.5, 0.6, 1.8),
            new THREE.MeshLambertMaterial({ color: 0xaa0000 })
        );
        playerRoof.position.y = 1.0;
        playerCar.add(playerRoof);

        const playerHood = new THREE.Mesh(
            new THREE.BoxGeometry(1.9, 0.1, 1.6),
            new THREE.MeshLambertMaterial({ color: 0xff2222 })
        );
        playerHood.position.set(0, 0.65, -1.1);
        playerCar.add(playerHood);

        const playerWheelGeo = new THREE.CylinderGeometry(0.33, 0.33, 0.3, 16);
        const playerWheelMat = new THREE.MeshLambertMaterial({ color: 0x111111 });
        const playerWheels = [];
        const playerPositions = [[-0.8, 1.1], [0.8, 1.1], [-0.8, -1.1], [0.8, -1.1]];
        playerPositions.forEach(pos => {
            const wheel = new THREE.Mesh(playerWheelGeo, playerWheelMat);
            wheel.rotation.x = Math.PI / 2;
            wheel.position.set(pos[0], 0.35, pos[1]);
            playerCar.add(wheel);
            playerWheels.push(wheel);
        });
        const playerFrontLeft = playerWheels[0];
        const playerFrontRight = playerWheels[1];

        playerCar.position.y = 0.1;
        scene.add(playerCar);

        // === AI CARS (ONE AT A TIME) ===
        const aiCars = [];
        const aiCarLength = 4.0;
        const aiCarWidth = 1.9;
        const collisionRadius = 2.2;

        const aiColors = [
            0x00ff00, 0x00ffff, 0xffff00, 0xff00ff,
            0x0066ff, 0xff8800, 0x88ff00, 0xff66cc
        ];

        function createAICar() {
            const aiCar = new THREE.Group();
            const color = aiColors[Math.floor(Math.random() * aiColors.length)];

            const aiBody = new THREE.Mesh(
                new THREE.BoxGeometry(aiCarWidth, 0.8, aiCarLength),
                new THREE.MeshLambertMaterial({ color })
            );
            aiBody.position.y = 0.4;
            aiCar.add(aiBody);

            const aiRoof = new THREE.Mesh(
                new THREE.BoxGeometry(1.5, 0.6, 2.0),
                new THREE.MeshLambertMaterial({ color: color * 0.8 })
            );
            aiRoof.position.y = 1.0;
            aiCar.add(aiRoof);

            aiCar.position.y = 0.1;
            const spawnZ = -80 - Math.random() * 60;
            aiCar.userData = { lane: 0, z: spawnZ };

            scene.add(aiCar);
            return aiCar;
        }

        function spawnAICar() {
            const carsAhead = aiCars.filter(car => car.userData.z < 30);
            if (carsAhead.length > 0) return;

            const laneIndex = Math.floor(Math.random() * 3) - 1;
            const newCar = createAICar();
            newCar.userData.lane = laneIndex;
            newCar.position.x = laneIndex * laneWidth;
            aiCars.push(newCar);
        }

        // === GAME STATE ===
        let joint6Angle = 0;
        let playerX = 0;
        let score = 0;
        let gameOver = false;
        let connected = false;
        let gameStartTime = Date.now();

        function getTargetX(angle) {
            const minAngle = -50.0;  // RIGHT
            const maxAngle = 140.0;  // LEFT
            const range = maxAngle - minAngle;
            const normalized = (angle - minAngle) / range;
            return THREE.MathUtils.lerp(4.0, -4.0, normalized);  // 4.0=RIGHT, -4.0=LEFT
        }

        function checkCollision() {
            if (gameOver) return false;
            for (let aiCar of aiCars) {
                const dx = playerX - aiCar.position.x;
                const dz = 0 - aiCar.userData.z;
                const distance = Math.sqrt(dx*dx + dz*dz);
                if (distance < collisionRadius) {
                    gameOver = true;
                    document.getElementById('final-score').textContent = Math.floor(score);
                    document.getElementById('gameover').style.display = 'block';
                    setTimeout(() => location.reload(), 3000);
                    return true;
                }
            }
            return false;
        }

        function updateRobotData() {
            fetch('/data')
                .then(r => r.json())
                .then(data => {
                    joint6Angle = data.joint6;
                    document.getElementById('force-mz').textContent = data.mz.toFixed(2);
                    document.getElementById('velocity').textContent = data.velocity.toFixed(2);
                    if (!connected) {
                        connected = true;
                        document.getElementById('debug').querySelector('div').innerHTML = 'Robot Connected';
                    }
                })
                .catch(() => {
                    document.getElementById('debug').querySelector('div').innerHTML = 'Disconnected';
                });
        }
        setInterval(updateRobotData, 50);

        function animate() {
            requestAnimationFrame(animate);

            if (gameOver) {
                renderer.render(scene, camera);
                return;
            }

            const targetX = getTargetX(joint6Angle);
            playerX += (targetX - playerX) * 0.08;

            playerCar.position.x = playerX;
            playerCar.rotation.y = 0;

            const wheelTurn = (joint6Angle + 50.0) / 190.0 * 0.7 - 0.35;
            playerFrontLeft.rotation.z = wheelTurn;
            playerFrontRight.rotation.z = wheelTurn;

            for (let i = aiCars.length - 1; i >= 0; i--) {
                const aiCar = aiCars[i];
                aiCar.userData.z += 0.208;
                aiCar.position.z = aiCar.userData.z;

                if (aiCar.userData.z > 20) {
                    scene.remove(aiCar);
                    aiCars.splice(i, 1);
                }
            }

            if (Math.random() < 0.005) {
                spawnAICar();
            }

            checkCollision();

            camera.position.x = playerX;
            camera.position.y = 1.8;
            camera.position.z = 3;
            camera.lookAt(playerX, 0.5, -10);

            roadSegments.forEach(seg => {
                seg.position.z += 0.208;
                if (seg.position.z > 20) seg.position.z -= 800;
            });

            score = (Date.now() - gameStartTime) / 1000 * 10;
            document.getElementById('score').textContent = Math.floor(score);

            document.getElementById('joint6').textContent = joint6Angle.toFixed(1);
            document.getElementById('angle-display').textContent = joint6Angle.toFixed(0) + '°';
            document.getElementById('position-display').textContent = playerX.toFixed(2);

            let lane = 'CENTER', color = '#0f0';
            if (playerX > 2.5) { lane = 'RIGHT'; color = '#ff0'; }
            else if (playerX < -2.5) { lane = 'LEFT'; color = '#ff0'; }
            const laneEl = document.getElementById('lane');
            laneEl.textContent = lane;
            laneEl.style.color = color;
            laneEl.parentElement.style.borderColor = color;

            renderer.render(scene, camera);
        }

        window.addEventListener('resize', () => {
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
        });

        animate();
    </script>
</body>
</html>
"""

# ====================== WEB SERVER ======================
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

class RobotDataHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode())
        elif self.path == '/data':
            with lock:
                data = {
                    'joint6': shared_joint6_angle,
                    'mz': shared_force_data['mz'],
                    'velocity': shared_force_data['velocity']
                }
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(data).encode())
        else:
            self.send_error(404)
    
    def log_message(self, format, *args):
        pass

def web_server_thread():
    server = ThreadedHTTPServer(('0.0.0.0', 8080), RobotDataHandler)
    print("🌐 Web server: http://localhost:8080")
    
    def open_browser():
        time.sleep(3)
        webbrowser.open('http://localhost:8080')
    
    threading.Thread(target=open_browser, daemon=True).start()
    server.serve_forever()

# ====================== ROBOT THREAD (ULTRA SMOOTH) ======================
def robot_control_thread():
    global shared_joint6_angle, shared_force_data, baseline_forces, running

    robot = Robot.RPC('192.168.58.2')
    print("🤖 Robot connected")

    company, device = 24, 0
    robot.FT_SetConfig(company, device)
    robot.FT_Activate(0)
    time.sleep(0.5)
    robot.FT_Activate(1)
    time.sleep(0.5)
    robot.SetLoadWeight(0, 0.0)
    robot.SetLoadCoord(0.0, 0.0, 0.0)
    robot.FT_SetZero(0)
    time.sleep(0.5)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("✓ FT sensor ready")

    print("📊 Calibrating...")
    samples = []
    for _ in range(gravity_compensation_samples):
        data = robot.FT_GetForceTorqueRCS()
        if data[0] == 0:
            f = [data[1][0], -data[1][1], data[1][2], data[1][3], data[1][4], data[1][5]]
            samples.append(f)
        time.sleep(0.01)
    
    if samples:
        baseline_forces = np.mean(samples, axis=0).tolist()
        print(f"✓ Baseline: {[round(x,2) for x in baseline_forces]}")

    err, joint_pos = robot.GetActualJointPosDegree()
    if err != 0:
        print("❌ Cannot read joints")
        return

    home_pos = joint_pos.copy()
    desired_pos = joint_pos.copy()
    velocity = [0.0] * 6

    if robot.ServoMoveStart() != 0:
        print("❌ Failed to start servo")
        return
    
    time.sleep(1.0)
    print("🎮 TWIST WRIST TO STEER! (ULTRA SMOOTH & EASY)")

    j6 = 5
    
    while running:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt)
            continue

        raw = [ft_data[1][0], -ft_data[1][1], ft_data[1][2], 
               ft_data[1][3], ft_data[1][4], ft_data[1][5]]
        
        forces = [raw[i] - baseline_forces[i] for i in range(6)] if baseline_forces else raw

        # Apply deadband
        for i in range(6):
            if abs(forces[i]) < deadband:
                forces[i] = 0.0

        mz = forces[5]
        
        # SMOOTHER CONTROL - Lower threshold for easier movement
        if abs(mz) < 0.3:  # Lower threshold = more responsive
            home_pos[j6] = desired_pos[j6]
            spring = -K[j6] * (desired_pos[j6] - home_pos[j6]) / force_to_deg
            acc = (spring - B[j6] * velocity[j6]) / M[j6]
        else:
            acc = (mz - B[j6] * velocity[j6]) / M[j6]

        velocity[j6] += acc * dt
        desired_pos[j6] += velocity[j6] * dt * force_to_deg

        # Lock other joints
        for i in range(6):
            if i+1 not in free_joints:
                desired_pos[i] = home_pos[i]
                velocity[i] = 0.0

        # Apply safety limits
        desired_pos[j6] = np.clip(desired_pos[j6], *JOINT_SAFETY_LIMITS[6])
        
        # Clean up tiny floating point values that break XML-RPC
        cleaned_pos = []
        for val in desired_pos:
            # Round to 6 decimal places and ensure valid range
            clean_val = round(float(val), 6)
            # Replace extremely small values with 0
            if abs(clean_val) < 1e-10:
                clean_val = 0.0
            cleaned_pos.append(clean_val)
        
        # Send smooth command
        robot.ServoJ(cleaned_pos, [0]*6)

        with lock:
            shared_joint6_angle = desired_pos[j6]
            shared_force_data['mz'] = mz
            shared_force_data['velocity'] = velocity[j6]

        time.sleep(dt)

    robot.ServoMoveEnd()
    print("🛑 Stopped")

# ====================== SHUTDOWN ======================
def shutdown(sig, frame):
    global running
    running = False
    print("\n🛑 Shutting down...")
    time.sleep(1)
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# ====================== START ======================
if __name__ == "__main__":
    print("=" * 60)
    print("🚗 ROBOT JOINT 6 → 3D DRIVER (ULTRA SMOOTH)")
    print("=" * 60)
    
    threading.Thread(target=web_server_thread, daemon=True).start()
    threading.Thread(target=robot_control_thread, daemon=True).start()
    
    try:
        while running:
            time.sleep(1)
    except KeyboardInterrupt:
        shutdown(None, None)