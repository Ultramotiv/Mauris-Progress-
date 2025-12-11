# neon_air_hockey_3d.py
# TRUE 3D Air Hockey with Three.js
# Full 3D perspective | Realistic physics | Beautiful lighting
# Run: python this_file.py → auto browser

from flask import Flask
from flask_socketio import SocketIO
import webbrowser, threading, time, math, random

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# ==================== CONSTANTS ====================
W, H = 12, 18  # 3D world units
PADDLE_R = 1.2
BALL_R = 0.4
GOAL_W = 3.5
PADDLE_TOP_Z = -7
PADDLE_BOTTOM_Z = 7

# State
p1_x = 0
p2_x = 0
bx, by, bz = 0, 0.5, 0
bvx, bvy, bvz = 0, 0, 0
score_top, score_bottom = 0, 0
reset_ball = False
user_paddle_x = 0
userCanMove = True
ball_touched_by_user = False

def get_state():
    return {
        "p1": p1_x, "p2": user_paddle_x, 
        "bx": bx, "by": by, "bz": bz,
        "s1": score_top, "s2": score_bottom
    }

# ==================== TRUE 3D HTML WITH THREE.JS ====================
HTML = """
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>3D NEON AIR HOCKEY</title>
<style>
body {
  margin: 0;
  overflow: hidden;
  background: #000;
  font-family: 'Arial Black', sans-serif;
}
#score {
  position: absolute;
  top: 20px;
  left: 50%;
  transform: translateX(-50%);
  color: #fff;
  font-size: 48px;
  text-shadow: 0 0 20px #0ff, 0 0 40px #0ff;
  z-index: 100;
  letter-spacing: 8px;
}
#info {
  position: absolute;
  bottom: 20px;
  left: 50%;
  transform: translateX(-50%);
  color: rgba(255,255,255,0.6);
  font-size: 14px;
  z-index: 100;
}
canvas {
  display: block;
  cursor: none;
}
</style>
</head>
<body>
<div id="score">0 – 0</div>
<div id="info">Move mouse to control RED paddle</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
<script>
const socket = io();
let scene, camera, renderer;
let table, topPaddle, bottomPaddle, ball;
let walls = [];
let userCanMove = true;

// Initialize Three.js scene
function init() {
  scene = new THREE.Scene();
  scene.fog = new THREE.Fog(0x000000, 10, 50);
  
  // Camera with perspective view
  camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);
  camera.position.set(0, 15, 10);
  camera.lookAt(0, 0, 0);
  
  // Renderer
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  document.body.appendChild(renderer.domElement);
  
  // Ambient light
  const ambient = new THREE.AmbientLight(0x404040, 0.5);
  scene.add(ambient);
  
  // Directional light
  const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
  dirLight.position.set(5, 20, 5);
  dirLight.castShadow = true;
  dirLight.shadow.camera.left = -15;
  dirLight.shadow.camera.right = 15;
  dirLight.shadow.camera.top = 15;
  dirLight.shadow.camera.bottom = -15;
  scene.add(dirLight);
  
  // Table
  const tableGeo = new THREE.BoxGeometry(12, 0.5, 18);
  const tableMat = new THREE.MeshStandardMaterial({ 
    color: 0x111111,
    roughness: 0.3,
    metalness: 0.8
  });
  table = new THREE.Mesh(tableGeo, tableMat);
  table.position.y = -0.25;
  table.receiveShadow = true;
  scene.add(table);
  
  // Add hexagon pattern texture
  addHexagonPattern();
  
  // Center line
  const lineGeo = new THREE.BoxGeometry(12, 0.05, 0.1);
  const lineMat = new THREE.MeshBasicMaterial({ color: 0x444444 });
  const centerLine = new THREE.Mesh(lineGeo, lineMat);
  centerLine.position.y = 0.05;
  scene.add(centerLine);
  
  // Walls with rainbow colors
  createWall(0, 0, -9, 12, 2, 0.2, 0x00ff00); // Top - green
  createWall(0, 0, 9, 12, 2, 0.2, 0xffff00);  // Bottom - yellow
  createWall(-6, 0, 0, 0.2, 2, 18, [0xff0000, 0xff00ff, 0x00ff00]); // Left - gradient
  createWall(6, 0, 0, 0.2, 2, 18, [0x00ffff, 0xff00ff, 0xffff00]);  // Right - gradient
  
  // Goal areas
  createGoal(0, -9, 0x00ffff);
  createGoal(0, 9, 0xff0000);
  
  // Top paddle (blue) - AI
  const paddleGeo = new THREE.CylinderGeometry(1.2, 1.2, 0.4, 32);
  const topPaddleMat = new THREE.MeshStandardMaterial({ 
    color: 0x0088ff,
    emissive: 0x0066ff,
    emissiveIntensity: 0.5,
    roughness: 0.2,
    metalness: 0.8
  });
  topPaddle = new THREE.Mesh(paddleGeo, topPaddleMat);
  topPaddle.rotation.x = Math.PI / 2;
  topPaddle.position.y = 0.3;
  topPaddle.castShadow = true;
  scene.add(topPaddle);
  
  // Add glow to top paddle
  addGlow(topPaddle, 0x0088ff, 2);
  
  // Bottom paddle (red) - USER
  const bottomPaddleMat = new THREE.MeshStandardMaterial({ 
    color: 0xff0033,
    emissive: 0xff0022,
    emissiveIntensity: 0.5,
    roughness: 0.2,
    metalness: 0.8
  });
  bottomPaddle = new THREE.Mesh(paddleGeo, bottomPaddleMat);
  bottomPaddle.rotation.x = Math.PI / 2;
  bottomPaddle.position.y = 0.3;
  bottomPaddle.castShadow = true;
  scene.add(bottomPaddle);
  
  // Add glow to bottom paddle
  addGlow(bottomPaddle, 0xff0033, 2);
  
  // Ball
  const ballGeo = new THREE.SphereGeometry(0.4, 32, 32);
  const ballMat = new THREE.MeshStandardMaterial({ 
    color: 0xffffff,
    emissive: 0xffff00,
    emissiveIntensity: 0.8,
    roughness: 0.1,
    metalness: 0.9
  });
  ball = new THREE.Mesh(ballGeo, ballMat);
  ball.castShadow = true;
  scene.add(ball);
  
  // Add intense glow to ball
  addGlow(ball, 0xffff00, 3);
  
  // Point light from ball
  const ballLight = new THREE.PointLight(0xffff00, 2, 10);
  ball.add(ballLight);
  
  animate();
}

function addHexagonPattern() {
  const hexMaterial = new THREE.LineBasicMaterial({ color: 0x333333, transparent: true, opacity: 0.3 });
  const hexSize = 0.5;
  
  for (let z = -9; z < 9; z += hexSize * 1.5) {
    for (let x = -6; x < 6; x += hexSize * Math.sqrt(3)) {
      const offsetX = (z / (hexSize * 1.5)) % 2 === 0 ? 0 : hexSize * Math.sqrt(3) / 2;
      const points = [];
      for (let i = 0; i < 7; i++) {
        const angle = (Math.PI / 3) * i;
        points.push(new THREE.Vector3(
          x + offsetX + hexSize * Math.cos(angle),
          0.01,
          z + hexSize * Math.sin(angle)
        ));
      }
      const hexGeo = new THREE.BufferGeometry().setFromPoints(points);
      const hex = new THREE.Line(hexGeo, hexMaterial);
      scene.add(hex);
    }
  }
}

function createWall(x, y, z, w, h, d, color) {
  const wallGeo = new THREE.BoxGeometry(w, h, d);
  let wallMat;
  
  if (Array.isArray(color)) {
    // Gradient wall
    wallMat = new THREE.MeshStandardMaterial({ 
      color: color[Math.floor(color.length / 2)],
      emissive: color[Math.floor(color.length / 2)],
      emissiveIntensity: 0.3
    });
  } else {
    wallMat = new THREE.MeshStandardMaterial({ 
      color: color,
      emissive: color,
      emissiveIntensity: 0.4
    });
  }
  
  const wall = new THREE.Mesh(wallGeo, wallMat);
  wall.position.set(x, y + 1, z);
  scene.add(wall);
  
  // Add glow
  const glowGeo = new THREE.BoxGeometry(w * 1.1, h * 1.1, d * 1.1);
  const glowMat = new THREE.MeshBasicMaterial({ 
    color: Array.isArray(color) ? color[1] : color,
    transparent: true,
    opacity: 0.2
  });
  const glow = new THREE.Mesh(glowGeo, glowMat);
  glow.position.copy(wall.position);
  scene.add(glow);
  
  walls.push(wall);
}

function createGoal(x, z, color) {
  // Goal arc
  const curve = new THREE.EllipseCurve(x, z, 3.5, 3.5, 0, Math.PI, false, 0);
  const points = curve.getPoints(50);
  const goalGeo = new THREE.BufferGeometry().setFromPoints(
    points.map(p => new THREE.Vector3(p.x, 0.01, p.y))
  );
  const goalMat = new THREE.LineBasicMaterial({ color: 0x666666, linewidth: 2 });
  const goal = new THREE.Line(goalGeo, goalMat);
  scene.add(goal);
  
  // Goal markers
  const markerGeo = new THREE.ConeGeometry(0.3, 0.6, 3);
  const markerMat = new THREE.MeshStandardMaterial({ 
    color: 0xffffff,
    emissive: 0xffffff,
    emissiveIntensity: 0.5
  });
  const marker = new THREE.Mesh(markerGeo, markerMat);
  marker.rotation.x = Math.PI;
  marker.position.set(x, 0.6, z + (z > 0 ? -1.5 : 1.5));
  scene.add(marker);
}

function addGlow(mesh, color, intensity) {
  const glowGeo = mesh.geometry.clone();
  const glowMat = new THREE.MeshBasicMaterial({ 
    color: color,
    transparent: true,
    opacity: 0.3,
    side: THREE.BackSide
  });
  const glow = new THREE.Mesh(glowGeo, glowMat);
  glow.scale.multiplyScalar(1.3);
  mesh.add(glow);
}

function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}

function updateState(state) {
  topPaddle.position.x = state.p1;
  topPaddle.position.z = -7;
  
  bottomPaddle.position.x = state.p2;
  bottomPaddle.position.z = 7;
  
  ball.position.x = state.bx;
  ball.position.y = state.by;
  ball.position.z = state.bz;
  
  // Rotate ball based on movement
  ball.rotation.x += 0.1;
  ball.rotation.y += 0.05;
  
  document.getElementById('score').textContent = state.s1 + ' – ' + state.s2;
}

socket.on('state', updateState);

// Mouse control for bottom paddle
document.addEventListener('mousemove', (e) => {
  if (!userCanMove) return;
  const x = (e.clientX / window.innerWidth) * 2 - 1;
  socket.emit('user_paddle', x * 5);
});

// Touch control
document.addEventListener('touchmove', (e) => {
  if (!userCanMove) return;
  const x = (e.touches[0].clientX / window.innerWidth) * 2 - 1;
  socket.emit('user_paddle', x * 5);
}, { passive: true });

socket.on('userCanMove', (val) => {
  userCanMove = val;
});

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

init();
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return HTML

@socketio.on("user_paddle")
def handle_user_paddle(x):
    global user_paddle_x
    user_paddle_x = max(-5, min(5, x))

# ==================== 3D PHYSICS LOOP ====================
def game_loop():
    global p1_x, user_paddle_x, bx, by, bz, bvx, bvy, bvz
    global score_top, score_bottom, reset_ball, userCanMove, ball_touched_by_user
    
    clock = 0
    while True:
        clock += 1
        
        # AI paddle follows ball
        target1 = bx + (bvx * 0.3 if clock % 120 < 60 else 0)
        p1_x += (target1 - p1_x) * 0.3
        p1_x = max(-5, min(5, p1_x))
        
        if reset_ball:
            bx, bz = 0, 0
            by = 0.5
            direction = random.choice([-1, 1])
            bvx = direction * random.uniform(0.08, 0.12)
            bvz = random.uniform(-0.05, 0.05)
            bvy = 0
            reset_ball = False
            reset_user_control()
        
        # Move ball
        bx += bvx
        bz += bvz
        by += bvy
        
        # Gravity
        bvy -= 0.001
        if by < 0.4:
            by = 0.4
            bvy = 0
        
        # Side walls
        if abs(bx) > 5.8:
            bx = 5.8 * (1 if bx > 0 else -1)
            bvx = -bvx * 0.9
        
        # Top paddle collision
        dist1 = math.hypot(bx - p1_x, bz - PADDLE_TOP_Z)
        if dist1 < PADDLE_R + BALL_R and bz < PADDLE_TOP_Z + 0.5:
            angle = math.atan2(bz - PADDLE_TOP_Z, bx - p1_x)
            speed = math.hypot(bvx, bvz) * 1.1
            bvx = math.cos(angle) * speed
            bvz = math.sin(angle) * speed
            overlap = PADDLE_R + BALL_R - dist1
            bx += math.cos(angle) * overlap
            bz += math.sin(angle) * overlap
        
        # Bottom paddle collision
        dist2 = math.hypot(bx - user_paddle_x, bz - PADDLE_BOTTOM_Z)
        if dist2 < PADDLE_R + BALL_R and bz > PADDLE_BOTTOM_Z - 0.5:
            if userCanMove:
                ball_touched_by_user = True
                userCanMove = False
                socketio.emit("userCanMove", False)
            angle = math.atan2(bz - PADDLE_BOTTOM_Z, bx - user_paddle_x)
            speed = math.hypot(bvx, bvz) * 1.1
            bvx = math.cos(angle) * speed
            bvz = math.sin(angle) * speed
            overlap = PADDLE_R + BALL_R - dist2
            bx += math.cos(angle) * overlap
            bz += math.sin(angle) * overlap
        
        # Return to center check
        if ball_touched_by_user and not userCanMove:
            if abs(bz) < 1 and abs(bx) < 1:
                reset_user_control()
        
        # Goals
        if bz < -8.5 and abs(bx) < GOAL_W:
            score_bottom += 1
            reset_ball = True
        elif bz > 8.5 and abs(bx) < GOAL_W:
            score_top += 1
            reset_ball = True
        
        # Wall bounces at goals
        if bz < -8.8:
            bz = -8.8
            bvz = -bvz * 0.9
        elif bz > 8.8:
            bz = 8.8
            bvz = -bvz * 0.9
        
        socketio.emit("state", get_state())
        time.sleep(1 / 60)

def reset_user_control():
    global userCanMove, ball_touched_by_user
    userCanMove = True
    ball_touched_by_user = False
    socketio.emit("userCanMove", True)

# ==================== LAUNCH ====================
if __name__ == "__main__":
    def open_browser():
        time.sleep(1.2)
        webbrowser.open("http://127.0.0.1:5000")

    threading.Thread(target=open_browser, daemon=True).start()
    threading.Thread(target=game_loop, daemon=True).start()

    print("🎮 TRUE 3D NEON AIR HOCKEY LAUNCHED!")
    print("Full Three.js 3D rendering | Realistic perspective")
    print("Control RED paddle with mouse!")

    socketio.run(app, host="127.0.0.1", port=5000, debug=False)