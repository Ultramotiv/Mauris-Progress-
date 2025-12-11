# neon_air_hockey_vertical_responsive.py
# Glowing vertical Air Hockey: RESPONSIVE + REALISTIC PHYSICS + semicircle goals
# Fits ANY screen perfectly | Ball bounces walls outside goals | Proper hit angles
# Run: python this_file.py → auto browser

from flask import Flask
from flask_socketio import SocketIO
import webbrowser, threading, time, math, random

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# ==================== CONSTANTS ====================
W, H = 600, 900
PADDLE_W, PADDLE_H = 120, 24
BALL_R = 20
GOAL_R = 140
GOAL_CENTER_X = 300
PADDLE_TOP_Y = 90
PADDLE_BOTTOM_Y = 780
PADDLE_CENTER_Y_TOP = PADDLE_TOP_Y + PADDLE_H / 2
PADDLE_CENTER_Y_BOTTOM = PADDLE_BOTTOM_Y + PADDLE_H / 2

# State
p1_x = W // 2 - PADDLE_W // 2
p2_x = W // 2 - PADDLE_W // 2
bx, by = W // 2, H // 2
bvx, bvy = 0, 0
score_top, score_bottom = 0, 0
reset_ball = False
user_paddle_x = W // 2 - PADDLE_W // 2
userCanMove = True
ball_touched_by_user = False

def get_state():
    return {"p1": p1_x, "p2": user_paddle_x, "bx": bx, "by": by, "s1": score_top, "s2": score_bottom}

# ==================== RESPONSIVE GLOWING NEON HTML ====================
HTML = """
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>NEON AIR HOCKEY</title>
<style>
body {margin:0;background:radial-gradient(circle at center,#000222,#000011);overflow:hidden;display:flex;flex-direction:column;align-items:center;justify-content:center;height:100vh;font-family:Arial,sans-serif;}
#score {color:#0ff;font:bold 60px Arial;text-shadow:0 0 30px #0ff;margin:10px 0;transition:font-size 0.3s;}
canvas {image-smoothing:never;border-radius:20px;box-shadow:0 0 80px #0ff, inset 0 0 80px rgba(0,255,255,0.1);}
</style>
</head>
<body>
<div id="score">0 – 0</div>
<canvas id="c"></canvas>

<script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
<script>
const canvas = document.getElementById("c");
const ctx = canvas.getContext("2d");
const score = document.getElementById("score");
const socket = io();
let scale = 1;
let userCanMove = true;
let lastTouch = false;

function resizeCanvas() {
  const maxW = window.innerWidth * 0.98;
  const maxH = window.innerHeight * 0.90;
  const scaleW = maxW / 600;
  const scaleH = maxH / 900;
  scale = Math.min(scaleW, scaleH);
  canvas.width = Math.floor(600 * scale);
  canvas.height = Math.floor(900 * scale);
  canvas.style.width = canvas.width + "px";
  canvas.style.height = canvas.height + "px";
  canvas.style.borderRadius = (25 * scale) + "px";
  ctx.imageSmoothingEnabled = false;
  score.style.fontSize = Math.min(65 * scale, 72) + "px";
  score.style.textShadow = `0 0 ${35 * scale}px #0ff, 0 0 ${15 * scale}px #fff`;
}

function glow(x, y, r, color, intensity) {
  const grad = ctx.createRadialGradient(x, y, 0, x, y, r + intensity);
  grad.addColorStop(0, `rgba(${color}, 0.9)`);
  grad.addColorStop(0.5, `rgba(${color}, 0.4)`);
  grad.addColorStop(1, `rgba(${color}, 0)`);
  ctx.fillStyle = grad;
  const gr = r + intensity;
  ctx.fillRect(x - gr, y - gr, gr * 2, gr * 2);
}

function draw(state) {
  const s = scale;
  const cw = canvas.width;
  const ch = canvas.height;

  // Background
  ctx.fillStyle = "#000811";
  ctx.fillRect(0, 0, cw, ch);

  // Aura glow
  glow(cw / 2, ch / 2, cw * 0.7, "0,180,255", cw * 0.08);

  // Center line
  ctx.strokeStyle = "#00ffff";
  ctx.lineWidth = 10 * s;
  ctx.shadowBlur = 35 * s;
  ctx.shadowColor = "#00ffff";
  ctx.setLineDash([28 * s, 28 * s]);
  ctx.beginPath();
  ctx.moveTo(0, 450 * s);
  ctx.lineTo(cw, 450 * s);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.shadowBlur = 0;

  // Top goal (cyan)
  ctx.strokeStyle = "#00ffff";
  ctx.lineWidth = 18 * s;
  ctx.shadowBlur = 55 * s;
  ctx.shadowColor = "#00ffff";
  ctx.beginPath();
  ctx.arc(300 * s, 0, 140 * s, 0, Math.PI, false);
  ctx.stroke();

  // Bottom goal (magenta)
  ctx.strokeStyle = "#ff00ff";
  ctx.shadowColor = "#ff00ff";
  ctx.beginPath();
  ctx.arc(300 * s, ch, 140 * s, Math.PI, 2 * Math.PI, false);
  ctx.stroke();
  ctx.shadowBlur = 0;

  // Top paddle glow + fill
  glow((state.p1 + 60) * s, 102 * s, 75 * s, "0,255,120", 45 * s);
  ctx.fillStyle = "#00ff88";
  ctx.fillRect(state.p1 * s, 90 * s, 120 * s, 24 * s);

  // Bottom paddle
  glow((state.p2 + 60) * s, 792 * s, 75 * s, "255,40,160", 45 * s);
  ctx.fillStyle = "#ff4488";
  ctx.fillRect(state.p2 * s, 780 * s, 120 * s, 24 * s);

  // Ball mega glow + fill
  glow(state.bx * s, state.by * s, 65 * s, "255,255,255", 55 * s);
  ctx.fillStyle = "#ffffff";
  ctx.shadowBlur = 25 * s;
  ctx.shadowColor = "#ffffff";
  ctx.beginPath();
  ctx.arc(state.bx * s, state.by * s, 20 * s, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur = 0;

  // Update score
  score.textContent = state.s1 + " – " + state.s2;
}

socket.on("state", draw);

window.addEventListener("load", resizeCanvas);
window.addEventListener("resize", resizeCanvas);
window.addEventListener("orientationchange", () => setTimeout(resizeCanvas, 200));
resizeCanvas();

// === USER CONTROL FOR PINK PADDLE ===
canvas.addEventListener("mousemove", function(e) {
  if (!userCanMove) return;
  const rect = canvas.getBoundingClientRect();
  const x = (e.clientX - rect.left) / scale - 60;
  socket.emit("user_paddle", Math.max(50, Math.min(430, x)));
});
canvas.addEventListener("touchmove", function(e) {
  if (!userCanMove) return;
  const rect = canvas.getBoundingClientRect();
  const x = (e.touches[0].clientX - rect.left) / scale - 60;
  socket.emit("user_paddle", Math.max(50, Math.min(430, x)));
});

// Listen for when the backend disables/enables user control
socket.on("userCanMove", function(val) {
  userCanMove = val;
});
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return HTML

# ==================== REALISTIC PHYSICS LOOP ====================
def game_loop():
    global p1_x, user_paddle_x, bx, by, bvx, bvy, score_top, score_bottom, reset_ball, userCanMove, ball_touched_by_user
    clock = 0
    while True:
        clock += 1
        # === SMART AI (predictive-ish follow) ===
        target1 = bx - PADDLE_W // 2 + (bvy * 0.5 if clock % 120 < 60 else 0)
        p1_x += (target1 - p1_x) * 0.36
        p1_x = max(50, min(W - PADDLE_W - 50, p1_x))
        # Pink paddle is user controlled
        p2_x = user_paddle_x
        p2_x = max(50, min(W - PADDLE_W - 50, p2_x))
        user_paddle_x = p2_x
        if reset_ball:
            bx, by = W // 2, H // 2
            direction = random.choice([-1, 1])
            bvx = direction * random.uniform(6, 8)
            bvy = random.uniform(-1.5, 1.5)
            reset_ball = False
            reset_user_control()
        # === MOVE BALL ===
        bx += bvx
        by += bvy
        # === SIDE WALLS (anti-tunnel) ===
        if bx < BALL_R:
            bx = BALL_R * 2 - bx
            bvx = -bvx * 0.93
        elif bx > W - BALL_R:
            bx = (W - BALL_R) * 2 - bx
            bvx = -bvx * 0.93
        # === TOP PADDLE COLLISION (realistic angle) ===
        pcx1 = p1_x + PADDLE_W / 2
        if (bvy < 0 and by < PADDLE_CENTER_Y_TOP + BALL_R + 8 and
            by > PADDLE_CENTER_Y_TOP - BALL_R - 8 and
            bx > p1_x - BALL_R and bx < p1_x + PADDLE_W + BALL_R):
            rel_pos = (bx - pcx1) / (PADDLE_W / 2)
            bvy = -bvy * 1.12
            bvx = bvx * 0.65 + rel_pos * 9.5
            by = PADDLE_CENTER_Y_TOP - BALL_R - 4
        # === BOTTOM PADDLE ===
        pcx2 = user_paddle_x + PADDLE_W / 2
        if (bvy > 0 and by > PADDLE_CENTER_Y_BOTTOM - BALL_R - 8 and
            by < PADDLE_CENTER_Y_BOTTOM + BALL_R + 8 and
            bx > user_paddle_x - BALL_R and bx < user_paddle_x + PADDLE_W + BALL_R):
            if userCanMove:
                ball_touched_by_user = True
                userCanMove = False
                socketio.emit("userCanMove", False)
            rel_pos = (bx - pcx2) / (PADDLE_W / 2)
            bvy = -bvy * 1.12
            bvx = bvx * 0.65 + rel_pos * 9.5
            by = PADDLE_CENTER_Y_BOTTOM + BALL_R + 4
        # === ENFORCE RETURN TO CENTER ===
        if ball_touched_by_user and not userCanMove:
            # Only re-enable user control when ball returns to center
            if abs(by - H // 2) < 30 and abs(bx - W // 2) < 30:
                reset_user_control()
        # === TOP WALL / GOAL ===
        if by <= BALL_R:
            dist = math.hypot(bx - GOAL_CENTER_X, by)
            if dist <= GOAL_R:
                score_bottom += 1
                reset_ball = True
            else:
                by = BALL_R * 2 - by
                bvy = -bvy * 0.90
        # === BOTTOM WALL / GOAL ===
        if by >= H - BALL_R:
            dist = math.hypot(bx - GOAL_CENTER_X, by - H)
            if dist <= GOAL_R:
                score_top += 1
                reset_ball = True
            else:
                by = 2 * (H - BALL_R) - by
                bvy = -bvy * 0.90

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

    print("🌟 NEON AIR HOCKEY LAUNCHED (Responsive + Realistic Physics)!")
    print("Fits any screen | Proper bounces/angles | Python AI controls all")
    print("Edit game_loop() for OpenCV/RL control!")

    socketio.run(app, host="127.0.0.1", port=5000, debug=False)