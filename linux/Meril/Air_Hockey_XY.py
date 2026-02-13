# this is air hockey game using robot as paddle controlled by admittance control in X and Y axis
# NOTE: in this code the X and Y limits are not set so the robot can move freely in the XY plane
# so take care of the fact that the robot can move in any plane 
# +Y is forward -Y is backward 
# +X is right -X is left

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math
import threading
import webbrowser
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit
import json

app = Flask(__name__)
app.config['SECRET_KEY'] = 'airhockey_secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

robot = None
running = True
fixed_tcp_ref = None
game_active = False
robot_position = {'x': 0, 'y': 0}

FORCE_TO_MOTION_SCALE = 6.0
M = [1.6, 1.6, 1.4, 1.8, 1.8, 1.8]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]
IK_TO_SERVO_RATIO = 2
IK_UPDATE_RATE = 0.0025
SERVO_UPDATE_RATE = 0.008
FORCE_THRESHOLD = 0.8
FORCE_FILTER_ALPHA = 0.28
force_thresholds = [2.0, 2.0, 2.5, 1.0, 1.0, 1.0]
joint_velocity = [0.0] * 6
desired_joint_pos = [0.0] * 6
filtered_desired_joints = None
filtered_fx_world = 0.0
filtered_fy_world = 0.0
baseline_forces = [0.0] * 6
MAX_JOINT_VELOCITY = 40.0

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Robot Air Hockey</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            color: white;
        }
        .container { text-align: center; padding: 20px; }
        h1 { font-size: 3em; margin-bottom: 10px; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); }
        .status {
            font-size: 1.2em;
            margin-bottom: 20px;
            padding: 10px;
            background: rgba(255,255,255,0.1);
            border-radius: 10px;
            backdrop-filter: blur(10px);
        }
        #gameCanvas {
            border: 4px solid white;
            border-radius: 15px;
            box-shadow: 0 10px 40px rgba(0,0,0,0.4);
            background: linear-gradient(180deg, #0f172a 0%, #1e293b 50%, #0f172a 100%);
        }
        .controls {
            margin-top: 20px;
            display: flex;
            gap: 15px;
            justify-content: center;
            flex-wrap: wrap;
        }
        button {
            padding: 15px 30px;
            font-size: 1.1em;
            border: none;
            border-radius: 10px;
            cursor: pointer;
            transition: all 0.3s;
            font-weight: bold;
            text-transform: uppercase;
        }
        .start-btn { background: #10b981; color: white; }
        .start-btn:hover { background: #059669; transform: translateY(-2px); }
        .stop-btn { background: #ef4444; color: white; }
        .stop-btn:hover { background: #dc2626; transform: translateY(-2px); }
        .reset-btn { background: #f59e0b; color: white; }
        .reset-btn:hover { background: #d97706; transform: translateY(-2px); }
        .launch-btn { background: #8b5cf6; color: white; }
        .launch-btn:hover { background: #7c3aed; transform: translateY(-2px); }
        button:disabled { opacity: 0.5; cursor: not-allowed; transform: none !important; }
        .score {
            display: flex;
            justify-content: space-around;
            font-size: 2.5em;
            margin-top: 20px;
            font-weight: bold;
        }
        .score-label { font-size: 0.5em; display: block; margin-bottom: 5px; }
        .info { margin-top: 15px; font-size: 0.9em; opacity: 0.8; }
        .settings {
            margin-top: 15px;
            display: flex;
            gap: 20px;
            justify-content: center;
            flex-wrap: wrap;
            align-items: center;
        }
        .setting-group {
            background: rgba(255,255,255,0.1);
            padding: 10px 20px;
            border-radius: 8px;
            backdrop-filter: blur(10px);
        }
        .setting-group label {
            display: block;
            font-size: 0.9em;
            margin-bottom: 5px;
            opacity: 0.9;
        }
        .setting-group select,
        .setting-group input[type="range"] {
            padding: 8px 15px;
            font-size: 1em;
            border-radius: 5px;
            border: none;
            background: rgba(255,255,255,0.2);
            color: white;
            cursor: pointer;
            min-width: 150px;
        }
        .setting-group input[type="range"] { width: 200px; }
        .speed-value {
            display: inline-block;
            margin-left: 10px;
            font-weight: bold;
            font-size: 1.1em;
            min-width: 40px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🏒 ROBOT AIR HOCKEY 🏒</h1>
        <div class="status">
            <div id="connection">⚡ Connecting to robot...</div>
            <div id="robotPos">Robot: X=0.0mm Y=0.0mm</div>
            <div id="forces">Forces: Fx=0.0N Fy=0.0N</div>
        </div>
        
        <canvas id="gameCanvas" width="900" height="1100"></canvas>
        
        <div class="score">
            <div>
                <span class="score-label">AI COMPUTER</span>
                <span id="aiScore">0</span>
            </div>
            <div>
                <span class="score-label">YOU (ROBOT)</span>
                <span id="playerScore">0</span>
            </div>
        </div>
        
        <div class="settings">
            <div class="setting-group">
                <label>AI Difficulty:</label>
                <select id="difficulty" onchange="changeDifficulty()">
                    <option value="easy">Easy</option>
                    <option value="medium" selected>Medium</option>
                    <option value="hard">Hard</option>
                    <option value="expert">Expert</option>
                </select>
            </div>
            <div class="setting-group">
                <label>Puck Speed:</label>
                <input type="range" id="puckSpeed" min="1" max="5" step="0.5" value="2" oninput="updateSpeedDisplay()">
                <span class="speed-value" id="speedValue">2.0x</span>
            </div>
        </div>
        
        <div class="controls">
            <button class="start-btn" onclick="startGame()">Start Game</button>
            <button class="launch-btn" onclick="launchPuck()">Launch Puck</button>
            <button class="stop-btn" onclick="stopGame()">Stop Game</button>
            <button class="reset-btn" onclick="resetGame()">Reset Score</button>
        </div>
        
        <div class="info">
            Robot Controls: +X=right (→) | -X=left (←) | +Y=forward (↑) | -Y=backward (↓)<br>
            Workspace: X: -1000mm to -300mm (700mm) | Y: -200mm to +100mm (300mm)<br>
            First to 7 wins! • You are at the BOTTOM
        </div>
    </div>

    <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
    <script>
        const canvas = document.getElementById('gameCanvas');
        const ctx = canvas.getContext('2d');
        const socket = io();
        
        let gameRunning = false;
        let playerScore = 0;
        let aiScore = 0;
        let puckActive = false;
        let aiDifficulty = 'medium';
        let puckSpeedMultiplier = 2.0;
        
        const aiSettings = {
            easy: { speed: 2, reactionDelay: 15, accuracy: 0.7, maxSpeed: 3 },
            medium: { speed: 3.5, reactionDelay: 8, accuracy: 0.85, maxSpeed: 5 },
            hard: { speed: 5, reactionDelay: 4, accuracy: 0.95, maxSpeed: 7 },
            expert: { speed: 7, reactionDelay: 2, accuracy: 0.98, maxSpeed: 9 }
        };
        
        let aiReactionCounter = 0;
        
        const puck = {
            x: canvas.width / 2,
            y: canvas.height / 2,
            radius: 12,
            vx: 0,
            vy: 0,
            speed: 1.5,
            lastCollisionTime: 0
        };
        
        const aiPaddle = {
            x: canvas.width / 2,
            y: 120,
            radius: 35,
            vx: 0,
            vy: 0,
            targetX: canvas.width / 2,
            targetY: 120
        };
        
        const playerPaddle = {
            x: canvas.width / 2,
            y: canvas.height - 120,
            radius: 35,
            vx: 0,
            vy: 0,
            prevX: canvas.width / 2,
            prevY: canvas.height - 120
        };
        
        const ROBOT_LIMITS = {
            xMin: -1000,
            xMax: -300,
            yMin: -200,
            yMax: 100
        };
        
        const ROBOT_X_RANGE = ROBOT_LIMITS.xMax - ROBOT_LIMITS.xMin;
        const ROBOT_Y_RANGE = ROBOT_LIMITS.yMax - ROBOT_LIMITS.yMin;
        
        function robotToCanvas(robotX, robotY) {
            robotX = Math.max(ROBOT_LIMITS.xMin, Math.min(ROBOT_LIMITS.xMax, robotX));
            robotY = Math.max(ROBOT_LIMITS.yMin, Math.min(ROBOT_LIMITS.yMax, robotY));
            
            const normalizedX = (robotX - ROBOT_LIMITS.xMin) / ROBOT_X_RANGE;
            const x = normalizedX * canvas.width;
            
            const normalizedY = (robotY - ROBOT_LIMITS.yMin) / ROBOT_Y_RANGE;
            const y = canvas.height - (normalizedY * (canvas.height / 2));
            
            return { 
                x: Math.max(playerPaddle.radius, Math.min(canvas.width - playerPaddle.radius, x)), 
                y: Math.max(canvas.height / 2 + 50, Math.min(canvas.height - playerPaddle.radius - 10, y))
            };
        }
        
        socket.on('connect', () => {
            document.getElementById('connection').innerHTML = '✅ Connected to robot';
        });
        
        socket.on('robot_position', (data) => {
            const pos = robotToCanvas(data.x, data.y);
            
            playerPaddle.prevX = playerPaddle.x;
            playerPaddle.prevY = playerPaddle.y;
            
            playerPaddle.x = pos.x;
            playerPaddle.y = pos.y;
            
            playerPaddle.vx = playerPaddle.x - playerPaddle.prevX;
            playerPaddle.vy = playerPaddle.y - playerPaddle.prevY;
            
            document.getElementById('robotPos').innerHTML = 
                `Robot: X=${data.x.toFixed(1)}mm Y=${data.y.toFixed(1)}mm`;
        });
        
        socket.on('robot_forces', (data) => {
            document.getElementById('forces').innerHTML = 
                `Forces: Fx=${data.fx.toFixed(1)}N Fy=${data.fy.toFixed(1)}N`;
        });
        
        socket.on('game_state', (data) => {
            gameRunning = data.active;
        });
        
        function changeDifficulty() {
            aiDifficulty = document.getElementById('difficulty').value;
        }
        
        function updateSpeedDisplay() {
            puckSpeedMultiplier = parseFloat(document.getElementById('puckSpeed').value);
            document.getElementById('speedValue').textContent = puckSpeedMultiplier.toFixed(1) + 'x';
        }
        
        function startGame() {
            socket.emit('start_game');
            gameRunning = true;
        }
        
        function launchPuck() {
            if (!gameRunning) {
                alert('Please start the game first!');
                return;
            }
            puckActive = true;
            puck.x = canvas.width / 2;
            puck.y = canvas.height / 2;
            const angle = (Math.random() - 0.5) * Math.PI / 3;
            const direction = Math.random() > 0.5 ? 1 : -1;
            puck.vx = Math.sin(angle) * puck.speed * 1.5 * puckSpeedMultiplier;
            puck.vy = direction * Math.cos(angle) * puck.speed * 2 * puckSpeedMultiplier;
        }
        
        function stopGame() {
            socket.emit('stop_game');
            gameRunning = false;
            puckActive = false;
            puck.vx = 0;
            puck.vy = 0;
            puck.x = canvas.width / 2;
            puck.y = canvas.height / 2;
        }
        
        function resetGame() {
            playerScore = 0;
            aiScore = 0;
            document.getElementById('playerScore').textContent = playerScore;
            document.getElementById('aiScore').textContent = aiScore;
            stopGame();
        }
        
        function updateAI() {
            if (!gameRunning || !puckActive) return;
            
            const settings = aiSettings[aiDifficulty];
            aiReactionCounter++;
            
            if (aiReactionCounter >= settings.reactionDelay) {
                aiReactionCounter = 0;
                
                let predictX = puck.x;
                let predictY = aiPaddle.y;
                
                if (puck.vy < 0) {
                    const timeToReach = (aiPaddle.y - puck.y) / Math.abs(puck.vy);
                    predictX = puck.x + puck.vx * timeToReach * settings.accuracy;
                    predictX += (Math.random() - 0.5) * 80 * (1 - settings.accuracy);
                    predictY = 120;
                } else {
                    predictX = canvas.width / 2 + (Math.random() - 0.5) * 100;
                    if (puck.y < canvas.height / 2) {
                        predictY = Math.min(220, puck.y + 100);
                    } else {
                        predictY = 120;
                    }
                }
                
                aiPaddle.targetX = Math.max(aiPaddle.radius + 20, Math.min(canvas.width - aiPaddle.radius - 20, predictX));
                aiPaddle.targetY = Math.max(aiPaddle.radius + 20, Math.min(canvas.height / 2 - 50, predictY));
            }
            
            const dx = aiPaddle.targetX - aiPaddle.x;
            const moveSpeed = settings.speed;
            
            if (Math.abs(dx) > moveSpeed) {
                aiPaddle.vx = Math.sign(dx) * moveSpeed;
            } else {
                aiPaddle.vx = dx;
            }
            
            aiPaddle.x += aiPaddle.vx;
            aiPaddle.x = Math.max(aiPaddle.radius + 20, Math.min(canvas.width - aiPaddle.radius - 20, aiPaddle.x));
            
            const dy = aiPaddle.targetY - aiPaddle.y;
            const moveSpeedY = settings.speed * 0.8;
            
            if (Math.abs(dy) > moveSpeedY) {
                aiPaddle.vy = Math.sign(dy) * moveSpeedY;
            } else {
                aiPaddle.vy = dy;
            }
            
            aiPaddle.y += aiPaddle.vy;
            aiPaddle.y = Math.max(aiPaddle.radius + 20, Math.min(canvas.height / 2 - 50, aiPaddle.y));
        }
        
        function checkPaddleCollision(paddle, isAI = false) {
            const dx = puck.x - paddle.x;
            const dy = puck.y - paddle.y;
            const dist = Math.sqrt(dx * dx + dy * dy);
            
            if (dist < puck.radius + paddle.radius) {
                const now = Date.now();
                if (now - puck.lastCollisionTime < 100) return false;
                puck.lastCollisionTime = now;
                
                const angle = Math.atan2(dy, dx);
                const speed = Math.sqrt(puck.vx * puck.vx + puck.vy * puck.vy);
                
                const paddleVelocity = isAI ? 
                    Math.sqrt(aiPaddle.vx * aiPaddle.vx + aiPaddle.vy * aiPaddle.vy) : 
                    Math.sqrt(playerPaddle.vx * playerPaddle.vx + playerPaddle.vy * playerPaddle.vy);
                
                let newSpeed = speed * 1.08;
                
                const forceMultiplier = isAI ? 0.4 : 0.6;
                const forceBoost = paddleVelocity * forceMultiplier;
                
                newSpeed = newSpeed + forceBoost;
                
                const minSpeed = puck.speed * puckSpeedMultiplier;
                const maxSpeed = puckSpeedMultiplier * 12;
                
                newSpeed = Math.max(minSpeed, Math.min(newSpeed, maxSpeed));
                
                puck.vx = Math.cos(angle) * newSpeed;
                puck.vy = Math.sin(angle) * newSpeed;
                
                const overlap = puck.radius + paddle.radius - dist + 2;
                puck.x += Math.cos(angle) * overlap;
                puck.y += Math.sin(angle) * overlap;
                
                return true;
            }
            return false;
        }
        
        function updateGame() {
            if (!gameRunning || !puckActive) return;
            
            updateAI();
            
            puck.x += puck.vx;
            puck.y += puck.vy;
            puck.vx *= 0.998;
            puck.vy *= 0.998;
            
            if (puck.x - puck.radius < 20) {
                puck.vx = Math.abs(puck.vx) * 0.9;
                puck.x = 20 + puck.radius;
            } else if (puck.x + puck.radius > canvas.width - 20) {
                puck.vx = -Math.abs(puck.vx) * 0.9;
                puck.x = canvas.width - 20 - puck.radius;
            }
            
            const goalWidth = canvas.width * 0.3;
            const goalLeft = (canvas.width - goalWidth) / 2;
            const goalRight = goalLeft + goalWidth;
            
            if (puck.y - puck.radius < 10) {
                if (puck.x > goalLeft && puck.x < goalRight) {
                    playerScore++;
                    document.getElementById('playerScore').textContent = playerScore;
                    puckActive = false;
                    puck.x = canvas.width / 2;
                    puck.y = canvas.height / 2;
                    puck.vx = 0;
                    puck.vy = 0;
                } else {
                    puck.vy = Math.abs(puck.vy) * 0.8;
                    puck.y = 10 + puck.radius;
                }
            } else if (puck.y + puck.radius > canvas.height - 10) {
                if (puck.x > goalLeft && puck.x < goalRight) {
                    aiScore++;
                    document.getElementById('aiScore').textContent = aiScore;
                    puckActive = false;
                    puck.x = canvas.width / 2;
                    puck.y = canvas.height / 2;
                    puck.vx = 0;
                    puck.vy = 0;
                } else {
                    puck.vy = -Math.abs(puck.vy) * 0.8;
                    puck.y = canvas.height - 10 - puck.radius;
                }
            }
            
            checkPaddleCollision(aiPaddle, true);
            checkPaddleCollision(playerPaddle, false);
        }
        
        function draw() {
            const gradient = ctx.createLinearGradient(0, 0, 0, canvas.height);
            gradient.addColorStop(0, '#1e3a5f');
            gradient.addColorStop(0.5, '#2d5a8c');
            gradient.addColorStop(1, '#1e3a5f');
            ctx.fillStyle = gradient;
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 20;
            ctx.strokeRect(10, 10, canvas.width - 20, canvas.height - 20);
            
            ctx.strokeStyle = 'rgba(255, 0, 0, 0.5)';
            ctx.lineWidth = 4;
            ctx.setLineDash([30, 20]);
            ctx.beginPath();
            ctx.moveTo(20, canvas.height / 2);
            ctx.lineTo(canvas.width - 20, canvas.height / 2);
            ctx.stroke();
            ctx.setLineDash([]);
            
            ctx.beginPath();
            ctx.arc(canvas.width / 2, canvas.height / 2, 80, 0, Math.PI * 2);
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
            ctx.lineWidth = 3;
            ctx.stroke();
            
            const goalWidth = canvas.width * 0.3;
            const goalLeft = (canvas.width - goalWidth) / 2;
            
            ctx.fillStyle = 'rgba(255, 0, 0, 0.3)';
            ctx.fillRect(goalLeft, 0, goalWidth, 15);
            ctx.strokeStyle = '#ff0000';
            ctx.lineWidth = 3;
            ctx.strokeRect(goalLeft, 0, goalWidth, 15);
            
            ctx.fillStyle = 'rgba(0, 255, 100, 0.3)';
            ctx.fillRect(goalLeft, canvas.height - 15, goalWidth, 15);
            ctx.strokeStyle = '#00ff64';
            ctx.lineWidth = 3;
            ctx.strokeRect(goalLeft, canvas.height - 15, goalWidth, 15);
            
            const aiGradient = ctx.createRadialGradient(aiPaddle.x, aiPaddle.y, 0, aiPaddle.x, aiPaddle.y, aiPaddle.radius);
            aiGradient.addColorStop(0, '#60a5fa');
            aiGradient.addColorStop(0.7, '#3b82f6');
            aiGradient.addColorStop(1, '#1e40af');
            ctx.fillStyle = aiGradient;
            ctx.beginPath();
            ctx.arc(aiPaddle.x, aiPaddle.y, aiPaddle.radius, 0, Math.PI * 2);
            ctx.fill();
            ctx.strokeStyle = '#93c5fd';
            ctx.lineWidth = 4;
            ctx.stroke();
            
            ctx.fillStyle = '#fff';
            ctx.font = 'bold 14px Arial';
            ctx.fillText('AI', aiPaddle.x - 10, aiPaddle.y + 5);
            
            const playerGradient = ctx.createRadialGradient(playerPaddle.x, playerPaddle.y, 0, playerPaddle.x, playerPaddle.y, playerPaddle.radius);
            playerGradient.addColorStop(0, '#fca5a5');
            playerGradient.addColorStop(0.7, '#ef4444');
            playerGradient.addColorStop(1, '#991b1b');
            ctx.fillStyle = playerGradient;
            ctx.beginPath();
            ctx.arc(playerPaddle.x, playerPaddle.y, playerPaddle.radius, 0, Math.PI * 2);
            ctx.fill();
            ctx.strokeStyle = '#fecaca';
            ctx.lineWidth = 4;
            ctx.stroke();
            
            ctx.fillStyle = '#fff';
            ctx.font = 'bold 12px Arial';
            ctx.fillText('YOU', playerPaddle.x - 15, playerPaddle.y + 5);
            
            if (puckActive) {
                const puckGlow = ctx.createRadialGradient(puck.x, puck.y, 0, puck.x, puck.y, puck.radius * 2);
                puckGlow.addColorStop(0, 'rgba(255, 255, 255, 0.8)');
                puckGlow.addColorStop(1, 'rgba(255, 255, 255, 0)');
                ctx.fillStyle = puckGlow;
                ctx.beginPath();
                ctx.arc(puck.x, puck.y, puck.radius * 2, 0, Math.PI * 2);
                ctx.fill();
                
                const puckGradient = ctx.createRadialGradient(puck.x - 3, puck.y - 3, 0, puck.x, puck.y, puck.radius);
                puckGradient.addColorStop(0, '#ffffff');
                puckGradient.addColorStop(0.5, '#e0e0e0');
                puckGradient.addColorStop(1, '#b0b0b0');
                ctx.fillStyle = puckGradient;
                ctx.beginPath();
                ctx.arc(puck.x, puck.y, puck.radius, 0, Math.PI * 2);
                ctx.fill();
                ctx.strokeStyle = '#ffffff';
                ctx.lineWidth = 2;
                ctx.stroke();
                
                const speed = Math.sqrt(puck.vx * puck.vx + puck.vy * puck.vy);
                if (speed > 1) {
                    for (let i = 1; i <= 3; i++) {
                        const trailX = puck.x - puck.vx * i * 3;
                        const trailY = puck.y - puck.vy * i * 3;
                        const alpha = 0.3 - i * 0.1;
                        ctx.fillStyle = `rgba(255, 255, 255, ${alpha})`;
                        ctx.beginPath();
                        ctx.arc(trailX, trailY, puck.radius * (1 - i * 0.2), 0, Math.PI * 2);
                        ctx.fill();
                    }
                }
            } else {
                ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
                ctx.beginPath();
                ctx.arc(puck.x, puck.y, puck.radius, 0, Math.PI * 2);
                ctx.fill();
                ctx.strokeStyle = 'rgba(255, 255, 255, 0.6)';
                ctx.lineWidth = 2;
                ctx.stroke();
            }
        }
        
        function gameLoop() {
            updateGame();
            draw();
            requestAnimationFrame(gameLoop);
        }
        
        gameLoop();
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@socketio.on('start_game')
def handle_start_game():
    global game_active
    game_active = True
    emit('game_state', {'active': True}, broadcast=True)
    print("Game started!")

@socketio.on('stop_game')
def handle_stop_game():
    global game_active
    game_active = False
    emit('game_state', {'active': False}, broadcast=True)
    print("Game stopped!")

def euler_to_rotation_matrix(rx, ry, rz):
    rx = math.radians(rx); ry = math.radians(ry); rz = math.radians(rz)
    c, s = math.cos, math.sin
    Rx = np.array([[1,0,0], [0,c(rx),-s(rx)], [0,s(rx),c(rx)]])
    Ry = np.array([[c(ry),0,s(ry)], [0,1,0], [-s(ry),0,c(ry)]])
    Rz = np.array([[c(rz),-s(rz),0], [s(rz),c(rz),0], [0,0,1]])
    return Rz @ Ry @ Rx

def transform_forces_to_world(ft_forces, orientation):
    R = euler_to_rotation_matrix(*orientation)
    tcp_force = np.array([ft_forces[0], ft_forces[1], -ft_forces[2]])
    world_force = R @ tcp_force
    return world_force[0], world_force[1]

def ema(new, old, alpha):
    return alpha * new + (1 - alpha) * old

def init_ft_sensor():
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT sensor ready")

def calibrate_baseline(samples=150):
    print("Calibrating baseline... (keep tool still)")
    forces = []
    for _ in range(samples):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            forces.append(ret[1][:6])
        time.sleep(0.01)
    if forces:
        global baseline_forces
        baseline_forces = np.mean(forces, axis=0).tolist()
        print(f"Baseline: {[f'{x:+.3f}' for x in baseline_forces]}")

def control_loop():
    global running, filtered_fx_world, filtered_fy_world
    global desired_joint_pos, joint_velocity, filtered_desired_joints, fixed_tcp_ref
    global robot_position

    print("\n" + "="*70)
    print(" AIR HOCKEY MODE - X+Y ADMITTANCE CONTROL")
    print(" Robot position controls the paddle in the web game!")
    print("="*70)

    err, tcp = robot.GetActualTCPPose()
    if err != 0: return
    fixed_tcp_ref = tcp.copy()

    if robot.ServoMoveStart() != 0: return

    j = robot.GetActualJointPosDegree(flag=0)
    if j[0] == 0:
        desired_joint_pos[:] = j[1][:6]
    filtered_desired_joints = desired_joint_pos[:]

    acc_joints = None
    ik_count = servo_count = 0

    try:
        while running:
            t0 = time.time()

            err, current_tcp = robot.GetActualTCPPose()
            if err != 0:
                time.sleep(IK_UPDATE_RATE); continue

            robot_position['x'] = current_tcp[0]
            robot_position['y'] = current_tcp[1]
            
            if servo_count % 5 == 0:
                socketio.emit('robot_position', robot_position)

            ft = robot.FT_GetForceTorqueRCS()
            if ft[0] != 0:
                time.sleep(IK_UPDATE_RATE); continue

            raw = ft[1][:6]
            compensated = [raw[i] - baseline_forces[i] for i in range(6)]

            for i in range(6):
                if abs(compensated[i]) < force_thresholds[i]:
                    compensated[i] = 0.0

            fx_world, fy_world = transform_forces_to_world(compensated, current_tcp[3:6])
            filtered_fx_world = ema(fx_world, filtered_fx_world, FORCE_FILTER_ALPHA)
            filtered_fy_world = ema(fy_world, filtered_fy_world, FORCE_FILTER_ALPHA)

            if servo_count % 10 == 0:
                socketio.emit('robot_forces', {'fx': filtered_fx_world, 'fy': filtered_fy_world})

            active_fx = filtered_fx_world if abs(filtered_fx_world) > FORCE_THRESHOLD else 0.0
            active_fy = filtered_fy_world if abs(filtered_fy_world) > FORCE_THRESHOLD else 0.0

            delta_x = active_fx * FORCE_TO_MOTION_SCALE
            delta_y = active_fy * FORCE_TO_MOTION_SCALE

            target_x = current_tcp[0] + delta_x
            target_y = current_tcp[1] + delta_y

            target_tcp = [
                target_x, target_y, fixed_tcp_ref[2],
                fixed_tcp_ref[3], fixed_tcp_ref[4], fixed_tcp_ref[5]
            ]

            ik = robot.GetInverseKin(0, target_tcp, -1)
            if ik[0] != 0:
                time.sleep(IK_UPDATE_RATE); continue

            tj = np.array(ik[1][:6])
            acc_joints = tj if acc_joints is None else acc_joints + tj
            ik_count += 1

            if ik_count >= IK_TO_SERVO_RATIO:
                avg_joints = (acc_joints / IK_TO_SERVO_RATIO).tolist()

                for j in range(6):
                    err_joint = avg_joints[j] - desired_joint_pos[j]
                    f = err_joint * 3.9
                    acc = (f - B[j] * joint_velocity[j]) / M[j]
                    joint_velocity[j] += acc * SERVO_UPDATE_RATE
                    joint_velocity[j] = np.clip(joint_velocity[j], -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
                    desired_joint_pos[j] += joint_velocity[j] * SERVO_UPDATE_RATE

                alpha = 0.32
                if filtered_desired_joints is None:
                    filtered_desired_joints = desired_joint_pos[:]
                else:
                    for j in range(6):
                        filtered_desired_joints[j] = (
                            alpha * desired_joint_pos[j] + (1 - alpha) * filtered_desired_joints[j]
                        )

                robot.ServoJ(filtered_desired_joints, [0]*6, 0, 0, SERVO_UPDATE_RATE, 0, 0)

                if servo_count % 50 == 0:
                    max_jv = max(abs(v) for v in joint_velocity)
                    print(
                        f"🏒 X={current_tcp[0]:7.1f}mm Y={current_tcp[1]:7.1f}mm | "
                        f"Fx={filtered_fx_world:+5.1f}N Fy={filtered_fy_world:+5.1f}N | "
                        f"MaxVel={max_jv:4.1f}°/s"
                    )

                acc_joints = None
                ik_count = 0
                servo_count += 1

            sleep_t = IK_UPDATE_RATE - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except Exception as e:
        print("Error:", e)
    finally:
        robot.ServoMoveEnd()

def shutdown(sig, frame):
    global running
    print("\nStopping robot and server...")
    running = False
    time.sleep(0.5)
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

if __name__ == "__main__":
    robot = Robot.RPC('192.168.58.2')
    print("Connected to Fairino Cobot")
    init_ft_sensor()
    calibrate_baseline()
    
    robot_thread = threading.Thread(target=control_loop, daemon=True)
    robot_thread.start()
    
    print("\n" + "="*70)
    print("🏒 AIR HOCKEY GAME SERVER STARTING")
    print("="*70)
    print("Opening browser automatically...")
    print("If browser doesn't open, go to: http://localhost:5000")
    print("Push the robot to control the paddle!")
    print("="*70 + "\n")
    
    def open_browser():
        time.sleep(1.5)
        webbrowser.open('http://localhost:5000')
    
    browser_thread = threading.Thread(target=open_browser, daemon=True)
    browser_thread.start()
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)