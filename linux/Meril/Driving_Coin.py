# ROBOT JOINT 6 → 3D FIRST-PERSON DRIVER (ULTRA SMOOTH VERSION WITH COINS!)

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
    1: (-90.0, 85.0), 
    2: (-179.0, -35.0), 
    3: (-158.0, 158.0),
    4: (-264.0, 80.0), 
    5: (-170.0, 12.0), 
    6: (-50.0, 140.0),
}

JOINT_SPEED_LIMITS = {
    6: 60.0  # deg/s (cap to avoid joint 6 speed limit alarms)
}

# ====================== HTML CONTENT ======================

HTML_CONTENT = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Robot Joint 6 to HIGHWAY COIN COLLECTOR</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <style>
        body { margin: 0; overflow: hidden; font-family: Arial; background: #000; }
        #info { position: absolute; top: 75px; left: 20px; color: white; font-size: 18px; font-weight: 600; background: rgba(10,12,18,0.75); padding: 16px; border-radius: 16px; z-index: 100; border: 1px solid rgba(255,255,255,0.1); backdrop-filter: blur(6px); box-shadow: 0 10px 30px rgba(0,0,0,0.35); width: 320px; }
        .hud-debug { margin-top: 10px; padding-top: 8px; border-top: 1px solid rgba(255,255,255,0.08); font-size: 12px; color: #c5f9ff; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
        .hud-title { display: flex; align-items: center; gap: 8px; font-size: 20px; letter-spacing: 0.6px; }
        .hud-pill { font-size: 12px; padding: 4px 8px; border-radius: 999px; background: rgba(0,255,255,0.12); color: #9ff8ff; border: 1px solid rgba(0,255,255,0.25); }
        .hud-row { display: flex; justify-content: space-between; align-items: center; margin-top: 8px; font-size: 14px; color: #d6e6ff; }
        .hud-metric { font-size: 22px; font-weight: 700; color: #ffd700; }
        .hud-sub { font-size: 12px; color: #9fb4d1; }
        .hud-bar { height: 6px; background: rgba(255,255,255,0.12); border-radius: 999px; overflow: hidden; margin-top: 6px; }
        .hud-bar > span { display: block; height: 100%; background: linear-gradient(90deg, #00e5ff, #7dfd9a); width: 0%; transition: width 0.2s ease; }
        .hud-score { display: flex; justify-content: space-between; align-items: baseline; margin-top: 10px; }
        .hud-score .value { font-size: 28px; font-weight: 800; color: #7dfd9a; }
        #dashboard { position: absolute; bottom: 30px; left: 50%; transform: translateX(-50%); width: 550px; height: 130px; background: rgba(20,20,30,0.85); border: 3px solid #00ffff; border-radius: 15px; display: flex; justify-content: space-around; align-items: center; padding: 15px; }
        .gauge { text-align: center; color: #00ffff; }
        .gauge-value { font-size: 36px; font-weight: bold; color: #0f0; }
        .gauge-label { font-size: 14px; margin-top: 5px; }
        #gameover {
            position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);
            color: #ff4444; font-size: 56px; font-weight: bold; text-align: center;
            background: rgba(0,0,0,0.9); padding: 30px; border-radius: 20px; display: none;
            border: 4px solid #ff0000; z-index: 200;
        }
        #coin-popup {
            position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);
            color: #ff5fd7; font-size: 64px; font-weight: 900; text-align: center;
            pointer-events: none; display: none; animation: coinPop 0.6s ease-out;
            text-shadow: 0 0 20px #ff5fd7, 0 0 40px #ff5fd7;
        }
        @keyframes coinPop {
            0% { transform: translate(-50%, -50%) scale(0); opacity: 1; }
            50% { transform: translate(-50%, -70%) scale(1.2); }
            100% { transform: translate(-50%, -100%) scale(1); opacity: 0; }
        }
    </style>
</head>
<body>
    <div id="info">
        <div class="hud-title">🚗 HIGHWAY COIN COLLECTOR <span class="hud-pill">LIVE</span></div>
        <div class="hud-row">
            <div>Joint 6</div>
            <div><span id="joint6">0.0</span>°</div>
        </div>
        <div class="hud-row">
            <div>Speed</div>
            <div><span id="speed">0</span> km/h</div>
        </div>
        <div class="hud-bar"><span id="speed-bar"></span></div>
        <div class="hud-row" style="margin-top: 10px;">
            <div>Coins</div>
            <div class="hud-metric" id="coins">0</div>
        </div>
        <div class="hud-row">
            <div>Missed</div>
            <div id="passed-coins">0</div>
        </div>
        <div class="hud-row">
            <div>Accuracy</div>
            <div id="accuracy">0%</div>
        </div>
        <div class="hud-score">
            <div class="hud-sub">Score</div>
            <div class="value"><span id="score">0</span> pts</div>
        </div>
    </div>
        <div class="hud-debug">
            <div id="connection-status">Connecting...</div>
            <div>Force: <span id="force-mz">0.0</span> N·m</div>
            <div>Velocity: <span id="velocity">0.0</span> deg/s</div>
            <div style="color: #ffd54a;">-50° RIGHT | 140° LEFT</div>
        </div>
    </div>
    <div id="dashboard">
        <div class="gauge">
            <div class="gauge-value" id="angle-display">0 degrees</div>
            <div class="gauge-label">JOINT 6</div>
        </div>
        <div class="gauge" style="padding: 10px 20px; background: rgba(0,0,0,0.7); border: 2px solid #0f0; border-radius: 8px;">
            <div style="font-size: 16px; color: #888;">LANE</div>
            <div id="lane" style="margin-top: 5px; font-size: 28px;">CENTER</div>
        </div>
        <div class="gauge">
            <div class="gauge-value" style="color: #ffd700;" id="coin-display">0</div>
            <div class="gauge-label">COINS</div>
        </div>
        <div class="gauge">
            <div class="gauge-value" id="position-display">0.0</div>
            <div class="gauge-label">POSITION</div>
        </div>
    </div>
    <div id="gameover">
        <div>GAME OVER</div>
        <div style="font-size: 30px; margin-top: 20px;">Final Score: <span id="final-score">0</span>pts</div>
        <div style="font-size: 24px; margin-top: 10px; color: #ffd700;">
            Collected: <span id="final-coins">0</span> | Missed: <span id="final-passed">0</span>
        </div>
        <div style="font-size: 22px; margin-top: 8px; color: #0f0;">Accuracy: <span id="final-accuracy">0%</span></div>
        <div style="font-size: 20px; margin-top: 10px; color: #888;">Restarting in 3s...</div>
    </div>
    <div id="coin-popup">+5 POINTS!</div>
    <script>
        console.log('=== HIGHWAY COIN COLLECTOR ===');
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x9fc8ff);
        scene.fog = new THREE.Fog(0x9fc8ff, 60, 320);
        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        camera.position.set(0, 1.8, 3);
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.setPixelRatio(window.devicePixelRatio || 1);
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        renderer.outputEncoding = THREE.sRGBEncoding;
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1.1;
        renderer.physicallyCorrectLights = true;
        document.body.appendChild(renderer.domElement);
        scene.add(new THREE.AmbientLight(0xffffff, 0.25));
        const hemi = new THREE.HemisphereLight(0xbadfff, 0x3a4c2b, 0.7);
        scene.add(hemi);
        const sun = new THREE.DirectionalLight(0xffffff, 0.9);
        sun.position.set(100, 100, 50);
        sun.castShadow = true;
        sun.shadow.mapSize.set(2048, 2048);
        sun.shadow.camera.near = 10;
        sun.shadow.camera.far = 300;
        sun.shadow.camera.left = -80;
        sun.shadow.camera.right = 80;
        sun.shadow.camera.top = 80;
        sun.shadow.camera.bottom = -80;
        scene.add(sun);

        // Sky dome with subtle gradient
        const skyGeo = new THREE.SphereGeometry(500, 32, 16);
        const skyMat = new THREE.ShaderMaterial({
            side: THREE.BackSide,
            uniforms: {
                topColor: { value: new THREE.Color(0x9fc8ff) },
                bottomColor: { value: new THREE.Color(0xe6f2ff) },
                offset: { value: 33 },
                exponent: { value: 0.6 }
            },
            vertexShader: `
                varying vec3 vWorldPosition;
                void main() {
                    vec4 worldPosition = modelMatrix * vec4(position, 1.0);
                    vWorldPosition = worldPosition.xyz;
                    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
                }
            `,
            fragmentShader: `
                uniform vec3 topColor;
                uniform vec3 bottomColor;
                uniform float offset;
                uniform float exponent;
                varying vec3 vWorldPosition;
                void main() {
                    float h = normalize(vWorldPosition + offset).y;
                    float mixAmount = pow(max(h, 0.0), exponent);
                    gl_FragColor = vec4(mix(bottomColor, topColor, mixAmount), 1.0);
                }
            `
        });
        const sky = new THREE.Mesh(skyGeo, skyMat);
        scene.add(sky);

        const laneWidth = 4.0;
        const roadWidth = laneWidth * 3;
        const roadSegments = [];
        function createTree(x, z) {
            const tree = new THREE.Group();
            const trunk = new THREE.Mesh(
                new THREE.CylinderGeometry(0.25, 0.3, 1.6, 8),
                new THREE.MeshStandardMaterial({ color: 0x8b5a2b, roughness: 1.0, metalness: 0.0 })
            );
            trunk.position.y = 0.8;
            trunk.castShadow = true;
            tree.add(trunk);
            const crown = new THREE.Mesh(
                new THREE.SphereGeometry(0.9, 10, 10),
                new THREE.MeshStandardMaterial({ color: 0x2f7a2f, roughness: 1.0, metalness: 0.0 })
            );
            crown.position.y = 1.9;
            crown.castShadow = true;
            tree.add(crown);
            tree.position.set(x, 0, z);
            return tree;
        }

        function createHouse(x, z, color) {
            const house = new THREE.Group();
            const base = new THREE.Mesh(
                new THREE.BoxGeometry(2.2, 1.6, 2.4),
                new THREE.MeshStandardMaterial({ color, roughness: 0.8, metalness: 0.05 })
            );
            base.position.y = 0.8;
            base.castShadow = true;
            house.add(base);
            const roof = new THREE.Mesh(
                new THREE.ConeGeometry(1.8, 1.0, 4),
                new THREE.MeshStandardMaterial({ color: 0x6b2f2f, roughness: 0.7, metalness: 0.1 })
            );
            roof.position.y = 2.0;
            roof.rotation.y = Math.PI / 4;
            roof.castShadow = true;
            house.add(roof);
            house.position.set(x, 0, z);
            return house;
        }

        function createLake(x, z) {
            const lake = new THREE.Mesh(
                new THREE.CircleGeometry(6.0, 24),
                new THREE.MeshStandardMaterial({
                    color: 0x2b6cb0,
                    roughness: 0.2,
                    metalness: 0.1,
                    transparent: true,
                    opacity: 0.85
                })
            );
            lake.rotation.x = -Math.PI / 2;
            lake.position.set(x, -0.08, z);
            lake.receiveShadow = true;
            return lake;
        }
        function createAsphaltTexture() {
            const size = 512;
            const canvas = document.createElement('canvas');
            canvas.width = size;
            canvas.height = size;
            const ctx = canvas.getContext('2d');
            ctx.fillStyle = '#2a2a2a';
            ctx.fillRect(0, 0, size, size);
            const imageData = ctx.getImageData(0, 0, size, size);
            for (let i = 0; i < imageData.data.length; i += 4) {
                const v = 20 + Math.random() * 35;
                imageData.data[i] += v;
                imageData.data[i + 1] += v;
                imageData.data[i + 2] += v;
            }
            ctx.putImageData(imageData, 0, 0);
            ctx.strokeStyle = 'rgba(255,255,255,0.05)';
            for (let i = 0; i < 25; i++) {
                ctx.beginPath();
                ctx.moveTo(Math.random() * size, Math.random() * size);
                ctx.lineTo(Math.random() * size, Math.random() * size);
                ctx.stroke();
            }
            const texture = new THREE.CanvasTexture(canvas);
            texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
            texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
            return texture;
        }
        const asphaltTexture = createAsphaltTexture();
        asphaltTexture.repeat.set(1, 4);
        function createRoad(z) {
            const group = new THREE.Group();
            const road = new THREE.Mesh(
                new THREE.PlaneGeometry(roadWidth + 4, 20),
                new THREE.MeshStandardMaterial({
                    map: asphaltTexture,
                    color: 0x303030,
                    roughness: 0.95,
                    metalness: 0.05
                })
            );
            road.rotation.x = -Math.PI / 2;
            road.receiveShadow = true;
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
                    new THREE.MeshStandardMaterial({ color: 0x2f5e2f, roughness: 1.0, metalness: 0.0 })
                );
                grass.rotation.x = -Math.PI / 2;
                grass.position.set(x, -0.1, 0);
                group.add(grass);
            });

            // Roadside scenery
            const leftX = -roadWidth / 2 - 6.5;
            const rightX = roadWidth / 2 + 6.5;
            for (let i = 0; i < 2; i++) {
                const zOffset = -6 + i * 8;
                group.add(createTree(leftX + (Math.random() * 1.5 - 0.75), zOffset));
                group.add(createTree(rightX + (Math.random() * 1.5 - 0.75), zOffset));
            }
            const houseColors = [0xf2d7b6, 0xd9e8f5, 0xf7c9c9, 0xe2f0cb];
            group.add(createHouse(leftX - 4.0, -4 + Math.random() * 6, houseColors[Math.floor(Math.random() * houseColors.length)]));
            group.add(createHouse(rightX + 4.0, -4 + Math.random() * 6, houseColors[Math.floor(Math.random() * houseColors.length)]));
            group.add(createLake(leftX - 10.0, -6 + Math.random() * 8));
            group.add(createLake(rightX + 10.0, -6 + Math.random() * 8));
            group.position.z = z;
            return group;
        }
        for (let i = 0; i < 40; i++) {
            const segment = createRoad(-i * 20);
            roadSegments.push(segment);
            scene.add(segment);
        }

        const playerCar = new THREE.Group();
        const playerBody = new THREE.Mesh(
            new THREE.BoxGeometry(1.9, 0.6, 4.2),
            new THREE.MeshPhysicalMaterial({
                color: 0xdd0000,
                roughness: 0.25,
                metalness: 0.6,
                clearcoat: 0.6,
                clearcoatRoughness: 0.1
            })
        );
        playerBody.position.y = 0.4; playerBody.castShadow = true; playerCar.add(playerBody);
        const playerCabin = new THREE.Mesh(
            new THREE.BoxGeometry(1.7, 0.5, 2.0),
            new THREE.MeshPhysicalMaterial({
                color: 0xbb0000,
                roughness: 0.3,
                metalness: 0.5,
                clearcoat: 0.4,
                clearcoatRoughness: 0.15
            })
        );
        playerCabin.position.set(0, 0.85, -0.3); playerCabin.castShadow = true; playerCar.add(playerCabin);
        const playerHood = new THREE.Mesh(
            new THREE.BoxGeometry(1.8, 0.15, 1.5),
            new THREE.MeshPhysicalMaterial({
                color: 0xff1111,
                roughness: 0.22,
                metalness: 0.6,
                clearcoat: 0.7,
                clearcoatRoughness: 0.08
            })
        );
        playerHood.position.set(0, 0.625, -1.35); playerCar.add(playerHood);
        const windshield = new THREE.Mesh(
            new THREE.BoxGeometry(1.6, 0.4, 0.8),
            new THREE.MeshPhysicalMaterial({
                color: 0x111144,
                roughness: 0.05,
                metalness: 0.0,
                transparent: true,
                opacity: 0.55,
                transmission: 0.3
            })
        );
        windshield.position.set(0, 0.95, -0.7); windshield.rotation.x = -0.2; playerCar.add(windshield);
        const rearWindow = new THREE.Mesh(
            new THREE.BoxGeometry(1.6, 0.35, 0.6),
            new THREE.MeshPhysicalMaterial({
                color: 0x111144,
                roughness: 0.05,
                metalness: 0.0,
                transparent: true,
                opacity: 0.55,
                transmission: 0.3
            })
        );
        rearWindow.position.set(0, 0.9, 0.5); rearWindow.rotation.x = 0.15; playerCar.add(rearWindow);
        [-0.6, 0.6].forEach(x => {
            const light = new THREE.Mesh(
                new THREE.CircleGeometry(0.15, 16),
                new THREE.MeshStandardMaterial({ color: 0xffffaa, emissive: 0xffffaa, emissiveIntensity: 0.8 })
            );
            light.position.set(x, 0.5, -2.05); light.rotation.y = Math.PI; playerCar.add(light);
        });
        [-0.6, 0.6].forEach(x => {
            const light = new THREE.Mesh(
                new THREE.CircleGeometry(0.12, 16),
                new THREE.MeshStandardMaterial({ color: 0xff0000, emissive: 0xff0000, emissiveIntensity: 0.7 })
            );
            light.position.set(x, 0.45, 2.05); playerCar.add(light);
        });
        const spoiler = new THREE.Mesh(
            new THREE.BoxGeometry(1.8, 0.1, 0.4),
            new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.6, metalness: 0.2 })
        );
        spoiler.position.set(0, 1.0, 1.8); playerCar.add(spoiler);
        const playerWheelGeo = new THREE.CylinderGeometry(0.35, 0.35, 0.3, 16);
        const playerWheelMat = new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.9, metalness: 0.1 });
        const playerWheels = [];
        const playerPositions = [[-0.85, 1.2], [0.85, 1.2], [-0.85, -1.2], [0.85, -1.2]];
        playerPositions.forEach(pos => {
            const wheelGroup = new THREE.Group();
            const wheel = new THREE.Mesh(playerWheelGeo, playerWheelMat);
            wheel.rotation.x = Math.PI / 2; wheel.castShadow = true; wheelGroup.add(wheel);
            const rim = new THREE.Mesh(
                new THREE.CircleGeometry(0.2, 16),
                new THREE.MeshStandardMaterial({ color: 0x9a9a9a, roughness: 0.35, metalness: 0.8 })
            );
            rim.position.z = 0.16; rim.rotation.x = Math.PI / 2; wheelGroup.add(rim);
            wheelGroup.position.set(pos[0], 0.35, pos[1]);
            playerCar.add(wheelGroup); playerWheels.push(wheelGroup);
        });
        const playerFrontLeft = playerWheels[0];
        const playerFrontRight = playerWheels[1];
        playerCar.position.y = 0.1;
        scene.add(playerCar);

        const aiCars = [];
        const aiCarLength = 4.0;
        const aiCarWidth = 1.9;
        const collisionRadius = 2.2;
        const aiCarConfigs = [
            { bodyColor: 0x00aa00, cabinColor: 0x008800 },
            { bodyColor: 0x0088ff, cabinColor: 0x0066cc },
            { bodyColor: 0xffaa00, cabinColor: 0xdd8800 },
            { bodyColor: 0xff00ff, cabinColor: 0xcc00cc },
            { bodyColor: 0x00ffff, cabinColor: 0x00cccc },
            { bodyColor: 0xffff00, cabinColor: 0xcccc00 },
            { bodyColor: 0xff66cc, cabinColor: 0xdd44aa },
            { bodyColor: 0xffffff, cabinColor: 0xdddddd }
        ];
        function createAICar() {
            const config = aiCarConfigs[Math.floor(Math.random() * aiCarConfigs.length)];
            const aiCar = new THREE.Group();
            const aiBody = new THREE.Mesh(
                new THREE.BoxGeometry(aiCarWidth, 0.6, aiCarLength),
                new THREE.MeshPhysicalMaterial({
                    color: config.bodyColor,
                    roughness: 0.35,
                    metalness: 0.5,
                    clearcoat: 0.5,
                    clearcoatRoughness: 0.12
                })
            );
            aiBody.position.y = 0.4; aiBody.castShadow = true; aiCar.add(aiBody);
            const aiCabin = new THREE.Mesh(
                new THREE.BoxGeometry(1.6, 0.5, 2.2),
                new THREE.MeshPhysicalMaterial({
                    color: config.cabinColor,
                    roughness: 0.4,
                    metalness: 0.4,
                    clearcoat: 0.35,
                    clearcoatRoughness: 0.15
                })
            );
            aiCabin.position.y = 0.85; aiCabin.castShadow = true; aiCar.add(aiCabin);
            const frontWindow = new THREE.Mesh(
                new THREE.BoxGeometry(1.5, 0.35, 0.7),
                new THREE.MeshPhysicalMaterial({
                    color: 0x222244,
                    transparent: true,
                    opacity: 0.5,
                    roughness: 0.05,
                    metalness: 0.0,
                    transmission: 0.25
                })
            );
            frontWindow.position.set(0, 0.9, -0.8); frontWindow.rotation.x = -0.15; aiCar.add(frontWindow);
            [-0.55, 0.55].forEach(x => {
                const light = new THREE.Mesh(new THREE.CircleGeometry(0.12, 12), new THREE.MeshBasicMaterial({ color: 0xffffcc }));
                light.position.set(x, 0.45, -2.05); light.rotation.y = Math.PI; aiCar.add(light);
            });
            const wheelGeo = new THREE.CylinderGeometry(0.33, 0.33, 0.3, 12);
            const wheelMat = new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.9, metalness: 0.1 });
            [[-0.8, 1.2], [0.8, 1.2], [-0.8, -1.2], [0.8, -1.2]].forEach(pos => {
                const wheel = new THREE.Mesh(wheelGeo, wheelMat);
                wheel.rotation.x = Math.PI / 2;
                wheel.position.set(pos[0], 0.35, pos[1]);
                wheel.castShadow = true;
                aiCar.add(wheel);
            });
            aiCar.position.y = 0.1;
            const spawnZ = -80 - Math.random() * 60;
            aiCar.userData = { lane: 0, z: spawnZ };
            scene.add(aiCar);
            return aiCar;
        }

        const coins = [];
        const coinRadius = 0.6;
        const coinCollectRadius = 1.5;
        let lastCoinSpawnTime = 0;
        let lastCarSpawnTime = 0;
        const COIN_SPAWN_INTERVAL = 7000;
        const CAR_SPAWN_INTERVAL = 7000;
        let currentCoinLane = null;
        let lastCoinLaneIndex = -1;
        let lastCarLaneIndex = -1;
        const activeCoinLanes = new Set();
        const activeCarLanes = new Set();
        const SPEED_MULTIPLIER = 1.0;
        const BASE_SPEED_KMH = 60;
        const MAX_SPEED_KMH = 120;

        // ADDED: Coin stats
        let coinsCollected = 0;
        let coinsPassed = 0;
        let totalCoinsSpawned = 0;

        function createCoin(x, z) {
            const coinGroup = new THREE.Group();
            const coinGeo = new THREE.CylinderGeometry(coinRadius, coinRadius, 0.15, 20);
            const coinMat = new THREE.MeshLambertMaterial({ color: 0xffd700, emissive: 0xffaa00, emissiveIntensity: 0.3 });
            const coin = new THREE.Mesh(coinGeo, coinMat);
            coin.rotation.x = Math.PI / 2; coin.castShadow = true; coinGroup.add(coin);
            const innerCircle = new THREE.Mesh(new THREE.CircleGeometry(coinRadius * 0.6, 20), new THREE.MeshBasicMaterial({ color: 0xffee44 }));
            innerCircle.position.z = 0.08; coinGroup.add(innerCircle);
            const starShape = new THREE.Shape();
            const spikes = 5; const outerRadius = coinRadius * 0.4; const innerRadius = outerRadius * 0.5;
            for (let i = 0; i < spikes * 2; i++) {
                const radius = i % 2 === 0 ? outerRadius : innerRadius;
                const angle = (i * Math.PI) / spikes;
                const px = Math.cos(angle) * radius;
                const py = Math.sin(angle) * radius;
                if (i === 0) starShape.moveTo(px, py);
                else starShape.lineTo(px, py);
            }
            starShape.closePath();
            const starGeo = new THREE.ShapeGeometry(starShape);
            const star = new THREE.Mesh(starGeo, new THREE.MeshBasicMaterial({ color: 0xffaa00 }));
            star.position.z = 0.09; coinGroup.add(star);
            coinGroup.position.set(x, 0.8, z);
            coinGroup.userData = { z: z, collected: false, rotation: 0 };
            scene.add(coinGroup);
            return coinGroup;
        }

        function spawnCoinBatch() {
            const now = Date.now();
            if (now - lastCoinSpawnTime < COIN_SPAWN_INTERVAL) return;
            const lanes = [-laneWidth, 0, laneWidth];
            const carsAhead = aiCars.filter(car => car.userData.z > -100 && car.userData.z < 50);
            const lanesWithCars = new Set(carsAhead.map(car => car.position.x));
            const availableLanes = lanes.filter(lane => !lanesWithCars.has(lane));
            if (availableLanes.length === 0) return;
            let candidateLanes = availableLanes.filter((lane, i) => {
                const idx = lanes.indexOf(lane);
                return lastCoinLaneIndex === -1 || idx !== lastCoinLaneIndex;
            });
            if (candidateLanes.length === 0) candidateLanes = availableLanes;
            const selectedLaneIndex = lanes.indexOf(candidateLanes[Math.floor(Math.random() * candidateLanes.length)]);
            const selectedLane = lanes[selectedLaneIndex];
            lastCoinLaneIndex = selectedLaneIndex;
            currentCoinLane = selectedLane;
            activeCoinLanes.add(selectedLane);
            lastCoinSpawnTime = now;
            const baseZ = -80;
            const spacing = 8;
            for (let i = 0; i < 4; i++) {
                const z = baseZ - (i * spacing);
                const coin = createCoin(selectedLane, z);
                coin.userData.z = z;
                coins.push(coin);
                totalCoinsSpawned++;
            }
            setTimeout(() => {
                activeCoinLanes.delete(selectedLane);
                if (currentCoinLane === selectedLane) currentCoinLane = null;
            }, 15000);
        }

        function spawnCar() {
            const now = Date.now();
            if (now - lastCarSpawnTime < CAR_SPAWN_INTERVAL) return;
            const carsAhead = aiCars.filter(car => car.userData.z < 30);
            if (carsAhead.length > 0) return;
            lastCarSpawnTime = now;
            const lanes = [-laneWidth, 0, laneWidth];
            const lanesWithCoins = new Set();
            for (let coin of coins) {
                if (!coin.userData.collected && coin.userData.z < 60) {
                    const closestLane = lanes.reduce((a, b) =>
                        Math.abs(b - coin.position.x) < Math.abs(a - coin.position.x) ? b : a
                    );
                    lanesWithCoins.add(closestLane);
                }
            }
            let availableLanes = lanes.filter(lane => !lanesWithCoins.has(lane));
            if (availableLanes.length === 0) return;
            let candidateLanes = availableLanes.filter((lane, i) => {
                const idx = lanes.indexOf(lane);
                return lastCarLaneIndex === -1 || idx !== lastCarLaneIndex;
            });
            if (candidateLanes.length === 0) candidateLanes = availableLanes;
            const selectedLane = candidateLanes[Math.floor(Math.random() * candidateLanes.length)];
            const laneIndex = lanes.indexOf(selectedLane);
            lastCarLaneIndex = laneIndex;
            activeCarLanes.add(selectedLane);
            const newCar = createAICar();
            newCar.userData.lane = laneIndex - 1;
            newCar.userData.laneX = selectedLane;
            newCar.position.x = selectedLane;
            newCar.position.z = -80 - Math.random() * 40;
            newCar.userData.z = newCar.position.z;
            aiCars.push(newCar);
            scene.add(newCar);
            setTimeout(() => activeCarLanes.delete(selectedLane), 12000);
        }

        let joint6Angle = 0;
        let playerX = 0;
        let score = 0;
        let gameOver = false;
        let connected = false;
        let gameStartTime = Date.now();

        function getTargetX(angle) {
            const minAngle = -50.0;
            const maxAngle = 140.0;
            const range = maxAngle - minAngle;
            const normalized = (angle - minAngle) / range;
            return THREE.MathUtils.lerp(4.0, -4.0, normalized);
        }

        function showCoinPopup() {
            const popup = document.getElementById('coin-popup');
            popup.style.display = 'block';
            setTimeout(() => popup.style.display = 'none', 600);
        }

        function checkCoinCollection() {
            if (gameOver) return;
            for (let i = coins.length - 1; i >= 0; i--) {
                const coin = coins[i];
                if (coin.userData.collected) continue;
                const dx = playerX - coin.position.x;
                const dz = 0 - coin.userData.z;
                const distance = Math.sqrt(dx*dx + dz*dz);
                if (distance < coinCollectRadius) {
                    coin.userData.collected = true;
                    coinsCollected++;
                    score += 5;
                    showCoinPopup();
                    const startY = coin.position.y;
                    const duration = 300;
                    const startTime = Date.now();
                    function anim() {
                        const p = Math.min((Date.now() - startTime) / duration, 1);
                        coin.position.y = startY + p * 2;
                        coin.scale.set(1 + p * 0.5, 1 + p * 0.5, 1 + p * 0.5);
                        coin.rotation.y += 0.3;
                        if (p < 1) requestAnimationFrame(anim);
                        else { scene.remove(coin); coins.splice(i, 1); }
                    }
                    anim();
                }
            }
        }

        function checkCollision() {
            if (gameOver) return false;
            for (let aiCar of aiCars) {
                const dx = playerX - aiCar.position.x;
                const dz = 0 - aiCar.userData.z;
                if (Math.sqrt(dx*dx + dz*dz) < collisionRadius) {
                    gameOver = true;
                    document.getElementById('final-score').textContent = score;
                    document.getElementById('final-coins').textContent = coinsCollected;
                    document.getElementById('final-passed').textContent = coinsPassed;
                    document.getElementById('final-accuracy').textContent = 
                        totalCoinsSpawned > 0 ? Math.round((coinsCollected / totalCoinsSpawned) * 100) + '%' : '0%';
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
                        document.getElementById('connection-status').textContent = 'Robot Connected';
                    }
                })
                .catch(() => {
                    document.getElementById('connection-status').textContent = 'Disconnected';
                });
        }
        setInterval(updateRobotData, 50);

        function animate() {
            requestAnimationFrame(animate);
            if (gameOver) { renderer.render(scene, camera); return; }

            const targetX = getTargetX(joint6Angle);
            playerX += (targetX - playerX) * 0.08;
            playerCar.position.x = playerX;
            const wheelTurn = (joint6Angle + 50.0) / 190.0 * 0.7 - 0.35;
            playerFrontLeft.rotation.z = wheelTurn;
            playerFrontRight.rotation.z = wheelTurn;

            for (let i = aiCars.length - 1; i >= 0; i--) {
                const car = aiCars[i];
                car.userData.z += 0.208 * SPEED_MULTIPLIER;
                car.position.z = car.userData.z;
                if (car.userData.z > 20) { scene.remove(car); aiCars.splice(i, 1); }
            }

            for (let i = coins.length - 1; i >= 0; i--) {
                const coin = coins[i];
                if (coin.userData.collected) continue;
                coin.userData.z += 0.208 * SPEED_MULTIPLIER;
                coin.position.z = coin.userData.z;
                coin.userData.rotation += 0.05;
                coin.rotation.y = coin.userData.rotation;
                coin.position.y = 0.8 + Math.sin(coin.userData.rotation * 2) * 0.2;

                if (coin.userData.z > 20) {
                    if (!coin.userData.collected) coinsPassed++;
                    scene.remove(coin);
                    coins.splice(i, 1);
                }
            }

            spawnCar();
            spawnCoinBatch();
            checkCoinCollection();
            checkCollision();

            camera.position.x = playerX;
            camera.position.y = 1.8;
            camera.position.z = 3;
            camera.lookAt(playerX, 0.5, -10);

            roadSegments.forEach(seg => {
                seg.position.z += 0.208 * SPEED_MULTIPLIER;
                if (seg.position.z > 20) seg.position.z -= 800;
            });

            document.getElementById('score').textContent = score;
            document.getElementById('coins').textContent = coinsCollected;
            document.getElementById('passed-coins').textContent = coinsPassed;
            document.getElementById('accuracy').textContent = 
                totalCoinsSpawned > 0 ? Math.round((coinsCollected / totalCoinsSpawned) * 100) + '%' : '0%';
            document.getElementById('coin-display').textContent = coinsCollected;
            document.getElementById('joint6').textContent = joint6Angle.toFixed(1);
            document.getElementById('angle-display').textContent = joint6Angle.toFixed(0) + ' degrees';
            document.getElementById('position-display').textContent = playerX.toFixed(2);
            const speedValue = Math.round(BASE_SPEED_KMH * SPEED_MULTIPLIER);
            document.getElementById('speed').textContent = speedValue;
            const speedPercent = Math.min(100, Math.max(0, (speedValue / MAX_SPEED_KMH) * 100));
            document.getElementById('speed-bar').style.width = speedPercent.toFixed(0) + '%';

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
</html>"""

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
    print("🎮 TWIST WRIST TO STEER! COLLECT COINS! (ULTRA SMOOTH & EASY)")

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
        max_speed = JOINT_SPEED_LIMITS.get(6)
        if max_speed is not None:
            velocity[j6] = float(np.clip(velocity[j6], -max_speed, max_speed))
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
    print("🚗 ROBOT JOINT 6 → 3D COIN COLLECTOR (ULTRA SMOOTH)")
    print("=" * 60)
    
    threading.Thread(target=web_server_thread, daemon=True).start()
    threading.Thread(target=robot_control_thread, daemon=True).start()
    
    try:
        while running:
            time.sleep(1)
    except KeyboardInterrupt:
        shutdown(None, None)