"""
Perfectly Designed Interface
Best Working Web based Isometric testing interface for Muscle Testing
Robot Rehabilitation Control System
Combined Flask Web Server + Robot Controller
"""

from flask import Flask, jsonify, request, send_file
from flask_cors import CORS
import threading
import time
from datetime import datetime
import os
import webbrowser
import matplotlib.pyplot as plt
import io
import base64

# Import your robot library
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

app = Flask(__name__)
CORS(app)

# HTML Template embedded in Python
HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Rehabilitation Dashboard</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
            background: #0c2d29; 
            padding: 40px;
            overflow-y: auto;
            min-height: 100vh;
        }
        
        .container { max-width: 1400px; margin: 0 auto; }
        
        .header {
            background: rgba(255, 255, 255, 0.15);
            padding: 40px;
            text-align: center;
            margin-bottom: 30px;
            border-radius: 8px;
            backdrop-filter: blur(10px);
        }
        
        .header h1 {
            color: white;
            font-size: 28px;
            font-weight: bold;
            margin-bottom: 8px;
        }
        
        .header p { color: rgba(255, 255, 255, 0.9); font-size: 12px; }
        
        .card {
            background: rgba(255, 255, 255, 0.12);
            backdrop-filter: blur(10px);
            border-radius: 8px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
            margin-bottom: 25px;
            padding: 30px;
            border: 1px solid rgba(255, 255, 255, 0.18);
            transition: all 0.4s ease;
        }
        
        .card:hover {
            transform: translateY(-8px) scale(1.02);
            box-shadow: 0 12px 24px rgba(0, 0, 0, 0.25);
            background: rgba(255, 255, 255, 0.18);
            border: 1px solid rgba(255, 255, 255, 0.3);
        }
        
        .top-row {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 25px;
            margin-bottom: 25px;
        }
        
        @media (max-width: 968px) {
            .top-row {
                grid-template-columns: 1fr;
            }
        }
        
        .card-title {
            display: flex;
            align-items: center;
            margin-bottom: 20px;
        }
        
        .card-title .bullet {
            color: white;
            font-size: 24px;
            margin-right: 10px;
        }
        
        .card-title h2 {
            color: white;
            font-size: 22px;
            font-weight: bold;
        }
        
        .separator {
            height: 2px;
            background: rgba(255, 255, 255, 0.2);
            margin-bottom: 20px;
        }
        
        .status-box {
            background: rgba(255, 255, 255, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
            border-radius: 6px;
            padding: 15px 20px;
            margin-bottom: 20px;
            display: flex;
            align-items: center;
        }
        
        .status-indicator {
            font-size: 32px;
            margin-right: 15px;
        }
        
        .status-indicator.disconnected { color: #e74c3c; }
        .status-indicator.active { color: #2ecc71; }
        .status-indicator.connecting { color: #f39c12; }
        
        .status-text h3 {
            color: white;
            font-size: 14px;
            font-weight: bold;
            margin-bottom: 3px;
        }
        
        .status-text p { color: rgba(255, 255, 255, 0.8); font-size: 11px; }
        
        .button-row {
            display: flex;
            gap: 15px;
            margin-top: 10px;
        }
        
        .btn {
            padding: 15px 35px;
            border: none;
            border-radius: 6px;
            font-size: 13px;
            font-weight: bold;
            color: white;
            cursor: pointer;
            transition: all 0.3s;
        }
        
        .btn-primary { background: rgba(255, 255, 255, 0.25); }
        .btn-primary:hover { background: rgba(255, 255, 255, 0.35); }
        .btn-primary:disabled { background: rgba(255, 255, 255, 0.1); cursor: not-allowed; opacity: 0.5; }
        .btn-secondary { background: rgba(255, 255, 255, 0.25); }
        .btn-secondary:hover { background: rgba(255, 255, 255, 0.35); }
        .btn-danger { background: rgba(231, 76, 60, 0.8); }
        .btn-danger:hover { background: rgba(192, 57, 43, 0.9); }
        
        .form-row {
            display: flex;
            align-items: center;
            margin-bottom: 10px;
            gap: 20px;
        }
        
        .form-row label {
            color: white;
            font-weight: bold;
            font-size: 12px;
            min-width: 120px;
        }
        
        .form-row input, .form-row select {
            flex: 1;
            padding: 10px 12px;
            background: rgba(255, 255, 255, 0.15);
            border: 1px solid rgba(255, 255, 255, 0.25);
            border-radius: 4px;
            font-size: 11px;
            color: white;
        }
        
        .form-row input::placeholder {
            color: rgba(255, 255, 255, 0.6);
        }
        
        .form-row select option {
            background: #3a9d8f;
            color: white;
        }
        
        .exercise-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin-top: 20px;
        }

        # .exercise-grid {
        #     display: grid;
        #     grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
        #     gap: 15px;
        #     margin-top: 20px;
        # }
        
        .exercise-btn {
            padding: 25px 20px;
            border: none;
            border-radius: 6px;
            color: white;
            font-weight: bold;
            font-size: 12px;
            cursor: pointer;
            transition: all 0.3s;
            text-align: center;
        }
        
        .exercise-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 8px rgba(0,0,0,0.3);
        }
        
        .upper-btn { background: #3498db; }
        .upper-btn:hover { background: #2980b9; }
        
        .lower-btn { background: #e67e22; }
        .lower-btn:hover { background: #d87018; }

        .notification {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 15px 25px;
            background: white;
            border-radius: 6px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.15);
            z-index: 1000;
            display: none;
            animation: slideIn 0.3s ease;
        }

        @keyframes slideIn {
            from { transform: translateX(400px); opacity: 0; }
            to { transform: translateX(0); opacity: 1; }
        }

        .notification.success { border-left: 4px solid #27ae60; }
        .notification.error { border-left: 4px solid #e74c3c; }
        .notification.info { border-left: 4px solid #3498db; }
        
        /* Exercise Testing Page */
        .exercise-page {
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100vh;
            background: #0c2d29;
            z-index: 2000;
            overflow-y: auto;
        }
        
        .exercise-page.active {
            display: block;
        }
        
        .exercise-header {
            background: #3498db;
            padding: 20px 40px;
            color: white;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        
        .exercise-header h1 {
            font-size: 20px;
            font-weight: bold;
        }
        
        .back-btn {
            padding: 10px 20px;
            background: rgba(255, 255, 255, 0.2);
            border: 1px solid rgba(255, 255, 255, 0.4);
            border-radius: 4px;
            color: white;
            cursor: pointer;
            font-weight: bold;
            transition: all 0.3s;
        }
        
        .back-btn:hover {
            background: rgba(255, 255, 255, 0.3);
        }
        
        .exercise-content {
            padding: 40px;
            max-width: 1400px;
            margin: 0 auto;
            background: #0c2d29;
        }
        
        .exercise-title-bar {
            background: rgba(255, 255, 255, 0.15);
            color: white;
            padding: 15px 30px;
            border-radius: 6px;
            margin-bottom: 30px;
            text-align: center;
            font-size: 18px;
            font-weight: bold;
            border: 1px solid rgba(255, 255, 255, 0.2);
        }
        
        .testing-mode-section {
            margin-bottom: 30px;
        }
        
        .testing-mode-section h3 {
            color: white;
            font-size: 16px;
            margin-bottom: 15px;
            font-weight: bold;
        }
        
        .radio-group {
            display: flex;
            gap: 30px;
        }
        
        .radio-option {
            display: flex;
            align-items: center;
            gap: 8px;
        }
        
        .radio-option input[type="radio"] {
            width: 18px;
            height: 18px;
            cursor: pointer;
        }
        
        .radio-option label {
            color: white;
            font-size: 14px;
            cursor: pointer;
        }
        
        .exercise-image-container {
            display: flex;
            justify-content: center;
            margin: 30px 0;
        }
        
        .exercise-image {
            border: 2px solid rgba(255, 255, 255, 0.2);
            border-radius: 6px;
            padding: 20px;
            background: rgba(255, 255, 255, 0.1);
            max-width: 400px;
        }
        
        .exercise-image img {
            width: 100%;
            height: auto;
            display: block;
        }
        
        .graph-area {
            border: 2px solid rgba(255, 255, 255, 0.2);
            border-radius: 6px;
            background: rgba(255, 255, 255, 0.05);
            min-height: 500px;
            margin: 30px 0;
            padding: 20px;
            display: flex;
            align-items: center;
            justify-content: center;
            color: rgba(255, 255, 255, 0.6);
            font-style: italic;
        }
        
        .graph-area img {
            max-width: 100%;
            height: auto;
            border-radius: 6px;
        }
        
        .control-buttons {
            display: flex;
            gap: 15px;
            margin-top: 30px;
        }
        
        .control-btn {
            padding: 12px 30px;
            border: none;
            border-radius: 4px;
            font-weight: bold;
            font-size: 14px;
            cursor: pointer;
            transition: all 0.3s;
        }
        
        .control-btn.start {
            background: rgba(52, 152, 219, 0.9);
            color: white;
        }
        
        .control-btn.start:hover {
            background: rgba(41, 128, 185, 0.9);
        }
        
        .control-btn.reset {
            background: rgba(230, 126, 34, 0.9);
            color: white;
        }
        
        .control-btn.reset:hover {
            background: rgba(216, 112, 24, 0.9);
        }
        
        .control-btn.save {
            background: rgba(39, 174, 96, 0.9);
            color: white;
            margin-left: auto;
        }
        
        .control-btn.save:hover {
            background: rgba(34, 153, 84, 0.9);
        }
        
        .progress-bar-container {
            flex: 1;
            height: 40px;
            background: rgba(255, 255, 255, 0.2);
            border-radius: 4px;
            overflow: hidden;
            display: flex;
            align-items: center;
            padding: 0 15px;
            border: 1px solid rgba(255, 255, 255, 0.3);
        }
        
        .progress-bar {
            height: 20px;
            background: rgba(52, 152, 219, 0.9);
            border-radius: 10px;
            transition: width 0.3s;
            width: 0%;
        }
    </style>
</head>
<body>
    <!-- Main Dashboard -->
    <div class="container" id="mainDashboard">
        <div class="header">
            <h1>ROBOTIC ISOMETRIC MUSCLE TESTING</h1>
            <p>Advanced Drag Mode Control & Exercise Management</p>
        </div>
        
        <div class="top-row">
            <div class="card">
                <div class="card-title">
                    <span class="bullet">●</span>
                    <h2>Drag Mode Control</h2>
                </div>
                <div class="separator"></div>
                
                <div class="status-box">
                    <div class="status-indicator connecting" id="statusIndicator">●</div>
                    <div class="status-text">
                        <h3 id="statusTitle">Status: Connecting...</h3>
                        <p id="statusMessage">Establishing connection to robot</p>
                    </div>
                </div>
                
                <div class="button-row">
                    <button class="btn btn-primary" id="dragModeBtn" onclick="toggleDragMode()" disabled>Enable Drag Mode</button>
                    <button class="btn btn-secondary" onclick="showProgressReport()">Progress Report</button>
                </div>
            </div>
            
            <div class="card">
                <div class="card-title">
                    <span class="bullet">●</span>
                    <h2>Patient Information</h2>
                </div>
                <div class="separator"></div>
                
                <div class="form-row">
                    <label>Full Name:</label>
                    <input type="text" id="patientName" placeholder="Enter patient name" onchange="savePatientData()">
                </div>
                <div class="form-row">
                    <label>Age:</label>
                    <input type="text" id="patientAge" placeholder="Enter age" onchange="savePatientData()">
                </div>
                <div class="form-row">
                    <label>Gender:</label>
                    <select id="patientGender" onchange="savePatientData()">
                        <option value="">Select gender</option>
                        <option value="Male">Male</option>
                        <option value="Female">Female</option>
                        <option value="Other">Other</option>
                    </select>
                </div>
                <div class="form-row">
                    <label>Diagnosis:</label>
                    <input type="text" id="patientDiagnosis" placeholder="Enter diagnosis" onchange="savePatientData()">
                </div>
            </div>
        </div>
        
        <div class="card exercise-card">
            <div class="card-title">
                <span class="bullet">●</span>
                <h2>Upper Limb Exercises</h2>
            </div>
            <div class="separator"></div>
            
            <div class="exercise-grid">
                <button class="exercise-btn upper-btn" onclick="startExercise('Shoulder Flexion')">Shoulder Flexion</button>
                <button class="exercise-btn upper-btn" onclick="startExercise('Shoulder Extension')">Shoulder Extension</button>
                <button class="exercise-btn upper-btn" onclick="startExercise('Shoulder Abduction')">Shoulder Abduction</button>
                <button class="exercise-btn upper-btn" onclick="startExercise('Shoulder Adduction')">Shoulder Adduction</button>
                <button class="exercise-btn upper-btn" onclick="startExercise('Elbow Flexion')">Elbow Flexion</button>
                <button class="exercise-btn upper-btn" onclick="startExercise('Elbow Extension')">Elbow Extension</button>
                <button class="exercise-btn upper-btn" onclick="startExercise('Wrist Flexion')">Wrist Flexion</button>
                <button class="exercise-btn upper-btn" onclick="startExercise('Wrist Extension')">Wrist Extension</button>
            </div>
        </div>
        
        <div class="card exercise-card lower">
            <div class="card-title">
                <span class="bullet">●</span>
                <h2>Lower Limb Exercises</h2>
            </div>
            <div class="separator"></div>
            
            <div class="exercise-grid">
                <button class="exercise-btn lower-btn" onclick="startExercise('Hip Flexion')">Hip Flexion</button>
                <button class="exercise-btn lower-btn" onclick="startExercise('Hip Extension')">Hip Extension</button>
                <button class="exercise-btn lower-btn" onclick="startExercise('Hip Abduction')">Hip Abduction</button>
                <button class="exercise-btn lower-btn" onclick="startExercise('Hip Adduction')">Hip Adduction</button>
                <button class="exercise-btn lower-btn" onclick="startExercise('Ankle Plantarflexion')">Ankle Plantarflexion</button>
                <button class="exercise-btn lower-btn" onclick="startExercise('Ankle Dorsiflexion')">Ankle Dorsiflexion</button>
                <button class="exercise-btn lower-btn" onclick="startExercise('Knee_extention')">Knee_extention</button>
                <button class="exercise-btn lower-btn" onclick="startExercise('Knee_flexion')">Knee_flexion</button>
            </div>
        </div>
    </div>

    <!-- Exercise Testing Page -->
    <div class="exercise-page" id="exercisePage">
        <div class="exercise-header">
            <h1> Muscle Testing Pannel </h1>
            <button class="back-btn" onclick="backToDashboard()">Back to Dashboard</button>
        </div>
        
        <div class="exercise-content">
            <div class="exercise-title-bar">
                <span id="exerciseTitle">Shoulder Adduction | Axis: Fz</span>
            </div>
            
            <div class="exercise-image-container">
                <div class="exercise-image">
                    <img id="exerciseImg" src="" alt="Exercise Illustration">
                </div>
            </div>
            
            <div class="graph-area" id="graphArea">
                Graph will appear here when trial starts
            </div>
            
            <div class="control-buttons">
                <button class="control-btn start" onclick="startTrial()">Start Trial</button>
                <button class="control-btn reset" onclick="resetTrials()">Reset Trial</button>
                <div class="progress-bar-container">
                    <div class="progress-bar" id="progressBar"></div>
                </div>
                <button class="control-btn save" onclick="saveToCSV()">Save to CSV</button>
            </div>
        </div>
    </div>

    <!-- Progress Report Page -->
    <div class="exercise-page" id="progressReportPage">
        <div class="exercise-header">
            <h1>Progress Report</h1>
            <button class="back-btn" onclick="backToDashboard()">Back to Dashboard</button>
        </div>
        
        <div class="exercise-content">
            <div style="background: rgba(255,255,255,0.1); border: 1px solid rgba(255,255,255,0.2); border-radius: 8px; padding: 30px; margin-bottom: 30px;">
                <h3 style="color: white; margin-bottom: 15px;">Search Patient Records</h3>
                <div style="display: flex; gap: 10px;">
                    <input type="text" id="searchPatientName" placeholder="Enter patient name..." 
                           style="flex: 1; padding: 12px; border-radius: 6px; border: 1px solid rgba(255,255,255,0.3); background: rgba(255,255,255,0.1); color: white; font-size: 14px;">
                    <button class="control-btn start" onclick="searchPatientData()" style="padding: 12px 30px;">Search</button>
                </div>
            </div>
            
            <div id="progressReportData" style="color: white;">
                <p style="text-align: center; color: rgba(255,255,255,0.6);">Enter a patient name to view their progress report</p>
            </div>
        </div>
    </div>

    <div class="notification" id="notification"></div>

    <script>
        const API_BASE = window.location.origin + '/api';
        let dragModeActive = false;
        let isConnected = false;
        let currentExerciseName = '';
        
        // Exercise images mapping
        const exerciseImages = {
            'Shoulder Flexion': '/api/exercise/image/Shoulder_flexion.jpeg',
            'Shoulder Extension': '/exercise/image/archana.jpg',
            'Shoulder Abduction': '/api/exercise/image/shoulder_adduction.jpeg',
            'Shoulder Adduction': '/api/exercise/image/shoulder_adduction.jpeg',
            'Elbow Flexion': '/api/exercise/image/Elbow_flexion.jpeg',
            'Elbow Extension': '/api/exercise/image/Elbow_extension.jpeg',
            'Wrist Flexion': '/api/exercise/image/wrist_flexion.jpeg',
            'Wrist Extension': '/api/exercise/image/Wrist_extention.jpeg',
            'Hip Flexion': '/api/exercise/image/Hip_flexion.jpeg',
            'Hip Extension': '/api/exercise/image/Hip_extention.jpeg',
            'Hip Abduction': '/api/exercise/image/Hip_abduction.jpeg',
            'Hip Adduction': '/api/exercise/image/Hip_adduction.jpeg',
            'Ankle Plantarflexion': '/api/exercise/image/Plantarflexion.jpeg',
            'Ankle Dorsiflexion': '/api/exercise/image/Dorsiflexion.jpeg',
            'Knee_extention': '/api/exercise/image/Knee_extention.jpeg',
            'Knee_flexion': '/api/exercise/image/Knee_flexion.jpeg'
        };

        window.addEventListener('load', () => {
            loadPatientData();
            // Check status immediately and then every second
            updateStatus();
            setInterval(updateStatus, 1000);
        });

        async function updateStatus() {
            try {
                const response = await fetch(`${API_BASE}/status`);
                const data = await response.json();
                dragModeActive = data.drag_mode_active;
                isConnected = data.connected;
                updateStatusUI(isConnected, dragModeActive);
            } catch (error) {
                console.error('Status check failed:', error);
                isConnected = false;
                updateStatusUI(false, false);
            }
        }

        function updateStatusUI(connected, active) {
            const indicator = document.getElementById('statusIndicator');
            const title = document.getElementById('statusTitle');
            const message = document.getElementById('statusMessage');
            const btn = document.getElementById('dragModeBtn');

            if (active) {
                indicator.className = 'status-indicator active';
                title.textContent = 'Status: Active';
                message.textContent = 'Free-drive mode enabled';
                btn.textContent = 'Disable Drag Mode';
                btn.className = 'btn btn-danger';
                btn.disabled = false;
            } else if (connected) {
                indicator.className = 'status-indicator active';
                title.textContent = 'Status: Connected';
                message.textContent = 'Robot connected - Ready to enable drag mode';
                btn.textContent = 'Enable Drag Mode';
                btn.className = 'btn btn-primary';
                btn.disabled = false;
            } else {
                indicator.className = 'status-indicator connecting';
                title.textContent = 'Status: Connecting...';
                message.textContent = 'Attempting to connect to robot';
                btn.textContent = 'Enable Drag Mode';
                btn.className = 'btn btn-primary';
                btn.disabled = true;
            }
        }

        async function toggleDragMode() {
            const btn = document.getElementById('dragModeBtn');
            const originalText = btn.textContent;
            btn.disabled = true;
            btn.textContent = 'Processing...';

            try {
                const response = await fetch(`${API_BASE}/drag_mode/toggle`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' }
                });
                
                const data = await response.json();
                
                if (data.success) {
                    showNotification(
                        data.active ? 'Drag mode enabled successfully' : 'Drag mode disabled',
                        'success'
                    );
                    // Update status immediately
                    await updateStatus();
                } else {
                    showNotification('Failed to toggle drag mode', 'error');
                    btn.textContent = originalText;
                    btn.disabled = false;
                }
            } catch (error) {
                showNotification('Connection error: ' + error.message, 'error');
                btn.textContent = originalText;
                btn.disabled = false;
            }
        }

        async function savePatientData() {
            const data = {
                name: document.getElementById('patientName').value,
                age: document.getElementById('patientAge').value,
                gender: document.getElementById('patientGender').value,
                diagnosis: document.getElementById('patientDiagnosis').value
            };

            try {
                const response = await fetch(`${API_BASE}/patient`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(data)
                });
                
                if (response.ok) {
                    showNotification('Patient data saved', 'success');
                }
            } catch (error) {
                showNotification('Failed to save patient data', 'error');
            }
        }

        async function loadPatientData() {
            try {
                const response = await fetch(`${API_BASE}/patient`);
                const data = await response.json();
                
                document.getElementById('patientName').value = data.name || '';
                document.getElementById('patientAge').value = data.age || '';
                document.getElementById('patientGender').value = data.gender || '';
                document.getElementById('patientDiagnosis').value = data.diagnosis || '';
            } catch (error) {
                console.error('Failed to load patient data:', error);
            }
        }

        async function startExercise(exerciseName) {
            currentExerciseName = exerciseName;
            
            // Update exercise page title
            document.getElementById('exerciseTitle').textContent = `${exerciseName} | Axis: Fz`;
            
            // Set exercise image
            const imgElement = document.getElementById('exerciseImg');
            imgElement.src = exerciseImages[exerciseName] || '';
            imgElement.alt = exerciseName;
            
            // Show exercise page
            document.getElementById('mainDashboard').style.display = 'none';
            document.getElementById('exercisePage').classList.add('active');
            
            try {
                const response = await fetch(`${API_BASE}/exercise/start`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ exercise_name: exerciseName })
                });
                
                const data = await response.json();
                
                if (data.success) {
                    showNotification(`Exercise loaded: ${exerciseName}`, 'success');
                } else {
                    showNotification('Failed to start exercise', 'error');
                }
            } catch (error) {
                showNotification('Connection error: ' + error.message, 'error');
            }
        }
        
        function backToDashboard() {
            document.getElementById('exercisePage').classList.remove('active');
            document.getElementById('progressReportPage').style.display = 'none';
            document.getElementById('mainDashboard').style.display = 'block';
            currentExerciseName = '';
            // Reset the graph area
            document.getElementById('graphArea').innerHTML = 'Graph will appear here when trial starts';
            document.getElementById('progressBar').style.width = '0%';
        }
        
        async function startTrial() {
            if (!isConnected) {
                showNotification('Robot not connected. Please wait for connection.', 'error');
                return;
            }
            
            // Disable start button immediately
            const startBtn = document.querySelector('.control-btn.start');
            startBtn.disabled = true;
            startBtn.textContent = 'Recording...';
            
            // Show immediate feedback
            showNotification('🔴 Recording started - 5 seconds', 'info');
            
            // Start progress bar immediately (actual 5-second duration)
            const progressBar = document.getElementById('progressBar');
            progressBar.style.width = '0%';
            progressBar.style.transition = 'width 5s linear';
            progressBar.style.width = '100%';
            
            // Update button text with countdown
            let secondsLeft = 5;
            const countdownInterval = setInterval(() => {
                secondsLeft--;
                if (secondsLeft > 0) {
                    startBtn.textContent = `${secondsLeft}s left...`;
                }
            }, 1000);
            
            try {
                const response = await fetch(`${API_BASE}/trial/start`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ 
                        exercise_name: currentExerciseName,
                        duration: 5.0 
                    })
                });
                
                const data = await response.json();
                
                clearInterval(countdownInterval);
                
                if (data.success) {
                    startBtn.disabled = false;
                    startBtn.textContent = 'Start Trial';
                    showNotification('✓ Trial completed! Processing graph...', 'success');
                    plotTrialResults(data.results);
                    
                    // Poll for plot if not ready yet (plot generates in background)
                    if (!data.results?.plot_image_base64) {
                        let pollAttempts = 0;
                        const pollInterval = setInterval(async () => {
                            pollAttempts++;
                            try {
                                const statusResponse = await fetch(`${API_BASE}/trial/plot-status`);
                                const statusData = await statusResponse.json();
                                
                                if (statusData.ready && statusData.plot_image_base64) {
                                    clearInterval(pollInterval);
                                    data.results.plot_image_base64 = statusData.plot_image_base64;
                                    plotTrialResults(data.results);
                                    showNotification('✓ Graph ready!', 'success');
                                }
                            } catch (e) {
                                // Silent fail on polling
                            }
                            
                            // Stop polling after 30 attempts (30 seconds)
                            if (pollAttempts >= 30) {
                                clearInterval(pollInterval);
                            }
                        }, 1000);
                    }
                } else {
                    showNotification('Failed to record trial', 'error');
                    startBtn.disabled = false;
                    startBtn.textContent = 'Start Trial';
                }
            } catch (error) {
                clearInterval(countdownInterval);
                showNotification('Connection error: ' + error.message, 'error');
                startBtn.disabled = false;
                startBtn.textContent = 'Start Trial';
                progressBar.style.width = '0%';
            }
        }
        
        function plotTrialResults(results) {
            const graphArea = document.getElementById('graphArea');
            
            console.log('plotTrialResults called with:', results);
            console.log('Has plot_image_base64?', !!results?.plot_image_base64);
            
            let graphHTML = '';
            // If we have a base64 encoded image from matplotlib, display it
            if (results?.plot_image_base64) {
                console.log('Base64 image found, displaying...');
                console.log('Image length:', results.plot_image_base64.length);
                graphHTML = `<img src="data:image/png;base64,${results.plot_image_base64}" style="max-width: 100%; height: auto; border-radius: 6px;" onerror="console.log('Image failed to load')">`;
            } else {
                console.log('No base64 image, showing placeholder');
                graphHTML = `<div style="height: 400px; display: flex; align-items: center; justify-content: center; color: rgba(255,255,255,0.6);">
                    Force vs Time Graph<br><small>(No data available)</small>
                </div>`;
            }
            
            graphArea.innerHTML = `
                <div style="width: 100%;">
                    <div style="margin-bottom: 30px;">
                        ${graphHTML}
                    </div>
                    <div style="background: rgba(255,255,255,0.1); border: 1px solid rgba(255,255,255,0.2); border-radius: 8px; padding: 30px;">
                        <h3 style="color: white; font-size: 20px; font-weight: bold; margin-bottom: 20px; text-align: center;">ISOMETRIC RESULTS</h3>
                        
                        <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 15px;">
                            <div style="background: rgba(255,255,255,0.08); border: 1px solid rgba(255,255,255,0.2); border-radius: 6px; padding: 15px;">
                                <div style="font-weight: bold; color: rgba(255,255,255,0.8); margin-bottom: 5px;">Peak Force</div>
                                <div style="font-size: 24px; font-weight: bold; color: #27ae60;">${results?.peak_force || '0.00'} N</div>
                                <div style="font-size: 12px; color: rgba(255,255,255,0.6); margin-top: 3px;">(${(results?.peak_force / 9.81 || 0).toFixed(2)} kg)</div>
                            </div>
                            
                            <div style="background: rgba(255,255,255,0.08); border: 1px solid rgba(255,255,255,0.2); border-radius: 6px; padding: 15px;">
                                <div style="font-weight: bold; color: rgba(255,255,255,0.8); margin-bottom: 5px;">Average Force</div>
                                <div style="font-size: 24px; font-weight: bold; color: #3498db;">${results?.avg_force || '0.00'} N</div>
                                <div style="font-size: 12px; color: rgba(255,255,255,0.6); margin-top: 3px;">(${(results?.avg_force / 9.81 || 0).toFixed(2)} kg)</div>
                            </div>
                            
                            <div style="background: rgba(255,255,255,0.08); border: 1px solid rgba(255,255,255,0.2); border-radius: 6px; padding: 15px;">
                                <div style="font-weight: bold; color: rgba(255,255,255,0.8); margin-bottom: 5px;">Start Force</div>
                                <div style="font-size: 24px; font-weight: bold; color: #e74c3c;">${results?.start_force || '0.00'} N</div>
                                <div style="font-size: 12px; color: rgba(255,255,255,0.6); margin-top: 3px;">(${(results?.start_force / 9.81 || 0).toFixed(2)} kg)</div>
                            </div>
                            
                            <div style="background: rgba(255,255,255,0.08); border: 1px solid rgba(255,255,255,0.2); border-radius: 6px; padding: 15px;">
                                <div style="font-weight: bold; color: rgba(255,255,255,0.8); margin-bottom: 5px;">Time to Peak</div>
                                <div style="font-size: 24px; font-weight: bold; color: #f39c12;">${results?.time_to_peak || '0.00'} s</div>
                            </div>
                            
                            <div style="background: rgba(255,255,255,0.08); border: 1px solid rgba(255,255,255,0.2); border-radius: 6px; padding: 15px;">
                                <div style="font-weight: bold; color: rgba(255,255,255,0.8); margin-bottom: 5px;">Early RFD</div>
                                <div style="font-size: 24px; font-weight: bold; color: #1abc9c;">${results?.early_rfd || '0'} N/s</div>
                                <div style="font-size: 12px; color: rgba(255,255,255,0.6); margin-top: 3px;">(50-100 ms)</div>
                            </div>
                        </div>
                    </div>
                </div>
            `;
            
            showNotification('Trial completed successfully!', 'success');
        }
        
        function resetTrials() {
            document.getElementById('graphArea').innerHTML = 'Graph will appear here when trial starts';
            document.getElementById('progressBar').style.width = '0%';
            showNotification('Trial reset', 'info');
        }
        
        function saveToCSV() {
            showNotification('Saving data to CSV...', 'info');
            
            fetch(`${API_BASE}/trial/save`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ exercise_name: currentExerciseName })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showNotification('Data saved to CSV successfully!', 'success');
                } else {
                    showNotification('Failed to save data', 'error');
                }
            })
            .catch(error => {
                showNotification('Error saving data: ' + error.message, 'error');
            });
        }

        function showProgressReport() {
            document.getElementById('mainDashboard').style.display = 'none';
            document.getElementById('exercisePage').style.display = 'none';
            document.getElementById('progressReportPage').style.display = 'block';
            document.getElementById('searchPatientName').value = '';
            document.getElementById('progressReportData').innerHTML = '<p style="text-align: center; color: rgba(255,255,255,0.6);">Enter a patient name to view their progress report</p>';
        }
        
        async function searchPatientData() {
            const patientName = document.getElementById('searchPatientName').value.trim();
            
            if (!patientName) {
                showNotification('Please enter a patient name', 'error');
                return;
            }
            
            try {
                const response = await fetch(`${API_BASE}/patient/progress?name=${encodeURIComponent(patientName)}`);
                const data = await response.json();
                
                if (data.success && data.records && data.records.length > 0) {
                    displayProgressReport(data.records, patientName);
                } else {
                    document.getElementById('progressReportData').innerHTML = `
                        <div style="text-align: center; padding: 40px;">
                            <p style="color: rgba(255,255,255,0.8); font-size: 16px;">No records found for patient: <strong>${patientName}</strong></p>
                        </div>
                    `;
                }
            } catch (error) {
                showNotification('Error fetching patient data: ' + error.message, 'error');
            }
        }
        
        function displayProgressReport(records, patientName) {
            let html = `
                <div style="margin-bottom: 30px;">
                    <h2 style="color: white; margin-bottom: 10px;">Patient: ${patientName}</h2>
                    <p style="color: rgba(255,255,255,0.7);">Total Records: ${records.length}</p>
                </div>
                
                <div style="display: grid; grid-template-columns: repeat(auto-fill, minmax(350px, 1fr)); gap: 20px;">
            `;
            
            records.forEach((record, index) => {
                const timestamp = new Date(record.timestamp).toLocaleString();
                const peakForceN = parseFloat(record.peak_force || 0);
                const peakForceKg = peakForceN / 9.81;
                const avgForceN = parseFloat(record.avg_force || 0);
                const avgForceKg = avgForceN / 9.81;
                const startForceN = parseFloat(record.start_force || 0);
                const startForceKg = startForceN / 9.81;
                const maxForceKg = 25;
                const progressPercent = Math.min((peakForceKg / maxForceKg) * 100, 100);
                
                html += `
                    <div style="background: rgba(255,255,255,0.08); border: 1px solid rgba(255,255,255,0.2); border-radius: 8px; padding: 20px; display: flex; flex-direction: column; height: 100%;">
                        <div style="background: rgba(52,152,219,0.3); padding: 12px; border-radius: 6px; margin-bottom: 15px;">
                            <p style="color: #3498db; font-weight: bold; margin: 0;">Trial ${index + 1}</p>
                            <p style="color: rgba(255,255,255,0.7); font-size: 12px; margin: 3px 0 0 0;">${timestamp}</p>
                        </div>
                        
                        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 15px;">
                            <div style="background: rgba(255,255,255,0.05); padding: 10px; border-radius: 6px;">
                                <p style="color: rgba(255,255,255,0.6); font-size: 12px; margin: 0;">Exercise</p>
                                <p style="color: white; font-weight: bold; margin: 5px 0 0 0;">${record.exercise || 'N/A'}</p>
                            </div>
                            
                            <div style="background: rgba(255,255,255,0.05); padding: 10px; border-radius: 6px;">
                                <p style="color: rgba(255,255,255,0.6); font-size: 12px; margin: 0;">Mode</p>
                                <p style="color: white; font-weight: bold; margin: 5px 0 0 0;">${record.mode || 'Isometric'}</p>
                            </div>
                            
                            <div style="background: rgba(255,255,255,0.05); padding: 10px; border-radius: 6px;">
                                <p style="color: rgba(255,255,255,0.6); font-size: 12px; margin: 0;">Peak Force</p>
                                <p style="color: #27ae60; font-weight: bold; margin: 5px 0 0 0;">${peakForceKg.toFixed(2)} kg</p>
                                <p style="color: rgba(255,255,255,0.5); font-size: 11px; margin: 3px 0 0 0;">${peakForceN.toFixed(2)} N</p>
                            </div>
                            
                            <div style="background: rgba(255,255,255,0.05); padding: 10px; border-radius: 6px;">
                                <p style="color: rgba(255,255,255,0.6); font-size: 12px; margin: 0;">Avg Force</p>
                                <p style="color: #f39c12; font-weight: bold; margin: 5px 0 0 0;">${avgForceKg.toFixed(2)} kg</p>
                                <p style="color: rgba(255,255,255,0.5); font-size: 11px; margin: 3px 0 0 0;">${avgForceN.toFixed(2)} N</p>
                            </div>
                            
                            <div style="background: rgba(255,255,255,0.05); padding: 10px; border-radius: 6px;">
                                <p style="color: rgba(255,255,255,0.6); font-size: 12px; margin: 0;">Start Force</p>
                                <p style="color: #e74c3c; font-weight: bold; margin: 5px 0 0 0;">${startForceKg.toFixed(2)} kg</p>
                                <p style="color: rgba(255,255,255,0.5); font-size: 11px; margin: 3px 0 0 0;">${startForceN.toFixed(2)} N</p>
                            </div>
                            
                            <div style="background: rgba(255,255,255,0.05); padding: 10px; border-radius: 6px;">
                                <p style="color: rgba(255,255,255,0.6); font-size: 12px; margin: 0;">Time to Peak</p>
                                <p style="color: white; font-weight: bold; margin: 5px 0 0 0;">${parseFloat(record.time_to_peak || 0).toFixed(3)} s</p>
                            </div>
                            
                            <div style="background: rgba(255,255,255,0.05); padding: 10px; border-radius: 6px; grid-column: 1 / -1;">
                                <p style="color: rgba(255,255,255,0.6); font-size: 12px; margin: 0;">Early RFD (50-100ms)</p>
                                <p style="color: #3498db; font-weight: bold; margin: 5px 0 0 0;">${parseInt(record.early_rfd || 0)} N/s</p>
                            </div>
                        </div>
                        
                        <div style="margin-top: auto; padding-top: 15px; border-top: 1px solid rgba(255,255,255,0.1);">
                            <div style="display: flex; align-items: center; gap: 10px;">
                                <span style="font-size: 18px;">💪🏽</span>
                                <div style="flex: 1;">
                                    <div style="background: rgba(255,255,255,0.2); border-radius: 4px; height: 24px; overflow: hidden; border: 1px solid rgba(255,255,255,0.3);">
                                        <div style="background: linear-gradient(90deg, #27ae60, #2ecc71); width: ${progressPercent}%; height: 100%; transition: width 0.3s ease; display: flex; align-items: center; justify-content: flex-end; padding-right: 6px;">
                                            <span style="color: white; font-weight: bold; font-size: 11px;">${peakForceKg.toFixed(1)}kg</span>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                `;
            });
            
            html += `</div>`;
            document.getElementById('progressReportData').innerHTML = html;
        }

        function showNotification(message, type) {
            const notification = document.getElementById('notification');
            notification.textContent = message;
            notification.className = `notification ${type}`;
            notification.style.display = 'block';

            setTimeout(() => {
                notification.style.display = 'none';
            }, 3000);
        }
    </script>
</body>
</html>"""


class RobotController:
    """Main robot control class"""
    
    def __init__(self):
        self.robot = None
        self.baseline_forces = None
        self.monitoring_active = False
        self.monitoring_thread = None
        self.drag_mode_active = False
        self.current_forces = [0.0] * 6
        self.current_robot_joints = None
        self.connection_attempted = False
        
        # Patient data
        self.patient_data = {
            'name': '',
            'age': '',
            'gender': '',
            'diagnosis': ''
        }
        
        # Exercise tracking
        self.current_exercise = None
        self.exercise_data = {}
        self.last_trial_results = None
        self.last_plot_image = None  # Store latest plot image
        
        os.makedirs("images", exist_ok=True)
        
    def log_message(self, msg):
        """Log message with timestamp"""
        ts = datetime.now().strftime("%H:%M:%S")
        print(f"[{ts}] {msg}")
    
    def connect_robot(self):
        """Connect to the robot"""
        if self.connection_attempted and self.robot:
            return True
            
        try:
            self.log_message("Connecting to robot...")
            self.robot = Robot.RPC('192.168.58.2')
            self.connection_attempted = True
            self.log_message("✓ Robot connected successfully!")
            return True
        except Exception as e:
            self.connection_attempted = True
            self.log_message(f"✗ Connection error: {e}")
            return False
    
    def init_ft_sensor(self):
        """Initialize force/torque sensor"""
        try:
            if not self.robot:
                return False
            
            self.robot.FT_SetConfig(24, 0)
            self.robot.FT_Activate(0)
            time.sleep(0.5)
            self.robot.FT_Activate(1)
            time.sleep(0.5)
            self.robot.SetLoadWeight(0, 0.0)
            self.robot.SetLoadCoord(0, 0, 0)
            self.robot.FT_SetZero(0)
            time.sleep(0.5)
            self.robot.FT_SetZero(1)
            time.sleep(0.5)
            
            self.log_message("✓ FT sensor initialized")
            return True
        except Exception as e:
            self.log_message(f"✗ FT sensor init error: {e}")
            return False
    
    def calibrate_baseline_forces(self):
        """Calibrate baseline forces"""
        samples = []
        for _ in range(100):
            try:
                if not self.robot:
                    break
                
                d = self.robot.FT_GetForceTorqueRCS()
                if d[0] == 0:
                    samples.append([d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]])
                
                time.sleep(0.01)
            except:
                pass
        
        if samples:
            self.baseline_forces = [sum(col)/len(samples) for col in zip(*samples)]
            self.log_message("✓ Baseline calibrated")
            return True
        
        self.log_message("⚠ Baseline calibration skipped")
        return False
    
    def enable_drag_mode(self):
        """Enable drag/free-drive mode"""
        try:
            if not self.robot:
                if not self.connect_robot():
                    return False
            
            if not self.init_ft_sensor():
                self.log_message("⚠ Sensor init skipped")
            
            if not self.calibrate_baseline_forces():
                self.log_message("⚠ Baseline skipped")
            
            ret = self.robot.DragTeachSwitch(1)
            if ret != 0:
                self.log_message(f"✗ DragTeachSwitch failed: {ret}")
                return False
            
            self.drag_mode_active = True
            
            if not self.monitoring_active:
                self.start_ft_monitoring()
            
            self.log_message("✓ FREE-DRIVE ACTIVE")
            return True
            
        except Exception as e:
            self.log_message(f"✗ Enable drag error: {e}")
            return False
    
    def disable_drag_mode(self):
        """Disable drag mode"""
        try:
            if self.robot:
                self.robot.DragTeachSwitch(0)
                
            self.drag_mode_active = False
            self.log_message("✓ Drag mode disabled")
            return True
        except Exception as e:
            self.log_message(f"✗ Disable drag error: {e}")
            return False
    
    def start_ft_monitoring(self):
        """Start force/torque monitoring thread"""
        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(target=self.ft_monitoring_loop, daemon=True)
        self.monitoring_thread.start()
    
    def ft_monitoring_loop(self):
        """Monitor force/torque sensor data"""
        while self.monitoring_active:
            try:
                if not self.robot:
                    time.sleep(0.1)
                    continue
                
                d = self.robot.FT_GetForceTorqueRCS()
                if d[0] == 0:
                    raw = [d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]]
                    forces = raw if not self.baseline_forces else [raw[i] - self.baseline_forces[i] for i in range(6)]
                    for i in range(6):
                        if abs(forces[i]) < 0.5:
                            forces[i] = 0.0
                    self.current_forces = forces
                
                time.sleep(0.008)
            except:
                time.sleep(0.008)
    
    def start_exercise(self, exercise_name):
        """Start a specific exercise"""
        self.current_exercise = exercise_name
        self.log_message(f"▶ Starting exercise: {exercise_name}")
        
        # Initialize exercise data storage
        if exercise_name not in self.exercise_data:
            self.exercise_data[exercise_name] = []
        
        return True
    
    def record_trial(self, duration=5.0):
            if not self.robot or not self.monitoring_active:
                return None

            samples = []
            times = []
            start_time = time.time()

            self.log_message(f"Recording {duration}s trial for {self.current_exercise}...")

            while time.time() - start_time < duration:
                elapsed = time.time() - start_time
                try:
                    d = self.robot.FT_GetForceTorqueRCS()
                    if d[0] == 0:
                        raw = [d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]]
                        forces = raw if not self.baseline_forces else [raw[i] - self.baseline_forces[i] for i in range(6)]
                        fz_value = max(-forces[2], 0)  # Only positive push force
                        samples.append(round(fz_value, 2))
                        times.append(round(elapsed, 3))
                    time.sleep(0.008)
                except:
                    break

            if not samples:
                return None

            valid = [f for f in samples if f > 0.5]
            peak = max(valid) if valid else 0.0
            avg = sum(valid) / len(valid) if valid else 0.0
            time_to_peak = times[samples.index(peak)] if peak in samples else 0.0
            early_rfd = self.calculate_early_rfd(times, samples)

            results = {
                'peak_force': round(peak, 2),
                'avg_force': round(avg, 2),
                'start_force': round(samples[0], 2),
                'time_to_peak': round(time_to_peak, 3),
                'early_rfd': int(early_rfd),
                'times': times,
                'forces': samples
            }
            self.last_trial_results = results
            return results

    def plot_results_in_thread(self, results):
        """Plot trial results using matplotlib and return as base64 image."""
        try:
            times = results.get('times', [])
            forces = results.get('forces', [])
            
            self.log_message(f"[PLOT] Starting plot generation with {len(times)} time points and {len(forces)} force points")
            
            if not times or not forces:
                self.log_message("[PLOT] ERROR: No times or forces data")
                return None

            fig, ax = plt.subplots(figsize=(12, 5))
            ax.plot(times, forces, color='#e74c3c', linewidth=2.5, label='Force')
            ax.set_title('Force vs Time (Isometric Trial)', fontsize=14, fontweight='bold', color='black')
            ax.set_xlabel('Time (s)', fontsize=12, color='black')
            ax.set_ylabel('Force (N)', fontsize=12, color='black')
            ax.grid(True, alpha=0.3, color='gray')
            ax.set_facecolor('white')
            fig.patch.set_facecolor('white')
            
            # Style axis spines and ticks
            ax.spines['bottom'].set_color('black')
            ax.spines['left'].set_color('black')
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.tick_params(colors='black', labelsize=10)

            # Get metrics
            peak = results.get('peak_force', max(forces) if forces else None)
            avg_force = results.get('avg_force', 0)
            start_force = results.get('start_force', 0)
            time_to_peak = results.get('time_to_peak', 0)
            early_rfd = results.get('early_rfd', 0)
            
            # Mark peak force with annotation
            if peak is not None:
                try:
                    p_idx = forces.index(peak)
                    ax.plot(times[p_idx], forces[p_idx], 'o', color='#27ae60', markersize=10, label='Peak Force', zorder=5)
                    ax.text(times[p_idx], forces[p_idx] + 1, f'Peak\n{peak:.1f}N', 
                           fontsize=9, ha='center', color='#27ae60', fontweight='bold')
                except (ValueError, IndexError):
                    pass

            # Mark average force as horizontal line
            ax.axhline(y=avg_force, color='#f39c12', linestyle='--', linewidth=2, label='Average Force')
            
            # Highlight Early RFD window (50-100 ms) with line between points
            onset_idx = None
            threshold = 0.5
            for i in range(len(forces)):
                if forces[i] > threshold:
                    if all(forces[j] > threshold * 0.8 for j in range(i, min(i+10, len(forces)))):
                        onset_idx = i
                        break

            if onset_idx is not None and len(times) > 1:
                dt = times[1] - times[0]
                idx50 = onset_idx + int(0.05 / dt)
                idx100 = onset_idx + int(0.10 / dt)
                idx50 = max(0, min(len(forces)-1, idx50))
                idx100 = max(0, min(len(forces)-1, idx100))
                
                if idx50 < len(forces) and idx100 < len(forces):
                    # Draw line segment for Early RFD
                    ax.plot([times[idx50], times[idx100]], [forces[idx50], forces[idx100]], 
                           color='#3498db', linewidth=4, marker='o', markersize=8, 
                           label=f'Early RFD (50-100ms)', zorder=4)
                    # Mark start point of RFD window
                    ax.text(times[idx50], forces[idx50] - 1.5, 'RFD Start\n50ms', 
                           fontsize=8, ha='center', color='#3498db', fontweight='bold')
                    # Mark end point of RFD window
                    ax.text(times[idx100], forces[idx100] + 1, 'RFD End\n100ms', 
                           fontsize=8, ha='center', color='#3498db', fontweight='bold')

            # Place legend outside graph on right side
            ax.legend(loc='upper left', fontsize=10, facecolor='white', 
                     edgecolor='black', labelcolor='black', framealpha=0.95)
            
            # Adjust layout to make room for legend outside
            plt.tight_layout()

            # Convert to base64
            buffer = io.BytesIO()
            fig.savefig(buffer, format='png', facecolor='white', edgecolor='none', dpi=100, bbox_inches='tight')
            buffer.seek(0)
            image_base64 = base64.b64encode(buffer.read()).decode('utf-8')
            plt.close(fig)
            
            self.log_message(f"[PLOT] Successfully generated base64 image ({len(image_base64)} chars)")
            
            # Store in both results and controller for later retrieval
            results['plot_image_base64'] = image_base64
            self.last_plot_image = image_base64
            return image_base64
            
        except Exception as e:
            self.log_message(f"[PLOT] ERROR: {e}")
            import traceback
            traceback.print_exc()
            return None

    def calculate_early_rfd(self, times, forces, threshold=0.5):
        """Calculate Early RFD from 50-100ms after force onset"""
        # Find force onset
        onset_idx = None
        for i in range(len(forces)):
            if forces[i] > threshold:
                # Check if force stays above threshold for next few samples
                if all(forces[j] > threshold * 0.8 for j in range(i, min(i+10, len(forces)))):
                    onset_idx = i
                    break
        
        if onset_idx is None:
            return 0.0
        
        # Calculate time step
        dt = times[1] - times[0] if len(times) > 1 else 0.008
        
        # Find indices for 50ms and 100ms after onset
        idx_50ms = onset_idx + int(0.05 / dt)
        idx_100ms = onset_idx + int(0.10 / dt)
        
        if idx_100ms >= len(forces) or idx_50ms >= len(forces):
            return 0.0
        
        # Calculate RFD
        force_diff = forces[idx_100ms] - forces[idx_50ms]
        time_diff = times[idx_100ms] - times[idx_50ms]
        
        early_rfd = force_diff / time_diff if time_diff > 0 else 0.0
        
        return early_rfd
    
    def save_trial_to_csv(self, exercise_name):
        """Save the last trial results to CSV"""
        if not self.last_trial_results:
            return False
        
        try:
            import csv
            
            # Prepare data
            row = {
                'Timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'Exercise': exercise_name,
                'Name': self.patient_data.get('name', ''),
                'Age': self.patient_data.get('age', ''),
                'Gender': self.patient_data.get('gender', ''),
                'Diagnosis': self.patient_data.get('diagnosis', ''),
                'Mode': 'Isometric',
                'Peak_Force_N': self.last_trial_results['peak_force'],
                'Average_Force_N': self.last_trial_results['avg_force'],
                'Start_Force_N': self.last_trial_results['start_force'],
                'Time_to_Peak_s': self.last_trial_results['time_to_peak'],
                'Early_RFD_N_s': self.last_trial_results['early_rfd']
            }
            
            # Check if file exists
            file_exists = os.path.isfile('results.csv')
            
            # Write to CSV
            with open('results.csv', 'a', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=row.keys())
                if not file_exists:
                    writer.writeheader()
                writer.writerow(row)
            
            self.log_message(f"✓ Trial saved to results.csv")
            return True
            
        except Exception as e:
            self.log_message(f"✗ CSV save error: {e}")
            return False
    
    def stop_monitoring(self):
        """Stop all monitoring"""
        self.monitoring_active = False
        if self.drag_mode_active:
            self.disable_drag_mode()


# Global robot controller instance
robot_controller = RobotController()


# Flask Routes
@app.route('/')
def index():
    """Serve the main page"""
    return HTML_TEMPLATE

@app.route('/api/status')
def get_status():
    """Get current robot status"""
    return jsonify({
        'connected': robot_controller.robot is not None,
        'drag_mode_active': robot_controller.drag_mode_active,
        'monitoring_active': robot_controller.monitoring_active,
        'current_forces': robot_controller.current_forces,
        'current_exercise': robot_controller.current_exercise
    })

@app.route('/api/drag_mode/enable', methods=['POST'])
def enable_drag():
    """Enable drag mode"""
    success = robot_controller.enable_drag_mode()
    return jsonify({'success': success, 'active': robot_controller.drag_mode_active})

@app.route('/api/drag_mode/disable', methods=['POST'])
def disable_drag():
    """Disable drag mode"""
    success = robot_controller.disable_drag_mode()
    return jsonify({'success': success, 'active': robot_controller.drag_mode_active})

@app.route('/api/drag_mode/toggle', methods=['POST'])
def toggle_drag():
    """Toggle drag mode"""
    if robot_controller.drag_mode_active:
        success = robot_controller.disable_drag_mode()
    else:
        success = robot_controller.enable_drag_mode()
    return jsonify({'success': success, 'active': robot_controller.drag_mode_active})

@app.route('/api/patient', methods=['GET', 'POST'])
def patient_data():
    """Get or update patient data"""
    if request.method == 'POST':
        data = request.json
        robot_controller.patient_data.update(data)
        return jsonify({'success': True, 'data': robot_controller.patient_data})
    else:
        return jsonify(robot_controller.patient_data)

@app.route('/api/patient/progress', methods=['GET'])
def get_patient_progress():
    """Get patient progress records from CSV by name"""
    import csv
    patient_name = request.args.get('name', '').strip()
    
    if not patient_name:
        return jsonify({'success': False, 'records': []})
    
    records = []
    csv_file = 'results.csv'
    
    if os.path.exists(csv_file):
        try:
            with open(csv_file, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # Case-insensitive name search
                    if row.get('Name', '').lower() == patient_name.lower():
                        # Normalize column names
                        record = {
                            'timestamp': row.get('Timestamp', ''),
                            'exercise': row.get('Exercise', ''),
                            'mode': row.get('Mode', 'Isometric'),
                            'peak_force': row.get('Peak_Force_N', row.get('Peak', '0')),
                            'avg_force': row.get('Average_Force_N', row.get('Average', '0')),
                            'start_force': row.get('Start_Force_N', row.get('Start', '0')),
                            'time_to_peak': row.get('Time_to_Peak_s', row.get('Time_to_Peak', '0')),
                            'early_rfd': row.get('Early_RFD_N_s', row.get('Early_RFD', '0'))
                        }
                        records.append(record)
        except Exception as e:
            robot_controller.log_message(f"Error reading CSV for progress: {e}")
    
    # Sort by timestamp descending (newest first)
    records.sort(key=lambda x: x['timestamp'], reverse=True)
    
    return jsonify({
        'success': len(records) > 0,
        'records': records,
        'patient_name': patient_name
    })

@app.route('/api/exercise/start', methods=['POST'])
def start_exercise():
    """Start an exercise"""
    data = request.json
    exercise_name = data.get('exercise_name')
    success = robot_controller.start_exercise(exercise_name)
    return jsonify({'success': success, 'exercise': exercise_name})

@app.route('/api/exercise/stop', methods=['POST'])
def stop_exercise():
    """Stop current exercise"""
    robot_controller.current_exercise = None
    return jsonify({'success': True})

@app.route('/api/forces')
def get_forces():
    """Get current force/torque readings"""
    return jsonify({
        'forces': robot_controller.current_forces,
        'timestamp': datetime.now().isoformat()
    })

# ABSOLUTE PATH — THIS IS THE ONLY CHANGE NEEDED IN BACKEND
EXERCISE_IMAGES_DIR = '/home/um/fairino-python-sdk-main/images'

@app.route('/api/exercise/image/<filename>')
def get_exercise_image(filename):
    """Serve exercise images from /home/um/fairino-python-sdk-main/images"""
    # Prevent directory traversal
    filename = os.path.basename(filename)
    filepath = os.path.join(EXERCISE_IMAGES_DIR, filename)

    # 1. Try exact filename
    if os.path.isfile(filepath):
        return send_file(filepath)

    # 2. Try with common extensions (case-insensitive)
    base_name = os.path.splitext(filename)[0]
    for ext in ['.jpg', '.jpeg', '.png', '.JPG', '.JPEG', '.PNG']:
        test_path = os.path.join(EXERCISE_IMAGES_DIR, base_name + ext)
        if os.path.isfile(test_path):
            return send_file(test_path)

    # 3. If still not found → return 404 with clear message
    app.logger.warning(f"Exercise image not found: {filename}")
    return "Image not found: " + filename, 404

# @app.route('/api/exercise/image/<filename>')
# def get_exercise_image(filename):
#     """Serve exercise images - supports .jpg, .jpeg, .JPG, .JPEG, .png automatically"""
#     image_dir = 'images'
    
#     # Security: prevent directory traversal
#     filename = os.path.basename(filename)
#     requested_path = os.path.join(image_dir, filename)

#     # 1. Try exact filename first
#     if os.path.isfile(requested_path):
#         return send_file(requested_path)

#     # 2. If not found → try common extensions (case-insensitive)
#     base_name = os.path.splitext(filename)[0].lower()
#     extensions = ['.jpg', '.jpeg', '.png', '.JPG', '.JPEG', '.PNG']
    
#     for ext in extensions:
#         test_path = os.path.join(image_dir, base_name + ext)
#         if os.path.isfile(test_path):
#             return send_file(test_path)

#     # 3. Still not found? → serve a nice fallback image
#     fallback_path = os.path.join(image_dir, 'placeholder.jpg')
#     if os.path.isfile(fallback_path):
#         return send_file(fallback_path)
    
#     # 4. Last resort: generate a simple colored placeholder
#     from PIL import Image, ImageDraw, ImageFont
#     img = Image.new('RGB', (600, 600), color='#2c3e50')
#     draw = ImageDraw.Draw(img)
#     try:
#         font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 40)
#     except:
#         font = ImageFont.load_default()
    
#     text = f"{base_name.replace('_', ' ').title()}\n(Image Missing)"
#     bbox = draw.textbbox((0,0), text, font=font)
#     text_width = bbox[2] - bbox[0]
#     text_height = bbox[3] - bbox[1]
#     position = ((600-text_width)//2, (600-text_height)//2)
    
#     draw.text(position, text, fill='#e74c3c', font=font, align='center')
#     import io
#     buffer = io.BytesIO()
#     img.save(buffer, format='JPEG')
#     buffer.seek(0)
#     return send_file(buffer, mimetype='image/jpeg')

@app.route('/api/trial/start', methods=['POST'])
def start_trial():
    """Start a trial and record force data"""
    data = request.json
    exercise_name = data.get('exercise_name')
    duration = data.get('duration', 5.0)
    
    if not robot_controller.robot:
        return jsonify({'success': False, 'error': 'Robot not connected'})
    
    # Record trial data
    results = robot_controller.record_trial(duration)
    
    # Generate plot asynchronously in background thread (non-blocking)
    if results:
        robot_controller.log_message("[API] Trial recorded, spawning plot thread...")
        def generate_plot_async():
            try:
                robot_controller.log_message("[ASYNC] Plot thread started")
                image_base64 = robot_controller.plot_results_in_thread(results)
                if image_base64:
                    results['plot_image_base64'] = image_base64
                    robot_controller.log_message(f"[ASYNC] Plot generated and stored ({len(image_base64)} chars)")
            except Exception as e:
                robot_controller.log_message(f"[ASYNC] Plot generation error: {e}")
        
        # Start plotting in background thread to avoid blocking response
        plot_thread = threading.Thread(target=generate_plot_async, daemon=True)
        plot_thread.start()

    return jsonify({
        'success': True,
        'results': results
    })

@app.route('/api/trial/save', methods=['POST'])
def save_trial():
    """Save trial data to CSV"""
    data = request.json
    exercise_name = data.get('exercise_name')
    
    success = robot_controller.save_trial_to_csv(exercise_name)
    
    return jsonify({'success': success})

@app.route('/api/trial/plot-status', methods=['GET'])
def get_plot_status():
    """Check if plot is ready (for polling)"""
    if robot_controller.last_plot_image:
        return jsonify({
            'ready': True,
            'plot_image_base64': robot_controller.last_plot_image
        })
    else:
        return jsonify({'ready': False})


def open_browser():
    """Open browser after a short delay"""
    time.sleep(1.5)  # Wait for server to start
    webbrowser.open('http://localhost:5000')


def auto_connect_robot():
    """Automatically connect to robot on startup"""
    time.sleep(2)  # Wait for server to be ready
    robot_controller.log_message("Attempting auto-connection to robot...")
    robot_controller.connect_robot()


if __name__ == '__main__':
    try:
        print("=" * 70)
        print("  ROBOT REHABILITATION CONTROL SYSTEM - WEB SERVER")
        print("=" * 70)
        print("\n  🚀 Server starting...")
        print(f"  🌐 URL: http://localhost:5000")
        print(f"  🤖 Auto-connecting to robot at 192.168.58.2")
        print(f"\n  Press Ctrl+C to stop the server")
        print("=" * 70)
        print()
        
        # Start browser opening thread
        browser_thread = threading.Thread(target=open_browser, daemon=True)
        browser_thread.start()
        
        # Start robot connection thread
        robot_thread = threading.Thread(target=auto_connect_robot, daemon=True)
        robot_thread.start()
        
        # Start Flask server
        app.run(debug=False, host='0.0.0.0', port=5000, threaded=True, use_reloader=False)
        
    except KeyboardInterrupt:
        print("\n\n" + "=" * 70)
        print("  Shutting down gracefully...")
        print("=" * 70)
        robot_controller.stop_monitoring()
        print("  ✓ Server stopped.")
        print("=" * 70)