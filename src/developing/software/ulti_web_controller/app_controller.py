'''
Robot Controller Server - Modified Version
Removed gyro mode, only manual and follow modes
Runs on port 5000
'''
from flask import Flask, render_template_string, request, jsonify
import socket_client
import threading
import time
import os

app = Flask(__name__)

# Global variables to track robot state
current_mode = "manual"
robot_status = "disconnected"
pi_ip = "192.168.16.154"  # Default Pi IP
pi_port = 8888

# Long-lived socket client
robot_client = None
client_lock = threading.Lock()

# Tracking data cache for follow mode - now includes distance
tracking_data = {'status': 'Searching...', 'error': 0, 'distance': 999, 'action': 'Waiting...'}
data_lock = threading.Lock()

# HTML template with gyro mode removed
CONTROLLER_HTML = '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🎮 Robot Controller</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }

        .container {
            max-width: 800px;
            margin: 0 auto;
            background: rgba(255, 255, 255, 0.95);
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
        }

        .header {
            text-align: center;
            margin-bottom: 30px;
        }

        .header h1 {
            color: #333;
            margin-bottom: 10px;
            font-size: 2.5em;
        }

        .header p {
            color: #666;
            font-size: 1.1em;
        }

        .status-bar {
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: #f8f9fa;
            padding: 15px;
            border-radius: 10px;
            margin-bottom: 30px;
            flex-wrap: wrap;
            gap: 10px;
        }

        .status-item {
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #dc3545;
            animation: pulse 2s infinite;
        }

        .status-dot.connected {
            background: #28a745;
            animation: none;
        }

        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        .control-section {
            margin-bottom: 30px;
        }

        .section-title {
            font-size: 1.4em;
            color: #333;
            margin-bottom: 15px;
            border-bottom: 2px solid #667eea;
            padding-bottom: 8px;
        }

        .mode-buttons {
            display: flex;
            gap: 10px;
            margin-bottom: 20px;
            flex-wrap: wrap;
        }

        .mode-btn {
            flex: 1;
            min-width: 150px;
            padding: 15px 20px;
            border: none;
            border-radius: 10px;
            font-weight: bold;
            font-size: 1.1em;
            cursor: pointer;
            transition: all 0.3s ease;
            background: #e9ecef;
            color: #495057;
        }

        .mode-btn.active {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(102, 126, 234, 0.4);
        }

        .mode-btn:hover:not(.active) {
            background: #dee2e6;
            transform: translateY(-1px);
        }

        /* Movement Controls */
        .movement-controls {
            display: grid;
            grid-template-columns: 1fr auto 1fr;
            grid-template-rows: auto auto auto;
            gap: 15px;
            max-width: 350px;
            margin: 0 auto;
        }

        .move-btn {
            padding: 20px;
            border: none;
            border-radius: 12px;
            font-size: 1.2em;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.2s ease;
            background: #28a745;
            color: white;
            min-height: 70px;
            display: flex;
            align-items: center;
            justify-content: center;
            box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1);
        }

        .move-btn:hover:not(:disabled) {
            transform: scale(1.05);
            box-shadow: 0 6px 20px rgba(0, 0, 0, 0.2);
        }

        .move-btn:active {
            transform: scale(0.98);
        }

        .move-btn:disabled {
            background: #6c757d;
            cursor: not-allowed;
            opacity: 0.6;
        }

        .move-btn.forward { grid-column: 2; grid-row: 1; }
        .move-btn.left { grid-column: 1; grid-row: 2; }
        .move-btn.stop { 
            grid-column: 2; 
            grid-row: 2; 
            background: #dc3545;
            font-size: 1.4em;
        }
        .move-btn.right { grid-column: 3; grid-row: 2; }
        .move-btn.backward { grid-column: 2; grid-row: 3; }

        /* Follow Mode Display */
        .follow-display {
            display: none;
            background: linear-gradient(135deg, #f8f9fa 0%, #e9ecef 100%);
            padding: 25px;
            border-radius: 15px;
            text-align: center;
        }

        .follow-display.active {
            display: block;
        }

        .tracking-info {
            background: white;
            padding: 20px;
            border-radius: 10px;
            margin-top: 15px;
            box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
        }

        .tracking-info p {
            margin: 10px 0;
            font-size: 1.1em;
        }

        .distance-indicator {
            margin-top: 15px;
            padding: 15px;
            background: linear-gradient(135deg, #fff 0%, #f8f9fa 100%);
            border-radius: 10px;
            border: 2px solid #667eea;
        }

        .distance-bar {
            width: 100%;
            height: 30px;
            background: #e9ecef;
            border-radius: 15px;
            overflow: hidden;
            position: relative;
            margin-top: 10px;
        }

        .distance-fill {
            height: 100%;
            background: linear-gradient(135deg, #28a745 0%, #20c997 100%);
            transition: width 0.3s ease;
        }

        .distance-fill.warning {
            background: linear-gradient(135deg, #ffc107 0%, #ff9800 100%);
        }

        .distance-fill.danger {
            background: linear-gradient(135deg, #dc3545 0%, #c82333 100%);
        }

        /* Settings */
        .settings {
            background: linear-gradient(135deg, #f8f9fa 0%, #e9ecef 100%);
            padding: 25px;
            border-radius: 15px;
        }

        .ip-input-group {
            display: flex;
            gap: 10px;
            align-items: center;
            flex-wrap: wrap;
            margin-bottom: 15px;
        }

        .ip-input {
            flex: 1;
            min-width: 200px;
            padding: 12px;
            border: 2px solid #dee2e6;
            border-radius: 8px;
            font-size: 1em;
        }

        .update-btn, .reconnect-btn {
            padding: 12px 24px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-weight: bold;
            transition: all 0.3s ease;
        }

        .message {
            margin-top: 15px;
            padding: 12px;
            border-radius: 8px;
            display: none;
        }

        .message.success {
            background: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }

        .message.error {
            background: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }

        .link-section {
            text-align: center;
            margin-top: 20px;
            padding-top: 20px;
            border-top: 2px solid #e9ecef;
        }

        .link-btn {
            display: inline-block;
            padding: 10px 20px;
            background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
            color: white;
            text-decoration: none;
            border-radius: 8px;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🎮 Robot Controller</h1>
            <p>Control your robot remotely</p>
        </div>

        <div class="status-bar">
            <div class="status-item">
                <span class="status-dot" id="pi-status"></span>
                <span>Pi: <strong id="pi-status-text">Checking...</strong></span>
            </div>
            <div class="status-item">
                <span>Mode: <strong id="current-mode">manual</strong></span>
            </div>
            <div class="status-item">
                <span>IP: <strong id="current-pi-ip">192.168.16.154</strong></span>
            </div>
        </div>

        <div class="control-section">
            <h2 class="section-title">🎯 Control Mode</h2>
            <div class="mode-buttons">
                <button class="mode-btn active" data-mode="manual" onclick="switchMode('manual')">
                    🕹️ Manual Control
                </button>
                <button class="mode-btn" data-mode="follow" onclick="switchMode('follow')">
                    👁️ Object Following
                </button>
            </div>
        </div>

        <!-- Manual Movement Controls -->
        <div class="control-section" id="manual-controls">
            <h2 class="section-title">🚗 Movement Controls</h2>
            <div class="movement-controls">
                <button class="move-btn forward" data-direction="forward">
                    ⬆️<br>Forward
                </button>
                <button class="move-btn left" data-direction="left">
                    ⬅️<br>Left
                </button>
                <button class="move-btn stop" data-direction="stop">
                    🛑<br>STOP
                </button>
                <button class="move-btn right" data-direction="right">
                    ➡️<br>Right
                </button>
                <button class="move-btn backward" data-direction="backward">
                    ⬇️<br>Back
                </button>
            </div>
        </div>

        <!-- Follow Mode Display -->
        <div class="follow-display" id="follow-display">
            <h3>👁️ Object Following Active</h3>
            <div class="tracking-info">
                <p>Status: <strong id="tracking-status">Searching...</strong></p>
                <p>Centering Error: <strong id="tracking-error">0</strong> pixels</p>
                <p>Distance: <strong id="tracking-distance">---</strong> cm</p>
                <p>Action: <strong id="tracking-action">Waiting...</strong></p>
            </div>
            <div class="distance-indicator">
                <h4>📏 Object Distance</h4>
                <div class="distance-bar">
                    <div class="distance-fill" id="distance-fill" style="width: 0%"></div>
                </div>
                <p style="margin-top: 10px; font-size: 0.9em; color: #666;">
                    Safe: 30-50cm | Too close: &lt;30cm | Too far: &gt;50cm
                </p>
            </div>
        </div>

        <div class="control-section">
            <h2 class="section-title">⚙️ Settings</h2>
            <div class="settings">
                <div class="ip-input-group">
                    <label>Pi IP:</label>
                    <input type="text" id="pi-ip" class="ip-input" value="192.168.16.154">
                    <button class="update-btn" onclick="updatePiIP()">Update IP</button>
                </div>
                <div style="text-align: center;">
                    <button class="reconnect-btn" onclick="reconnectToPi()">🔄 Reconnect</button>
                </div>
                <div class="message" id="settings-message"></div>
            </div>
        </div>

        <div class="link-section">
            <p>Need to view the camera feed?</p>
            <a href="http://localhost:5001" class="link-btn" target="_blank">
                📹 Open Video Stream
            </a>
        </div>
    </div>

    <script>
        let currentMode = 'manual';

        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            console.log('Controller interface loaded');
            updateStatus();
            setInterval(updateStatus, 3000);
            setupMovementButtons();
            setInterval(updateTrackingData, 500);  // Update tracking data faster for smoother distance display
        });

        function setupMovementButtons() {
            const moveButtons = document.querySelectorAll('.move-btn');
            moveButtons.forEach(button => {
                const direction = button.dataset.direction;
                let isPressed = false;
                let interval = null;

                const startMoving = () => {
                    if (currentMode !== 'manual' || isPressed) return;
                    isPressed = true;
                    
                    console.log('Start moving:', direction);
                    if (direction !== 'stop') {
                        sendMoveStart(direction);
                        interval = setInterval(() => sendMoveStart(direction), 100);
                    } else {
                        sendMoveOnce('stop');
                    }
                };

                const stopMoving = () => {
                    if (!isPressed) return;
                    isPressed = false;
                    
                    console.log('Stop moving:', direction);
                    if (interval) {
                        clearInterval(interval);
                        interval = null;
                    }
                    if (direction !== 'stop') {
                        sendMoveStop();
                    }
                };

                // Mouse events
                button.addEventListener('mousedown', startMoving);
                button.addEventListener('mouseup', stopMoving);
                button.addEventListener('mouseleave', stopMoving);
                
                // Touch events
                button.addEventListener('touchstart', e => {
                    e.preventDefault();
                    startMoving();
                });
                button.addEventListener('touchend', e => {
                    e.preventDefault();
                    stopMoving();
                });
            });
        }

        function sendMoveStart(direction) {
            fetch('/move_start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ direction })
            }).catch(err => console.error('Move start error:', err));
        }

        function sendMoveOnce(direction) {
            fetch('/move', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ direction })
            }).catch(err => console.error('Move error:', err));
        }
        
        function sendMoveStop() {
            fetch('/move_stop', {
                method: 'POST'
            }).catch(err => console.error('Move stop error:', err));
        }

        async function switchMode(mode) {
            console.log('Switching to mode:', mode);
            try {
                const response = await fetch('/set_mode', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ mode: mode })
                });

                const data = await response.json();
                console.log('Mode switch response:', data);
                
                if (data.status === 'success') {
                    currentMode = mode;
                    updateModeUI();
                    updateControlVisibility();
                    showMessage('Mode switched to ' + mode, 'success');
                } else {
                    showMessage('Failed to switch mode: ' + data.message, 'error');
                }
            } catch (error) {
                console.error('Mode switch error:', error);
                showMessage('Error switching mode', 'error');
            }
        }

        function updateModeUI() {
            const modeButtons = document.querySelectorAll('.mode-btn');
            modeButtons.forEach(button => {
                if (button.dataset.mode === currentMode) {
                    button.classList.add('active');
                } else {
                    button.classList.remove('active');
                }
            });
            document.getElementById('current-mode').textContent = currentMode;
        }

        function updateControlVisibility() {
            document.getElementById('manual-controls').style.display = 'none';
            document.getElementById('follow-display').classList.remove('active');
            
            switch(currentMode) {
                case 'manual':
                    document.getElementById('manual-controls').style.display = 'block';
                    break;
                case 'follow':
                    document.getElementById('follow-display').classList.add('active');
                    break;
            }
        }

        async function updateTrackingData() {
            if (currentMode === 'follow') {
                try {
                    const response = await fetch('/tracking_data');
                    const data = await response.json();
                    
                    document.getElementById('tracking-status').textContent = data.status;
                    document.getElementById('tracking-error').textContent = data.error;
                    document.getElementById('tracking-action').textContent = data.action;
                    
                    // Update distance display
                    const distance = data.distance || 999;
                    document.getElementById('tracking-distance').textContent = 
                        distance < 999 ? distance.toFixed(1) : '---';
                    
                    // Update distance bar
                    updateDistanceBar(distance);
                    
                } catch (error) {
                    console.error('Tracking data error:', error);
                }
            }
        }

        function updateDistanceBar(distance) {
            const fill = document.getElementById('distance-fill');
            if (distance >= 999) {
                fill.style.width = '0%';
                fill.className = 'distance-fill';
                return;
            }
            
            // Map distance to percentage (0-100cm range)
            const maxDist = 100;
            const percent = Math.min(100, (distance / maxDist) * 100);
            fill.style.width = percent + '%';
            
            // Color based on distance
            if (distance < 20) {
                fill.className = 'distance-fill danger';  // Too close - red
            } else if (distance < 30) {
                fill.className = 'distance-fill warning';  // Close - yellow
            } else if (distance <= 50) {
                fill.className = 'distance-fill';  // Good - green
            } else {
                fill.className = 'distance-fill warning';  // Too far - yellow
            }
        }

        async function updateStatus() {
            try {
                const response = await fetch('/status');
                const data = await response.json();
                
                currentMode = data.mode;
                updateModeUI();
                updateControlVisibility();
                
                document.getElementById('current-pi-ip').textContent = data.pi_ip;
                document.getElementById('pi-ip').value = data.pi_ip;
                
                const statusDot = document.getElementById('pi-status');
                const statusText = document.getElementById('pi-status-text');
                
                if (data.pi_status && data.pi_status.connected) {
                    statusDot.classList.add('connected');
                    statusText.textContent = 'Connected';
                } else {
                    statusDot.classList.remove('connected');
                    statusText.textContent = 'Disconnected';
                }
            } catch (error) {
                console.error('Status update error:', error);
            }
        }

        async function updatePiIP() {
            const newIP = document.getElementById('pi-ip').value.trim();
            
            if (!newIP) {
                showSettingsMessage('Please enter a valid IP address', 'error');
                return;
            }

            try {
                const response = await fetch('/set_pi_ip', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ ip: newIP })
                });

                const data = await response.json();
                
                if (data.status === 'success' || data.status === 'warning') {
                    showSettingsMessage(data.message, data.status);
                    updateStatus();
                } else {
                    showSettingsMessage(data.message, 'error');
                }
            } catch (error) {
                showSettingsMessage('Error updating IP', 'error');
            }
        }

        async function reconnectToPi() {
            try {
                const response = await fetch('/reconnect', {
                    method: 'POST'
                });

                const data = await response.json();
                
                if (data.status === 'success') {
                    showSettingsMessage('Reconnected successfully', 'success');
                } else {
                    showSettingsMessage('Failed to reconnect', 'error');
                }
                updateStatus();
            } catch (error) {
                showSettingsMessage('Error reconnecting', 'error');
            }
        }

        function showMessage(message, type) {
            console.log(`${type}: ${message}`);
        }

        function showSettingsMessage(message, type) {
            const messageEl = document.getElementById('settings-message');
            messageEl.textContent = message;
            messageEl.className = `message ${type}`;
            messageEl.style.display = 'block';
            
            setTimeout(() => {
                messageEl.style.display = 'none';
            }, 3000);
        }

        // Keyboard controls
        document.addEventListener('keydown', function(event) {
            if (currentMode !== 'manual') return;
            
            let direction = null;
            switch(event.key) {
                case 'ArrowUp':
                case 'w':
                case 'W':
                    direction = 'forward';
                    break;
                case 'ArrowDown':
                case 's':
                case 'S':
                    direction = 'backward';
                    break;
                case 'ArrowLeft':
                case 'a':
                case 'A':
                    direction = 'left';
                    break;
                case 'ArrowRight':
                case 'd':
                case 'D':
                    direction = 'right';
                    break;
                case ' ':
                    direction = 'stop';
                    break;
            }
            
            if (direction) {
                event.preventDefault();
                sendMoveOnce(direction);
            }
        });
    </script>
</body>
</html>
'''