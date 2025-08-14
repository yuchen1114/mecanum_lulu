'''
Robot Controller Server
Dedicated server for robot control (manual, follow, gyro modes)
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

# Tracking data cache for follow mode
tracking_data = {'status': 'Searching...', 'error': 0, 'action': 'Waiting...'}
data_lock = threading.Lock()

# HTML template embedded in Python file to avoid template path issues
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
            min-width: 100px;
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

        /* Gyro Control */
        .gyro-control {
            text-align: center;
            padding: 25px;
            background: linear-gradient(135deg, #f8f9fa 0%, #e9ecef 100%);
            border-radius: 15px;
            margin-bottom: 20px;
            display: none;
        }

        .gyro-control.active {
            display: block;
        }

        .gyro-indicator {
            width: 250px;
            height: 250px;
            margin: 20px auto;
            position: relative;
            background: linear-gradient(135deg, #fff 0%, #f8f9fa 100%);
            border-radius: 50%;
            border: 4px solid #667eea;
            box-shadow: inset 0 0 30px rgba(0, 0, 0, 0.1);
        }

        .gyro-dot {
            width: 24px;
            height: 24px;
            background: linear-gradient(135deg, #dc3545 0%, #c82333 100%);
            border-radius: 50%;
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            transition: all 0.1s ease;
            box-shadow: 0 3px 10px rgba(220, 53, 69, 0.5);
        }

        .gyro-values {
            margin-top: 20px;
            font-family: 'Courier New', monospace;
            background: white;
            padding: 15px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
        }

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
                    🕹️ Manual
                </button>
                <button class="mode-btn" data-mode="follow" onclick="switchMode('follow')">
                    👁️ Follow
                </button>
                <button class="mode-btn" data-mode="gyro" onclick="switchMode('gyro')">
                    📱 Gyro
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

        <!-- Gyro Control -->
        <div class="gyro-control" id="gyro-controls">
            <h3>📱 Tilt your phone to control the robot</h3>
            <div class="gyro-indicator">
                <div class="gyro-dot" id="gyro-dot"></div>
            </div>
            <div class="gyro-values">
                <div>Forward/Back: <span id="gyro-pitch">0</span>°</div>
                <div>Left/Right: <span id="gyro-roll">0</span>°</div>
                <div>Command: <span id="gyro-command">STOP</span></div>
            </div>
            <button class="update-btn" onclick="requestGyroPermission()">Enable Gyro Control</button>
        </div>

        <!-- Follow Mode Display -->
        <div class="follow-display" id="follow-display">
            <h3>👁️ Object Following Active</h3>
            <div class="tracking-info">
                <p>Status: <strong id="tracking-status">Searching...</strong></p>
                <p>Error: <strong id="tracking-error">0</strong> pixels</p>
                <p>Action: <strong id="tracking-action">Waiting...</strong></p>
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
        let gyroEnabled = false;

        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            console.log('Controller interface loaded');
            updateStatus();
            setInterval(updateStatus, 3000);
            setupMovementButtons();
            setInterval(updateTrackingData, 1000);
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
                    
                    if (mode === 'gyro' && !gyroEnabled) {
                        requestGyroPermission();
                    } else if (mode !== 'gyro' && gyroEnabled) {
                        stopGyroControl();
                    }
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
            document.getElementById('gyro-controls').classList.remove('active');
            document.getElementById('follow-display').classList.remove('active');
            
            switch(currentMode) {
                case 'manual':
                    document.getElementById('manual-controls').style.display = 'block';
                    break;
                case 'gyro':
                    document.getElementById('gyro-controls').classList.add('active');
                    break;
                case 'follow':
                    document.getElementById('follow-display').classList.add('active');
                    break;
            }
        }

        async function requestGyroPermission() {
            if (typeof DeviceOrientationEvent !== 'undefined' && 
                typeof DeviceOrientationEvent.requestPermission === 'function') {
                try {
                    const permission = await DeviceOrientationEvent.requestPermission();
                    if (permission === 'granted') {
                        startGyroControl();
                    }
                } catch (error) {
                    console.error('Gyro permission error:', error);
                }
            } else {
                startGyroControl();
            }
        }

        function startGyroControl() {
            gyroEnabled = true;
            window.addEventListener('deviceorientation', handleGyroData);
            showMessage('Gyro control enabled', 'success');
        }

        function stopGyroControl() {
            gyroEnabled = false;
            window.removeEventListener('deviceorientation', handleGyroData);
        }

        function handleGyroData(event) {
            if (!gyroEnabled || currentMode !== 'gyro') return;
            
            const pitch = event.beta;
            const roll = event.gamma;
            
            document.getElementById('gyro-pitch').textContent = pitch.toFixed(1);
            document.getElementById('gyro-roll').textContent = roll.toFixed(1);
            
            const gyroDot = document.getElementById('gyro-dot');
            const maxTilt = 30;
            const x = Math.max(-100, Math.min(100, (roll / maxTilt) * 100));
            const y = Math.max(-100, Math.min(100, (-pitch / maxTilt) * 100));
            gyroDot.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
            
            let command = 'stop';
            const threshold = 10;
            
            if (Math.abs(pitch) > Math.abs(roll)) {
                if (pitch > threshold) command = 'backward';
                else if (pitch < -threshold) command = 'forward';
            } else {
                if (roll > threshold) command = 'right';
                else if (roll < -threshold) command = 'left';
            }
            
            document.getElementById('gyro-command').textContent = command.toUpperCase();
            
            fetch('/gyro_control', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ pitch, roll, command })
            });
        }

        async function updateTrackingData() {
            if (currentMode === 'follow') {
                try {
                    const response = await fetch('/tracking_data');
                    const data = await response.json();
                    document.getElementById('tracking-status').textContent = data.status;
                    document.getElementById('tracking-error').textContent = data.error;
                    document.getElementById('tracking-action').textContent = data.action;
                } catch (error) {
                    console.error('Tracking data error:', error);
                }
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

def initialize_robot_client():
    """Initialize the robot client with callbacks"""
    global robot_client, robot_status
    
    try:
        with client_lock:
            if robot_client:
                try:
                    robot_client.disconnect()
                except:
                    pass
            
            robot_client = socket_client.RobotSocketClient(pi_ip, pi_port)
            
            # Set up callbacks
            def on_response(command, response, success):
                print(f"📥 Pi response to '{command}': {response}")
                
                # Parse tracking data from responses
                if "TRACKING_DATA:" in response:
                    parse_tracking_data(response)
            
            def on_connection(connected, message):
                global robot_status
                robot_status = "connected" if connected else "disconnected"
                print(f"🔗 Connection: {message}")
            
            robot_client.set_response_callback(on_response)
            robot_client.set_connection_callback(on_connection)
            
            # Try to connect
            if robot_client.connect():
                print(f"✅ Connected to Pi at {pi_ip}:{pi_port}")
                return True
            else:
                print(f"❌ Failed to connect to Pi at {pi_ip}:{pi_port}")
                return False
    except Exception as e:
        print(f"❌ Error initializing robot client: {e}")
        return False

def parse_tracking_data(response):
    """Parse tracking data from response"""
    global tracking_data
    try:
        # Expected format: TRACKING_DATA:status=XXX,error=XX,action=XXX
        data_part = response.split("TRACKING_DATA:")[1]
        parts = data_part.split(",")
        
        with data_lock:
            for part in parts:
                if "=" in part:
                    key, value = part.split("=", 1)
                    key = key.strip()
                    value = value.strip()
                    if key == 'error':
                        try:
                            tracking_data[key] = int(value)
                        except:
                            tracking_data[key] = 0
                    else:
                        tracking_data[key] = value
    except Exception as e:
        print(f"Error parsing tracking data: {e}")

@app.route('/')
def index():
    return render_template_string(CONTROLLER_HTML)

@app.route('/set_mode', methods=['POST'])
def set_mode():
    global current_mode, robot_client
    
    try:
        data = request.get_json()
        mode = data.get('mode', 'manual')
        
        print(f"Mode change request: {mode}")
        
        valid_modes = ['manual', 'follow', 'gyro']
        if mode not in valid_modes:
            return jsonify({
                'status': 'error',
                'message': 'Invalid mode'
            }), 400
        
        current_mode = mode
        
        # Send mode change command to Pi if connected
        if robot_client and robot_client.is_connected():
            success = robot_client.send_command(f"MODE:{mode}")
            print(f"Mode command sent: {success}")
            
            return jsonify({
                'status': 'success' if success else 'error',
                'mode': current_mode,
                'message': f'Mode switched to {mode}' if success else 'Failed to send command to Pi',
                'connected': True
            })
        else:
            # Still change mode locally even if not connected
            return jsonify({
                'status': 'warning',
                'mode': current_mode,
                'message': f'Mode set to {mode} (Pi not connected)',
                'connected': False
            })
    except Exception as e:
        print(f"Error in set_mode: {e}")
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500

@app.route('/move', methods=['POST'])
def move():
    global current_mode, robot_client
    data = request.get_json()
    direction = data.get('direction', '')
    
    if current_mode not in ['manual', 'gyro']:
        return jsonify({
            'status': 'error',
            'message': 'Movement commands only available in manual or gyro mode'
        }), 400
    
    valid_directions = ['forward', 'backward', 'left', 'right', 'stop']
    if direction not in valid_directions:
        return jsonify({
            'status': 'error',
            'message': 'Invalid direction'
        }), 400
    
    # Send movement command to Pi
    if robot_client and robot_client.is_connected():
        success = robot_client.send_command(f"MOVE:{direction}")
        
        return jsonify({
            'status': 'success' if success else 'error',
            'direction': direction,
            'message': f'Moving {direction}' if success else 'Failed to send command to Pi',
            'connected': robot_client.is_connected()
        })
    else:
        return jsonify({
            'status': 'error',
            'message': 'Not connected to Pi',
            'connected': False
        }), 500

@app.route('/move_start', methods=['POST'])
def move_start():
    """Start continuous movement (hold-to-move)"""
    global current_mode, robot_client
    data = request.get_json()
    direction = data.get('direction', '')
    
    if current_mode != 'manual':
        return jsonify({
            'status': 'error',
            'message': 'Movement commands only available in manual mode'
        }), 400
    
    valid_directions = ['forward', 'backward', 'left', 'right']
    if direction not in valid_directions:
        return jsonify({
            'status': 'error',
            'message': 'Invalid direction'
        }), 400
    
    # Start continuous movement
    if robot_client and robot_client.is_connected():
        robot_client.start_continuous_command(f"MOVE:{direction}", 0.1)
        
        return jsonify({
            'status': 'success',
            'direction': direction,
            'message': f'Started continuous movement: {direction}',
            'connected': robot_client.is_connected()
        })
    else:
        return jsonify({
            'status': 'error',
            'message': 'Not connected to Pi',
            'connected': False
        }), 500

@app.route('/move_stop', methods=['POST'])
def move_stop():
    """Stop continuous movement"""
    global robot_client
    
    if robot_client and robot_client.is_connected():
        robot_client.stop_continuous_command()
        # Send explicit stop command
        robot_client.send_command("MOVE:stop")
        
        return jsonify({
            'status': 'success',
            'message': 'Stopped continuous movement',
            'connected': robot_client.is_connected()
        })
    else:
        return jsonify({
            'status': 'error',
            'message': 'Not connected to Pi',
            'connected': False
        }), 500

@app.route('/gyro_control', methods=['POST'])
def gyro_control():
    """Handle gyro control data"""
    global current_mode, robot_client
    
    if current_mode != 'gyro':
        return jsonify({
            'status': 'error',
            'message': 'Not in gyro mode'
        }), 400
    
    data = request.get_json()
    pitch = data.get('pitch', 0)
    roll = data.get('roll', 0)
    command = data.get('command', 'stop')
    
    # Send gyro data to Pi
    if robot_client and robot_client.is_connected():
        gyro_command = f"GYRO:pitch={pitch:.1f},roll={roll:.1f},cmd={command}"
        success = robot_client.send_command(gyro_command)
        
        return jsonify({
            'status': 'success' if success else 'error',
            'connected': robot_client.is_connected()
        })
    else:
        return jsonify({
            'status': 'error',
            'message': 'Not connected to Pi',
            'connected': False
        }), 500

@app.route('/tracking_data')
def get_tracking_data():
    """Get object tracking data"""
    # Request fresh tracking data
    if robot_client and robot_client.is_connected():
        robot_client.send_command("GET_TRACKING")
    
    with data_lock:
        return jsonify(tracking_data)

@app.route('/status')
def status():
    global robot_client, current_mode
    
    # Request fresh status from Pi
    if robot_client and robot_client.is_connected():
        robot_client.send_command("STATUS")
    
    # Check connection to Pi
    pi_connected = robot_client.is_connected() if robot_client else False
    
    return jsonify({
        'mode': current_mode,
        'pi_status': {
            'connected': pi_connected,
            'status': 'online' if pi_connected else 'offline',
            'timestamp': time.time()
        },
        'pi_ip': pi_ip,
        'pi_port': pi_port
    })

@app.route('/set_pi_ip', methods=['POST'])
def set_pi_ip():
    global pi_ip, robot_client
    data = request.get_json()
    new_ip = data.get('ip', '')
    
    if new_ip:
        pi_ip = new_ip
        # Reconnect with new IP
        success = initialize_robot_client()
        
        return jsonify({
            'status': 'success' if success else 'warning',
            'message': f'Pi IP updated to {pi_ip}' + (' and connected' if success else ' but connection failed'),
            'connected': success
        })
    else:
        return jsonify({
            'status': 'error',
            'message': 'Invalid IP address'
        }), 400

@app.route('/reconnect', methods=['POST'])
def reconnect():
    """Manually reconnect to Pi"""
    success = initialize_robot_client()
    
    return jsonify({
        'status': 'success' if success else 'error',
        'message': 'Reconnected to Pi' if success else 'Failed to reconnect',
        'connected': success
    })

if __name__ == '__main__':
    print("🤖 Robot Controller Server Starting...")
    print(f"📡 Pi IP: {pi_ip}:{pi_port}")
    print("🌐 Web interface will be available at: http://localhost:5000")
    print("🎮 Control modes: manual, gyro, follow")
    print("🔄 Attempting initial connection...")
    print("=" * 50)
    
    # Try initial connection
    initialize_robot_client()
    
    try:
        app.run(debug=True, host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        if robot_client:
            robot_client.disconnect()
    finally:
        print("✅ Controller server stopped")