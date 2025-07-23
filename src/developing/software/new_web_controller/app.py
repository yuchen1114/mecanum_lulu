'''
Enhanced Robot Controller Flask App
Runs on PC with video streaming, gyro control, and sensor data
'''
from flask import Flask, render_template, request, jsonify, Response
import socket_client
import threading
import time
import cv2
import numpy as np
import base64
from collections import deque
import io
from PIL import Image

app = Flask(__name__)

# Global variables to track robot state
current_mode = "manual"
robot_status = "disconnected"
pi_ip = "192.168.16.154"  # Default Pi IP
pi_port = 8888

# Long-lived socket client
robot_client = None
client_lock = threading.Lock()

# Video streaming variables
video_frame = None
frame_lock = threading.Lock()
video_client = None

# Sensor data cache
sensor_data = {'left': 0, 'center': 0, 'right': 0, 'status': 'Initializing...'}
tracking_data = {'status': 'Searching...', 'error': 0, 'action': 'Waiting...'}
data_lock = threading.Lock()

def initialize_robot_client():
    """Initialize the robot client with callbacks"""
    global robot_client
    
    with client_lock:
        if robot_client:
            robot_client.disconnect()
        
        robot_client = socket_client.RobotSocketClient(pi_ip, pi_port)
        
        # Set up callbacks
        def on_response(command, response, success):
            print(f"📥 Pi response to '{command}': {response}")
            
            # Parse sensor data from responses
            if "SENSOR_DATA:" in response:
                parse_sensor_data(response)
            elif "TRACKING_DATA:" in response:
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
            # Start video stream client
            start_video_client()
            return True
        else:
            print(f"❌ Failed to connect to Pi at {pi_ip}:{pi_port}")
            return False

def parse_sensor_data(response):
    """Parse sensor data from response"""
    global sensor_data
    try:
        # Expected format: SENSOR_DATA:left=XX,center=XX,right=XX,status=XXX
        data_part = response.split("SENSOR_DATA:")[1]
        parts = data_part.split(",")
        
        with data_lock:
            for part in parts:
                if "=" in part:
                    key, value = part.split("=")
                    if key in ['left', 'center', 'right']:
                        sensor_data[key] = float(value)
                    elif key == 'status':
                        sensor_data['status'] = value
    except Exception as e:
        print(f"Error parsing sensor data: {e}")

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
                    key, value = part.split("=")
                    if key == 'error':
                        tracking_data[key] = int(value)
                    else:
                        tracking_data[key] = value
    except Exception as e:
        print(f"Error parsing tracking data: {e}")

class VideoStreamClient(threading.Thread):
    """Separate client for video streaming"""
    def __init__(self, host, port):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.running = False
        self.socket = None
        
    def run(self):
        """Connect and receive video frames"""
        self.running = True
        
        while self.running:
            try:
                # Connect to video stream
                self.socket = socket_client.socket.socket(socket_client.socket.AF_INET, socket_client.socket.SOCK_STREAM)
                self.socket.connect((self.host, self.port + 1))  # Video on port 8889
                print(f"📹 Connected to video stream at {self.host}:{self.port + 1}")
                
                # Receive frames
                buffer = b""
                while self.running:
                    data = self.socket.recv(4096)
                    if not data:
                        break
                    
                    buffer += data
                    
                    # Look for frame delimiter
                    while b'\xff\xd9' in buffer:
                        # Find JPEG end marker
                        end_idx = buffer.find(b'\xff\xd9') + 2
                        
                        # Extract frame
                        frame_data = buffer[:end_idx]
                        buffer = buffer[end_idx:]
                        
                        # Decode and store frame
                        try:
                            # Convert bytes to numpy array
                            nparr = np.frombuffer(frame_data, np.uint8)
                            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                            
                            if frame is not None:
                                global video_frame
                                with frame_lock:
                                    video_frame = frame
                        except Exception as e:
                            print(f"Error decoding frame: {e}")
                
            except Exception as e:
                print(f"Video stream error: {e}")
                time.sleep(5)  # Wait before reconnecting
            
            finally:
                if self.socket:
                    self.socket.close()
    
    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()

def start_video_client():
    """Start the video stream client"""
    global video_client
    
    if video_client:
        video_client.stop()
    
    video_client = VideoStreamClient(pi_ip, pi_port)
    video_client.start()

def generate_frames():
    """Generate frames for video streaming"""
    while True:
        with frame_lock:
            if video_frame is None:
                # Generate placeholder frame
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "No Video Signal", (200, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            else:
                frame = video_frame.copy()
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        if ret:
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.033)  # ~30 FPS

# Initialize client on startup
initialize_robot_client()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/set_mode', methods=['POST'])
def set_mode():
    global current_mode, robot_client
    data = request.get_json()
    mode = data.get('mode', 'manual')
    
    valid_modes = ['manual', 'auto', 'follow', 'gyro']
    if mode in valid_modes:
        current_mode = mode
        
        # Send mode change command to Pi
        if robot_client and robot_client.is_connected():
            success = robot_client.send_command(f"MODE:{mode}")
            
            return jsonify({
                'status': 'success' if success else 'error',
                'mode': current_mode,
                'message': f'Mode switched to {mode}' if success else 'Failed to send command to Pi',
                'connected': robot_client.is_connected()
            })
        else:
            return jsonify({
                'status': 'error',
                'message': 'Not connected to Pi',
                'connected': False
            }), 500
    else:
        return jsonify({
            'status': 'error',
            'message': 'Invalid mode'
        }), 400

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

@app.route('/sensor_data')
def get_sensor_data():
    """Get ultrasonic sensor data"""
    with data_lock:
        return jsonify(sensor_data)

@app.route('/tracking_data')
def get_tracking_data():
    """Get object tracking data"""
    with data_lock:
        return jsonify(tracking_data)

@app.route('/status')
def status():
    global robot_client
    
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
    print("🤖 Enhanced Robot Controller Web App Starting...")
    print(f"📡 Pi IP: {pi_ip}:{pi_port}")
    print("🌐 Web interface will be available at: http://localhost:5000")
    print("📹 Video streaming enabled")
    print("📱 Gyro control supported")
    print("🔄 Long-lived connection enabled")
    print("=" * 50)
    
    try:
        app.run(debug=True, host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        if robot_client:
            robot_client.disconnect()
        if video_client:
            video_client.stop()
    finally:
        print("✅ Web app stopped")