'''
Video Streaming Server
Dedicated server for robot camera video streaming
Runs on port 5001
'''
from flask import Flask, render_template, Response
import cv2
import numpy as np
import threading
import time
import socket

app = Flask(__name__)

# Configuration
PI_IP = "172.20.10.2"  # Default Pi IP
VIDEO_PORT = 8889  # Pi's video streaming port

# Video streaming variables
video_frame = None
frame_lock = threading.Lock()
video_client = None
connection_status = "disconnected"

class VideoStreamClient(threading.Thread):
    """Client for receiving video stream from Pi"""
    def __init__(self, host, port):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.running = False
        self.socket = None
        
    def run(self):
        """Connect and receive video frames"""
        global connection_status
        self.running = True
        
        while self.running:
            try:
                # Connect to video stream
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5)
                self.socket.connect((self.host, self.port))
                connection_status = "connected"
                print(f"📹 Connected to video stream at {self.host}:{self.port}")
                
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
                connection_status = "disconnected"
                print(f"Video stream error: {e}")
                time.sleep(5)  # Wait before reconnecting
            
            finally:
                if self.socket:
                    self.socket.close()
                connection_status = "disconnected"
    
    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()

def generate_frames():
    """Generate frames for video streaming"""
    while True:
        with frame_lock:
            if video_frame is None:
                # Generate placeholder frame
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "No Video Signal", (200, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(frame, f"Status: {connection_status}", (200, 280), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            else:
                frame = video_frame.copy()
                
                # Add timestamp overlay
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(frame, timestamp, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Add connection status
                status_color = (0, 255, 0) if connection_status == "connected" else (0, 0, 255)
                cv2.circle(frame, (600, 30), 10, status_color, -1)
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        if ret:
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.033)  # ~30 FPS

def start_video_client():
    """Start the video stream client"""
    global video_client
    
    if video_client:
        video_client.stop()
    
    video_client = VideoStreamClient(PI_IP, VIDEO_PORT)
    video_client.start()

@app.route('/')
def index():
    return render_template('video.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    """Get connection status"""
    return {
        'connection': connection_status,
        'pi_ip': PI_IP,
        'port': VIDEO_PORT,
        'timestamp': time.time()
    }

if __name__ == '__main__':
    print("📹 Video Streaming Server Starting...")
    print(f"📡 Connecting to Pi at {PI_IP}:{VIDEO_PORT}")
    print("🌐 Web interface will be available at: http://localhost:5001")
    print("=" * 50)
    
    # Start video client
    start_video_client()
    
    try:
        app.run(debug=False, host='0.0.0.0', port=5001, threaded=True)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        if video_client:
            video_client.stop()
    finally:
        print("✅ Video server stopped")