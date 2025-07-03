from flask import Flask, render_template, request, jsonify
import socket_client
import threading
import time

app = Flask(__name__)

# Global variables to track robot state
current_mode = "manual"
robot_status = "disconnected"
pi_ip = "192.168.16.154"  # Default Pi IP - change this to your Pi's IP
pi_port = 8888

# Long-lived socket client
robot_client = None
client_lock = threading.Lock()

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

# Initialize client on startup
initialize_robot_client()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/set_mode', methods=['POST'])
def set_mode():
    global current_mode, robot_client
    data = request.get_json()
    mode = data.get('mode', 'manual')
    
    if mode in ['manual', 'auto', 'follow']:
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
    
    if current_mode != 'manual':
        return jsonify({
            'status': 'error',
            'message': 'Movement commands only available in manual mode'
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

@app.route('/status')
def status():
    global robot_client
    
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

@app.route('/ping', methods=['POST'])
def ping():
    """Send ping to Pi"""
    global robot_client
    
    if robot_client and robot_client.is_connected():
        success = robot_client.ping()
        
        return jsonify({
            'status': 'success' if success else 'error',
            'message': 'Ping sent' if success else 'Failed to send ping',
            'connected': robot_client.is_connected()
        })
    else:
        return jsonify({
            'status': 'error',
            'message': 'Not connected to Pi',
            'connected': False
        }), 500

if __name__ == '__main__':
    print("🤖 Robot Controller Web App Starting...")
    print(f"📡 Pi IP: {pi_ip}:{pi_port}")
    print("🌐 Web interface will be available at: http://localhost:5000")
    print("🔄 Long-lived connection enabled")
    print("👆 Hold-to-move commands supported")
    print("=" * 50)
    
    try:
        app.run(debug=True, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        if robot_client:
            robot_client.disconnect()
    finally:
        print("✅ Web app stopped")