from flask import Flask, render_template, request, jsonify
import socket_client
import threading
import time

app = Flask(__name__)

# Global variables to track robot state
current_mode = "manual"
robot_status = "disconnected"
pi_ip = "192.168.1.100"  # Default Pi IP - change this to your Pi's IP
pi_port = 8888

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/set_mode', methods=['POST'])
def set_mode():
    global current_mode
    data = request.get_json()
    mode = data.get('mode', 'manual')
    
    if mode in ['manual', 'auto', 'follow']:
        current_mode = mode
        # Send mode change command to Pi
        response = socket_client.send_command(pi_ip, pi_port, f"MODE:{mode}")
        
        return jsonify({
            'status': 'success',
            'mode': current_mode,
            'message': f'Mode switched to {mode}',
            'pi_response': response
        })
    else:
        return jsonify({
            'status': 'error',
            'message': 'Invalid mode'
        }), 400

@app.route('/move', methods=['POST'])
def move():
    global current_mode
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
    response = socket_client.send_command(pi_ip, pi_port, f"MOVE:{direction}")
    
    return jsonify({
        'status': 'success',
        'direction': direction,
        'message': f'Moving {direction}',
        'pi_response': response
    })

@app.route('/status')
def status():
    # Check connection to Pi
    pi_status = socket_client.check_connection(pi_ip, pi_port)
    
    return jsonify({
        'mode': current_mode,
        'pi_status': pi_status,
        'pi_ip': pi_ip,
        'pi_port': pi_port
    })

@app.route('/set_pi_ip', methods=['POST'])
def set_pi_ip():
    global pi_ip
    data = request.get_json()
    new_ip = data.get('ip', '')
    
    if new_ip:
        pi_ip = new_ip
        return jsonify({
            'status': 'success',
            'message': f'Pi IP updated to {pi_ip}'
        })
    else:
        return jsonify({
            'status': 'error',
            'message': 'Invalid IP address'
        }), 400

if __name__ == '__main__':
    print("🤖 Robot Controller Web App Starting...")
    print(f"📡 Default Pi IP: {pi_ip}:{pi_port}")
    print("🌐 Web interface will be available at: http://localhost:5000")
    print("=" * 50)
    
    app.run(debug=True, host='0.0.0.0', port=5000)