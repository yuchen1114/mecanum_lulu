import socket
import time
import json

def send_command(pi_ip, pi_port, command, timeout=5):
    """
    Send a command to the Raspberry Pi and return the response
    
    Args:
        pi_ip (str): Raspberry Pi IP address
        pi_port (int): Port number
        command (str): Command to send
        timeout (int): Connection timeout in seconds
    
    Returns:
        dict: Response from Pi or error message
    """
    try:
        # Create socket connection
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        
        # Connect to Pi
        sock.connect((pi_ip, pi_port))
        
        # Send command
        message = command.encode('utf-8')
        sock.sendall(message)
        
        # Receive response
        response = sock.recv(1024).decode('utf-8')
        sock.close()
        
        return {
            'success': True,
            'response': response,
            'timestamp': time.time()
        }
        
    except socket.timeout:
        return {
            'success': False,
            'error': 'Connection timeout',
            'timestamp': time.time()
        }
    except ConnectionRefusedError:
        return {
            'success': False,
            'error': 'Connection refused - Pi server not running?',
            'timestamp': time.time()
        }
    except socket.gaierror:
        return {
            'success': False,
            'error': 'Invalid IP address or hostname',
            'timestamp': time.time()
        }
    except Exception as e:
        return {
            'success': False,
            'error': f'Unexpected error: {str(e)}',
            'timestamp': time.time()
        }
    finally:
        try:
            sock.close()
        except:
            pass

def check_connection(pi_ip, pi_port, timeout=3):
    """
    Check if the Raspberry Pi server is reachable
    
    Args:
        pi_ip (str): Raspberry Pi IP address
        pi_port (int): Port number
        timeout (int): Connection timeout in seconds
    
    Returns:
        dict: Connection status
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        
        result = sock.connect_ex((pi_ip, pi_port))
        sock.close()
        
        if result == 0:
            return {
                'connected': True,
                'status': 'online',
                'timestamp': time.time()
            }
        else:
            return {
                'connected': False,
                'status': 'offline',
                'timestamp': time.time()
            }
            
    except Exception as e:
        return {
            'connected': False,
            'status': 'error',
            'error': str(e),
            'timestamp': time.time()
        }

def send_batch_commands(pi_ip, pi_port, commands, delay=0.1):
    """
    Send multiple commands in sequence
    
    Args:
        pi_ip (str): Raspberry Pi IP address
        pi_port (int): Port number
        commands (list): List of commands to send
        delay (float): Delay between commands in seconds
    
    Returns:
        list: List of responses
    """
    responses = []
    
    for command in commands:
        response = send_command(pi_ip, pi_port, command)
        responses.append({
            'command': command,
            'response': response
        })
        
        if delay > 0:
            time.sleep(delay)
    
    return responses

# Test function
if __name__ == "__main__":
    # Test the socket client
    test_ip = "192.168.16.154"  # Change to your Pi's IP
    test_port = 8888
    
    print("🔌 Testing socket connection...")
    
    # Check connection
    status = check_connection(test_ip, test_port)
    print(f"Connection status: {status}")
    
    if status['connected']:
        # Test commands
        test_commands = ["MODE:manual", "MOVE:forward", "MOVE:stop"]
        
        for cmd in test_commands:
            print(f"\n📤 Sending: {cmd}")
            response = send_command(test_ip, test_port, cmd)
            print(f"📥 Response: {response}")
            time.sleep(1)
    else:
        print("❌ Cannot connect to Pi server. Make sure server.py is running on the Pi.")