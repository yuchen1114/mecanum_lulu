import socket

RASPBERRY_PI_IP = "192.168.50.123"  # 請換成你的 Pi 的 IP
PORT = 8888                         # 要和 Raspberry Pi 的 server 端一致

def send_command(cmd):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((RASPBERRY_PI_IP, PORT))
            s.sendall(cmd.encode())
            response = s.recv(1024).decode()
            return response
    except Exception as e:
        return f"錯誤：{e}"
