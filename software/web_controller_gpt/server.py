# server.py (放在 Raspberry Pi 執行)
import socket

HOST = "0.0.0.0"
PORT = 8888

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print("等待控制端連線中...")

    while True:
        conn, addr = s.accept()
        with conn:
            print("連線來自：", addr)
            data = conn.recv(1024).decode()
            if not data:
                continue
            print("收到指令：", data)

            # 根據指令執行動作
            if data == "mode:auto":
                print("切換到自動模式")
            elif data == "mode:manual":
                print("切換到手動模式")
            elif data.startswith("move:"):
                direction = data.split(":")[1]
                print(f"執行動作：{direction}")
            else:
                print("未知指令")

            conn.sendall("OK".encode())
