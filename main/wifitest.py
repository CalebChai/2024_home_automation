import socket

# ESP32的IP地址和端口
HOST = '192.168.31.191'  # 替换为ESP32的实际IP地址
PORT = 8080

# 创建一个TCP/IP socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    # 连接ESP32
    s.connect((HOST, PORT))
    # 发送消息
    s.sendall(b'Hello ESP32 from PC')
    # 接收来自ESP32的响应
    data = s.recv(1024)

print('Received from ESP32:', data.decode())
