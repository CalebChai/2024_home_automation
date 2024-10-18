import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import socket

# ESP32的IP地址和端口
ESP32_IP = "192.168.1.100"  # 替换为你的ESP32的IP地址
ESP32_PORT = 8080           # ESP32的TCP服务器端口

# 定义接收数据的长度
BUFFER_SIZE = 1024

# 发送命令并接收数据
def send_command(command):
    try:
        # 创建TCP连接
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ESP32_IP, ESP32_PORT))

        # 发送命令
        sock.sendall(command.encode())

        # 接收ESP32的响应
        response = sock.recv(BUFFER_SIZE).decode()
        print(f"ESP32响应: {response}")
        return response
    
    except Exception as e:
        print(f"连接失败: {e}")
        return None
    
    finally:
        sock.close()

# 创建按钮事件函数
def led_on_callback(event, led_num):
    response = send_command(f"LED{led_num}_ON")
    if response:
        print(f"LED{led_num} ON: {response}")

def led_off_callback(event, led_num):
    response = send_command(f"LED{led_num}_OFF")
    if response:
        print(f"LED{led_num} OFF: {response}")

def get_sensor_data_callback(event):
    # 请求传感器数据的命令
    response = send_command("GET_DATA")
    if response:
        print(f"传感器数据: {response}")

# 创建图形界面
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.3)

# 创建5个LED的按钮
buttons = []
for i in range(1, 6):
    # 创建LED打开按钮
    ax_on = plt.axes([0.1, 0.05 * i, 0.35, 0.04])
    button_on = Button(ax_on, f"打开LED{i}")
    button_on.on_clicked(lambda event, led_num=i: led_on_callback(event, led_num))
    buttons.append(button_on)

    # 创建LED关闭按钮
    ax_off = plt.axes([0.55, 0.05 * i, 0.35, 0.04])
    button_off = Button(ax_off, f"关闭LED{i}")
    button_off.on_clicked(lambda event, led_num=i: led_off_callback(event, led_num))
    buttons.append(button_off)

# 创建一个按钮来获取传感器数据
ax_sensor = plt.axes([0.3, 0.9, 0.4, 0.05])
button_sensor = Button(ax_sensor, "获取传感器数据")
button_sensor.on_clicked(get_sensor_data_callback)

# 显示图形界面
plt.show()
