简介
本项目是一个使用ESP32来监控环境参数的示例项目，集成了多种传感器（温湿度传感器SHTC3、烟雾传感器、PIR人体感应传感器）和Wi-Fi功能，用于通过TCP协议将数据传输到客户端。该Python脚本是一个测试客户端，用于接收ESP32发送的传感器数据。

主要功能
ESP32通过I2C与SHTC3温湿度传感器通信，读取环境温湿度。
通过ADC读取烟雾传感器的模拟输出。
读取PIR传感器的状态以检测人体活动。
通过Wi-Fi连接，将传感器数据通过TCP发送到Python客户端。
Python脚本用作TCP客户端来接收和显示ESP32发送的传感器数据。
运行要求
硬件：ESP32开发板、SHTC3温湿度传感器、烟雾传感器、PIR传感器。
软件：ESP-IDF环境（用于编译和烧录ESP32代码）、Python 3.x。
网络：一个支持Wi-Fi的本地网络。
文件说明
main.c：ESP32端的代码，配置传感器、Wi-Fi以及TCP服务器。
test_client.py：Python脚本，用于测试和接收ESP32发送的数据。
硬件连接
SHTC3温湿度传感器

SDA连接到GPIO 10
SCL连接到GPIO 11
PIR传感器

数据引脚连接到GPIO 9
烟雾传感器

模拟输出引脚连接到GPIO 8
ESP32

确保ESP32已连接到本地Wi-Fi网络。
配置ESP32代码
打开ESP32代码（main.c），配置Wi-Fi的SSID和密码：

Copy code
#define EXAMPLE_WIFI_SSID "Your_SSID"
#define EXAMPLE_WIFI_PASS "Your_PASSWORD"
使用ESP-IDF编译并烧录代码到ESP32开发板。
观察Python脚本的输出，验证是否能够接收到ESP32发送的传感器数据。