#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/adc.h"  // 使用旧版API

#include "driver/gpio.h"

/* Wi-Fi 设置 */
#define EXAMPLE_WIFI_SSID "calebpad"
#define EXAMPLE_WIFI_PASS "99405215"

/* FreeRTOS事件组，用于标志Wi-Fi连接状态 */
static EventGroupHandle_t wifi_event_group;
static const int CONNECTED_BIT = BIT0;

/* 日志标签 */
static const char *TAG = "wifi_sensor_example";

/* I2C 相关配置 */
#define I2C_MASTER_SCL_IO           11     
#define I2C_MASTER_SDA_IO           10  
#define I2C_MASTER_NUM              0                          
#define I2C_MASTER_FREQ_HZ          1000000                     
#define I2C_MASTER_TX_BUF_DISABLE   2048                          
#define I2C_MASTER_RX_BUF_DISABLE   0                          
#define I2C_MASTER_TIMEOUT_MS       0                      

/* SHTC3 传感器 I2C 地址及命令 */
#define SHTC3_I2C_ADDRESS           0x70  
#define SHTC3_CMD_WAKEUP            0x3517  
#define SHTC3_CMD_SLEEP             0xB098  
#define SHTC3_CMD_MEASURE_T_RH_CLOCK_STRETCH 0x7CA2  

/* 传感器引脚配置 */
#define PIR_SENSOR_PIN 9  // PIR传感器引脚
#define SMOKE_SENSOR_CHANNEL ADC_CHANNEL_8  // 烟雾传感器ADC通道
#define SENSOR_PIN_A 4  // TVOC-CO2传感器引脚A
#define SENSOR_PIN_B 5  // TVOC-CO2传感器引脚B

/* LED 引脚配置 */
#define LED1_PIN 39
#define LED2_PIN 38
#define LED3_PIN 37
#define LED4_PIN 36
#define LED5_PIN 35

// 声明函数以避免隐式声明错误
void led_init(void);
void sensor_pin_init(void);
void pir_sensor_init(void);
void smoke_sensor_init(void);
int read_smoke_sensor(void);

/* esp_netif对象表示Wi-Fi站点 */
static esp_netif_t *sta_netif = NULL;

/* SHTC3 传感器发送命令函数 */
esp_err_t shtc3_send_command(uint16_t command) {
    uint8_t command_buf[2];
    command_buf[0] = (uint8_t)(command >> 8);  // 高位字节
    command_buf[1] = (uint8_t)(command & 0xFF);  // 低位字节
    return i2c_master_write_to_device(I2C_MASTER_NUM, SHTC3_I2C_ADDRESS, command_buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/* SHTC3读取数据函数 */
esp_err_t shtc3_read_data(uint16_t *temp, uint16_t *humidity) {
    uint8_t data[6];  // 6字节缓冲区用于存储温度和湿度数据及其CRC
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_NUM, SHTC3_I2C_ADDRESS, data, 6, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (err == ESP_OK) {
        *temp = (data[0] << 8) | data[1];  // 温度数据
        *humidity = (data[3] << 8) | data[4];  // 湿度数据
    }
    return err;
}

/* 计算温度函数 */
float calculate_temperature(uint16_t raw_temp) {
    return -45.0 + 175.0 * ((float)raw_temp / 65535.0);
}

/* 计算湿度函数 */
float calculate_humidity(uint16_t raw_humidity) {
       return 100.0 * ((float)raw_humidity / 65535.0);
}

/* UART初始化函数 */
void uart1_init() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 41, 42, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);
}

/* 烟雾传感器初始化函数（使用新ADC API） */
void smoke_sensor_init() {
    adc1_config_width(ADC_WIDTH_BIT_12);  // 配置12位ADC精度
    adc1_config_channel_atten(ADC1_CHANNEL_8, ADC_ATTEN_DB_11);  // 配置衰减，使用GPIO 8
}

int read_smoke_sensor() {
    return adc1_get_raw(ADC1_CHANNEL_8);  // 读取ADC1通道8的原始值
}


/* PIR传感器初始化 */
void pir_sensor_init() {
    esp_rom_gpio_pad_select_gpio(PIR_SENSOR_PIN);
    gpio_set_direction(PIR_SENSOR_PIN, GPIO_MODE_INPUT);  // 配置为输入模式
}

/* 读取PIR传感器状态 */
int read_pir_sensor() {
    return gpio_get_level(PIR_SENSOR_PIN);  // 获取PIR传感器的引脚电平
}

/* LED初始化函数 */
void led_init() {
    gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED5_PIN, GPIO_MODE_OUTPUT);
}

/* 跑马灯函数 */
void led_chase() {
    int delay_ms = 200;  // 每个LED的延时
    gpio_set_level(LED1_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    gpio_set_level(LED1_PIN, 0);

    gpio_set_level(LED2_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    gpio_set_level(LED2_PIN, 0);

    gpio_set_level(LED3_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    gpio_set_level(LED3_PIN, 0);

    gpio_set_level(LED4_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    gpio_set_level(LED4_PIN, 0);

    gpio_set_level(LED5_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    gpio_set_level(LED5_PIN, 0);
}

/* TVOC-CO2传感器引脚初始化 */
void sensor_pin_init() {
    gpio_set_direction(SENSOR_PIN_A, GPIO_MODE_INPUT);
    gpio_set_direction(SENSOR_PIN_B, GPIO_MODE_INPUT);
}

/* 读取TVOC-CO2传感器的信号引脚 */
void read_sensor_pins() {
    int signal_A = gpio_get_level(SENSOR_PIN_A);
    int signal_B = gpio_get_level(SENSOR_PIN_B);
    printf("Sensor Signal A: %d\n", signal_A);
    printf("Sensor Signal B: %d\n", signal_B);
}

/* 控制LED状态的函数 */
void control_led(int state) {
    if (state == 1) {
        gpio_set_level(LED2_PIN, 1); // 打开LED
        ESP_LOGI(TAG, "LED turned ON");
    } else {
        gpio_set_level(LED2_PIN, 0); // 关闭LED
        ESP_LOGI(TAG, "LED turned OFF");
    }
}

/* I2C主机初始化函数 */
static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/* Wi-Fi事件处理 */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi STA started, trying to connect to SSID: %s", EXAMPLE_WIFI_SSID);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGE(TAG, "Wi-Fi disconnected, trying to reconnect...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    }
}

/* Wi-Fi 初始化 */
void initialise_wifi(void) {
    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

/* TCP服务器任务 */
void tcp_server_task(void *pvParameters) {
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 监听所有网络接口
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(8080);  // 监听8080端口
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            close(listen_sock);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "Socket bound, port 8080");

        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
            close(listen_sock);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "Socket listening");

        while (1) {
            struct sockaddr_in source_addr;
            socklen_t addr_len = sizeof(source_addr);
            int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
            if (sock < 0) {
                ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
                break;
            }

            inet_ntoa_r(source_addr.sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
            ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

            while (1) {
                int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
                if (len < 0) {
                    ESP_LOGE(TAG, "recv failed: errno %d", errno);
                    break;
                } else if (len == 0) {
                    ESP_LOGI(TAG, "Connection closed");
                    break;
                } else {
                    rx_buffer[len] = 0;
                    ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

                    if (strcmp(rx_buffer, "LED1_ON") == 0) {
                        control_led(1);
                        char *response = "LED turned ON\n";
                        send(sock, response, strlen(response), 0);
                    } else if (strcmp(rx_buffer, "LED1_OFF") == 0) {
                        control_led(0);
                        char *response = "LED turned OFF\n";
                        send(sock, response, strlen(response), 0);
                    } else if (strcmp(rx_buffer, "GET_DATA") == 0) {
                        // 读取温湿度传感器数据
                        uint16_t raw_temp, raw_humidity;
                        shtc3_send_command(SHTC3_CMD_WAKEUP);  // 唤醒SHTC3传感器
                        shtc3_read_data(&raw_temp, &raw_humidity);  // 读取数据
                        shtc3_send_command(SHTC3_CMD_SLEEP);  // 传感器进入睡眠

                        float temperature = calculate_temperature(raw_temp);
                        float humidity = calculate_humidity(raw_humidity);

                        char sensor_data[128];
                        snprintf(sensor_data, sizeof(sensor_data), "Temperature: %.2f C, Humidity: %.2f %%RH\n", temperature, humidity);
                        send(sock, sensor_data, strlen(sensor_data), 0);
                    } else {
                        char *response = "Unknown command\n";
                        send(sock, response, strlen(response), 0);
                    }
                }
            }
            if (sock != -1) {
                ESP_LOGI(TAG, "Shutting down socket and restarting...");
                shutdown(sock, 0);
                close(sock);
            }
        }
        if (listen_sock != -1) {
            ESP_LOGI(TAG, "Shutting down listen socket");
            close(listen_sock);
        }
    }
    vTaskDelete(NULL);
}

/* 主程序入口 */
void app_main(void) {
    nvs_flash_init();  // 初始化NVS
    initialise_wifi(); // 初始化Wi-Fi
    i2c_master_init(); // 初始化I2C主机
    led_init();  // 初始化LED
    sensor_pin_init();  // 初始化TVOC-CO2传感器引脚
    smoke_sensor_init();  // 初始化烟雾传感器
    pir_sensor_init();  // 初始化PIR传感器

    // 创建TCP服务器任务
    xTaskCreate(tcp_server_task, "tcp_server_task", 4096, NULL, 5, NULL);

    // 主循环读取传感器数据
    while (1) {
        int smoke_level = read_smoke_sensor();  // 读取烟雾传感器值
        int pir_status = read_pir_sensor();  // 读取PIR传感器状态
        read_sensor_pins();  // 读取TVOC-CO2传感器引脚信号

        printf("Smoke Level: %d, PIR Status: %d\n", smoke_level, pir_status);
        vTaskDelay(pdMS_TO_TICKS(1000));  // 每秒读取一次
    }
}

