#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "driver/adc.h"
#include "driver/gpio.h"  // 增加GPIO库

static const char *TAG = "shtc3_example";

#define I2C_MASTER_SCL_IO           11     
#define I2C_MASTER_SDA_IO           10  
#define I2C_MASTER_NUM              0                          
#define I2C_MASTER_FREQ_HZ          1000000                     
#define I2C_MASTER_TX_BUF_DISABLE   2048                          
#define I2C_MASTER_RX_BUF_DISABLE   0                          
#define I2C_MASTER_TIMEOUT_MS       0                      

#define SHTC3_I2C_ADDRESS     0x70  // SHTC3 I2C地址
#define SHTC3_CMD_WAKEUP      0x3517  // 唤醒命令
#define SHTC3_CMD_SLEEP       0xB098  // 休眠命令
#define SHTC3_CMD_MEASURE_T_RH_CLOCK_STRETCH 0x7CA2  // 启用时钟拉伸的测量命令（T先读取）

#define PIR_SENSOR_PIN 9  // GPIO 9引脚
#define SMOKE_SENSOR_PIN 8  // 烟雾传感器的模拟输出引脚 GPIO 8

#define TXD1_PIN (41)  
#define RXD1_PIN (42)

#define LED1_PIN 39  // LED引脚定义
#define LED2_PIN 38
#define LED3_PIN 37
#define LED4_PIN 36
#define LED5_PIN 35
#define SENSOR_PIN_A 4  // TVOC-CO2传感器的输出信号A连接到GPIO 4
#define SENSOR_PIN_B 5  // TVOC-CO2传感器的输出信号B连接到GPIO 5
void sensor_pin_init() {
    // 初始化PIN4和PIN5为输入模式
    gpio_set_direction(SENSOR_PIN_A, GPIO_MODE_INPUT);
    gpio_set_direction(SENSOR_PIN_B, GPIO_MODE_INPUT);
}

// 读取传感器引脚状态
void read_sensor_pins() {
    int signal_A = gpio_get_level(SENSOR_PIN_A);
    int signal_B = gpio_get_level(SENSOR_PIN_B);

    // 打印信号A和信号B的状态
    printf("Sensor Signal A: %d\n", signal_A);
    printf("Sensor Signal B: %d\n", signal_B);
}
// SHTC3传感器发送命令函数
esp_err_t shtc3_send_command(uint16_t command) {
    uint8_t command_buf[2];
    command_buf[0] = (uint8_t)(command >> 8);  // 高位字节
    command_buf[1] = (uint8_t)(command & 0xFF);  // 低位字节
    return i2c_master_write_to_device(I2C_MASTER_NUM, SHTC3_I2C_ADDRESS, command_buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// SHTC3读取数据函数
esp_err_t shtc3_read_data(uint16_t *temp, uint16_t *humidity) {
    uint8_t data[6];  // 6字节缓冲区用于存储温度和湿度数据及其CRC
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_NUM, SHTC3_I2C_ADDRESS, data, 6, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (err == ESP_OK) {
        *temp = (data[0] << 8) | data[1];  // 温度数据
        *humidity = (data[3] << 8) | data[4];  // 湿度数据
    }
    return err;
}

// 计算温度函数
float calculate_temperature(uint16_t raw_temp) {
    return -45.0 + 175.0 * ((float)raw_temp / 65535.0);
}

// 计算湿度函数
float calculate_humidity(uint16_t raw_humidity) {
    return 100.0 * ((float)raw_humidity / 65535.0);
}

// UART初始化函数
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
    uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);
}

// 烟雾传感器初始化函数
void smoke_sensor_init() {
    adc1_config_width(ADC_WIDTH_BIT_12);  // 配置12位ADC精度
    adc1_config_channel_atten(ADC1_CHANNEL_8, ADC_ATTEN_DB_11);  // 配置衰减，GPIO 8对应通道8
}

// 读取烟雾传感器的模拟值
int read_smoke_sensor() {
    return adc1_get_raw(ADC1_CHANNEL_8);  // 读取ADC1通道8的原始值
}

// PIR传感器初始化
void pir_sensor_init() {
    esp_rom_gpio_pad_select_gpio(PIR_SENSOR_PIN);
    gpio_set_direction(PIR_SENSOR_PIN, GPIO_MODE_INPUT);  // 配置为输入模式
}

// 读取人体感应传感器状态
int read_pir_sensor() {
    return gpio_get_level(PIR_SENSOR_PIN);
}

// LED初始化函数
void led_init() {
    gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED5_PIN, GPIO_MODE_OUTPUT);
}

// 跑马灯函数
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

// I2C主机初始化
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

// 主函数
void app_main(void) {
    uart1_init();  // 初始化UART
    ESP_ERROR_CHECK(i2c_master_init());  // 初始化I2C
    ESP_LOGI(TAG, "I2C initialized successfully");

    led_init();  // 初始化LED引脚
    sensor_pin_init();  // 初始化传感器引脚

    uint16_t raw_temperature, raw_humidity;
    float temperature, humidity;

    while (1) {
        // 唤醒传感器
        esp_err_t ret = shtc3_send_command(SHTC3_CMD_WAKEUP);
        vTaskDelay(pdMS_TO_TICKS(10));  // 延时10ms等待传感器唤醒

        // 检查命令发送结果
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Sensor wakeup command sent successfully.");
            
            // 发送测量命令
            ret = shtc3_send_command(SHTC3_CMD_MEASURE_T_RH_CLOCK_STRETCH);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Measurement command sent successfully.");
                vTaskDelay(pdMS_TO_TICKS(20));  // 等待测量完成

                // 读取温湿度数据
                ret = shtc3_read_data(&raw_temperature, &raw_humidity);
                if (ret == ESP_OK) {
                    temperature = calculate_temperature(raw_temperature);
                    humidity = calculate_humidity(raw_humidity);
                    printf("Temperature: %.2f C, Humidity: %.2f %%RH\n", temperature, humidity);
                } else {
                    ESP_LOGE(TAG, "Failed to read data from SHTC3");
                }
            } else {
                ESP_LOGE(TAG, "Failed to send measurement command: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "Failed to send wakeup command: %s", esp_err_to_name(ret));
        }

        // 读取PIR传感器状态
        int pir_state = read_pir_sensor();
        if (pir_state) {
            printf("人体感应传感器检测到活动！\n");
            led_chase();  // 调用跑马灯效果
        } else {
            printf("未检测到人体活动。\n");
        }

        // 读取烟雾传感器值
        int smoke_level = read_smoke_sensor();
        printf("烟雾传感器模拟量输出：%d\n", smoke_level);

        // 读取TVOC-CO2传感器信号
        read_sensor_pins();

        // 进入休眠模式
        ret = shtc3_send_command(SHTC3_CMD_SLEEP);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send sleep command: %s", esp_err_to_name(ret));
        }

        // 每隔1秒读取一次数据
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}