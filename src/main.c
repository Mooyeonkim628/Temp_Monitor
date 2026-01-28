#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

#include "driver/i2c.h"

#define WIFI_SSID   "SSID"
#define WIFI_PASS   "PASSWORD"
#define SERVER_IP   "IP"
#define SERVER_PORT 9000

static const char *TAG = "TCP_CLIENT";

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define BMP280_I2C_ADDR             0x76
#define BMP280_REG_DIG_T1           0x88
#define BMP280_REG_DIG_T2           0x8A
#define BMP280_REG_DIG_T3           0x8C
#define BMP280_REG_CTRL_MEAS        0xF4
#define BMP280_REG_CONFIG           0xF5
#define BMP280_REG_TEMP_MSB         0xFA

typedef int32_t  BMP280_S32_t;
typedef int64_t  BMP280_S64_t;
typedef uint32_t BMP280_U32_t;
typedef uint64_t BMP280_U64_t;

static uint16_t dig_T1;
static int16_t  dig_T2;
static int16_t  dig_T3;

static BMP280_S32_t t_fine;

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t bmp280_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    return i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_I2C_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

static esp_err_t bmp280_read_bytes(uint8_t start_reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_I2C_ADDR, &start_reg, 1, buf, len, 1000 / portTICK_PERIOD_MS);
}

static BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
    BMP280_S32_t var1, var2, T;
    var1 = ((((adc_T >> 3)  - ((BMP280_S32_t)dig_T1 << 1))) *
            ((BMP280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((BMP280_S32_t)dig_T1)) *
              ((adc_T >> 4) - ((BMP280_S32_t)dig_T1))) >> 12) *
            ((BMP280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;  
    return T;
}

void bmp280_init_default(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized.");

    uint8_t buffer[6];
    ESP_ERROR_CHECK(bmp280_read_bytes(BMP280_REG_DIG_T1, buffer, 6));

    dig_T1 = (uint16_t)(buffer[1] << 8 | buffer[0]);
    dig_T2 = (int16_t)(buffer[3] << 8 | buffer[2]);
    dig_T3 = (int16_t)(buffer[5] << 8 | buffer[4]);

    ESP_LOGI(TAG, "dig_T1=%u, dig_T2=%d, dig_T3=%d", dig_T1, dig_T2, dig_T3);
    ESP_ERROR_CHECK(bmp280_write_reg(BMP280_REG_CTRL_MEAS, 0x27));
    ESP_ERROR_CHECK(bmp280_write_reg(BMP280_REG_CONFIG,   0xA0));
    ESP_LOGI(TAG, "BMP280 initialized.");
}

float bmp280_read_temperature_c(void)
{
    uint8_t data[3];
    esp_err_t err = bmp280_read_bytes(BMP280_REG_TEMP_MSB, data, 3);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read temperature raw data, err=%d", err);
        return -1000.0f;
    }

    int32_t adc_T = (int32_t)(
        ((uint32_t)data[0] << 12) |
        ((uint32_t)data[1] << 4 ) |
        ((uint32_t)data[2] >> 4 )
    );

    BMP280_S32_t T_actual = bmp280_compensate_T_int32(adc_T);
    float temp_c = (float)T_actual / 100.0f;

    return temp_c;
}

typedef struct {
    uint32_t seq;
    float    temperature_c;
} temp_msg_t;

static QueueHandle_t temp_queue_wifi = NULL;
static void temp_task(void *pvParameters);
static void tcp_client_task(void *pvParameters);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    static bool tcp_task_started = false;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA START, connecting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *dis = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGI(TAG, "WiFi disconnected, reason: %d", dis->reason);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP, starting TCP client task...");
        if (!tcp_task_started) {
            tcp_task_started = true;
            xTaskCreate(&tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
        }
    }
}

static void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

static void temp_task(void *pvParameters)
{
    uint32_t seq = 0;
    ESP_LOGI(TAG, "Temperature task started.");
    const TickType_t period = pdMS_TO_TICKS(500); 

    while (1) {
        float temp_c = bmp280_read_temperature_c();
        temp_msg_t msg = {
            .seq = seq++,
            .temperature_c = temp_c
        };
        if (temp_queue_wifi != NULL) {
            xQueueSend(temp_queue_wifi, &msg, portMAX_DELAY);
        }
        vTaskDelay(period);
    }
}

static void tcp_client_task(void *pvParameters)
{
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(SERVER_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Connecting to %s:%d", SERVER_IP, SERVER_PORT);
    int err = connect(sock, (struct sockaddr *)&dest_addr,sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Successfully connected");

    temp_msg_t msg;
    char payload[100];

    while (1) {
        if (xQueueReceive(temp_queue_wifi, &msg,portMAX_DELAY) == pdTRUE) {
            printf("[%lu] Temp=%.2f C\n",(unsigned long)msg.seq, msg.temperature_c);
            int len = snprintf(payload, sizeof(payload), "[%lu] Temp=%.2f\n", (unsigned long)msg.seq, msg.temperature_c);
            int sent = send(sock, payload, len, 0);
            if (sent < 0) {
                ESP_LOGE(TAG,"Error occurred during sending: errno %d", errno);
                break;
            }
        }
    }

    ESP_LOGI(TAG, "Closing socket");
    close(sock);
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t x = nvs_flash_init();
    if (x == ESP_ERR_NVS_NO_FREE_PAGES || x == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        x = nvs_flash_init();
    }
    ESP_ERROR_CHECK(x);

    temp_queue_wifi = xQueueCreate(10, sizeof(temp_msg_t));
    bmp280_init_default();
    xTaskCreate(&temp_task, "temp_task", 4096, NULL, 5, NULL);
    wifi_init_sta();

    ESP_LOGI(TAG, "System started.");
}
