#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "string.h"

static const char *TAG = "MASTER";

// SPI pin config 
#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

void wifi_init_ap() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "WASP_AP",
            .password = "wasp1234",
            .ssid_len = 0,
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started");
}

void spi_slave_task(void *arg) {
    uint8_t recvbuf[512];
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = sizeof(recvbuf) * 8;  // bit
    t.rx_buffer = recvbuf;

    while (1) {
        esp_err_t ret = spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Received CSI over SPI: %d bytes", t.trans_len / 8);
            // bebas si mau make atau gak dump data
            // ESP_LOG_BUFFER_HEXDUMP(TAG, recvbuf, t.trans_len / 8, ESP_LOG_INFO);
        }
    }
}

void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_ap();

    // SPI Slave config
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = PIN_NUM_CS,
        .mode = 0,
        .queue_size = 3,
    };

    ESP_ERROR_CHECK(spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI slave initialized");

    xTaskCreate(spi_slave_task, "spi_slave_task", 4096, NULL, 5, NULL);
}
