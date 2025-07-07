#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include <string.h>

#define TAG "NODE"
#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5   // <--- Ganti sesuai ID node (CS pin)
#define BUF_SIZE     256

static bool csi_enabled = false;

// Mengirim CSI ke master via SPI
void send_csi_spi(const uint8_t *data, uint16_t len) {
    if (!csi_enabled || len > BUF_SIZE) return;

    spi_slave_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };

    esp_err_t ret = spi_slave_transmit(HSPI_HOST, &t, pdMS_TO_TICKS(10));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SPI TX failed: %s", esp_err_to_name(ret));
    }
}

// Callback saat CSI diterima
void csi_callback(void *ctx, wifi_csi_info_t *info) {
    if (csi_enabled && info && info->buf) {
        send_csi_spi(info->buf, info->len);
    }
}

// Eksekusi perintah dari master via SPI
void handle_command(const char *cmd) {
    if (strncmp(cmd, "START", 5) == 0) {
        csi_enabled = true;
        ESP_LOGI(TAG, "CSI STARTED");
    } else if (strncmp(cmd, "STOP", 4) == 0) {
        csi_enabled = false;
        ESP_LOGI(TAG, "CSI STOPPED");
    } else if (strncmp(cmd, "SSID:", 5) == 0) {
        char ssid[33] = {0};
        strncpy(ssid, cmd + 5, sizeof(ssid) - 1);
        ESP_LOGI(TAG, "Received SSID: %s", ssid);
        // Optional: Simpan ke NVS atau apply langsung
    } else if (strncmp(cmd, "PWD:", 4) == 0) {
        char pwd[65] = {0};
        strncpy(pwd, cmd + 4, sizeof(pwd) - 1);
        ESP_LOGI(TAG, "Received PWD: %s", pwd);
        // Optional: Simpan ke NVS atau apply langsung
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd);
    }
}

// Task untuk menerima perintah dari master
void spi_slave_command_task(void *arg) {
    uint8_t rxbuf[BUF_SIZE] = {0};
    spi_slave_transaction_t t;

    while (1) {
        memset(&t, 0, sizeof(t));
        t.length = BUF_SIZE * 8;
        t.rx_buffer = rxbuf;

        if (spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY) == ESP_OK) {
            rxbuf[t.trans_len / 8] = '\0'; // Null-terminate string aman
            ESP_LOGI(TAG, "Received SPI CMD: %s", (char *)rxbuf);
            handle_command((char *)rxbuf);
        }
    }
}

// Inisialisasi Wi-Fi Station
void wifi_init_sta() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "WASP_AP",
            .password = "wasp1234",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

// Entry Point
void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    // Init SPI slave
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

    // Enable CSI
    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htlf_en = true,
        .stbc_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(&csi_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    ESP_LOGI(TAG, "Node ready. Waiting for commands via SPI...");

    xTaskCreate(spi_slave_command_task, "spi_slave_command_task", 4096, NULL, 5, NULL);
}
