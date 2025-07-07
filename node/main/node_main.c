#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

static const char *TAG = "NODE";

// SPI config
#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO -1
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

spi_device_handle_t spi;

void send_csi_spi(const uint8_t *data, uint16_t len) {
    if (len > 256) return; // Hindari overflow

    spi_transaction_t t = {
        .length = len * 8,  // dalam bit
        .tx_buffer = data,
    };

    for (int i = 0; i < 3; i++) {
        esp_err_t ret = spi_device_transmit(spi, &t);
        if (ret == ESP_OK) return;
        ESP_LOGW(TAG, "SPI transmit failed (try %d): %s", i + 1, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGE(TAG, "SPI transmit failed permanently");
}

void csi_callback(void *ctx, wifi_csi_info_t *data) {
    if (data && data->buf) {
        ESP_LOGI(TAG, "CSI len: %d", data->len);
        send_csi_spi(data->buf, data->len);
        vTaskDelay(pdMS_TO_TICKS(10)); // Throttle biar SPI stabil
    }
}

void wifi_init_sta() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "WASP_AP",          // SSID AP
            .password = "wasp1234",     // Password AP
        },
    };

    ESP_LOGI(TAG, "WiFi connecting to %s", wifi_config.sta.ssid);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    // Init SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));

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

    ESP_LOGI(TAG, "Node ready with CSI over SPI");
}
