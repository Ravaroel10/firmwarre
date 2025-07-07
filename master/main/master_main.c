#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_websocket_client.h"
#include "string.h"
#include <math.h>

#define NUM_NODES 8
#define CSI_BUF_SIZE 256
#define WEBSOCKET_URI "ws://192.168.4.2:8000/ws" // Ganti alamat backend 

#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK  18
#define CS_PINS { 5, 4, 2, 15, 13, 12, 14, 27 }

static const char *TAG = "MASTER";

spi_device_handle_t node_spi[NUM_NODES];
esp_websocket_client_handle_t ws_client = NULL;

bool calibration_mode = false;
int16_t phase_offset[64][2] = {0}; // [real, imag]

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
            .channel = 1,
            .max_connection = 8,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi AP started");
}

void websocket_init() {
    esp_websocket_client_config_t websocket_cfg = {
        .uri = WEBSOCKET_URI,
    };
    ws_client = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_client_start(ws_client);
    ESP_LOGI(TAG, "WebSocket client started");
}

void send_csi_over_ws(int esp_id, int16_t *csi, int num_subcarriers) {
    char json[1024] = {0};
    for (int i = 0; i < num_subcarriers; i++) {
        int amp = csi[i * 2];
        char temp[16];
        sprintf(temp, "%d,", amp);
        strcat(json, temp);
    }
    int len = strlen(json);
    if (len > 0) json[len - 1] = 0;

    char output[1100];
    sprintf(output, "{\"esp_id\":%d,\"amplitudes\":[%s]}", esp_id, json);
    if (esp_websocket_client_is_connected(ws_client)) {
        esp_websocket_client_send_text(ws_client, output, strlen(output), portMAX_DELAY);
    }
}

void apply_phase_correction(int16_t *data, int num_subcarriers) {
    for (int i = 0; i < num_subcarriers; i++) {
        data[i * 2] -= phase_offset[i][0];
        data[i * 2 + 1] -= phase_offset[i][1];
    }
}

void calibrate_phase_reference(const int16_t *data, int num_subcarriers) {
    for (int i = 0; i < num_subcarriers; i++) {
        phase_offset[i][0] = data[i * 2];
        phase_offset[i][1] = data[i * 2 + 1];
    }
    ESP_LOGI(TAG, "Phase offset calibrated.");
}

void process_csi_data(int node_id, const uint8_t *data, size_t length_bytes) {
    int num_subcarriers = length_bytes / 4;
    int16_t *csi = (int16_t *)data;

    if (calibration_mode) {
        calibrate_phase_reference(csi, num_subcarriers);
    } else {
        apply_phase_correction(csi, num_subcarriers);
        send_csi_over_ws(node_id, csi, num_subcarriers);
    }
}

void send_command_to_node(int node_id, const uint8_t *cmd, size_t len) {
    if (node_id < 0 || node_id >= NUM_NODES) {
        ESP_LOGW(TAG, "Invalid node ID: %d", node_id);
        return;
    }

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = cmd,
    };

    esp_err_t ret = spi_device_transmit(node_spi[node_id], &t);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sent command to node %d: %.*s", node_id, len, cmd);
    } else {
        ESP_LOGW(TAG, "Failed to send command to node %d: %s", node_id, esp_err_to_name(ret));
    }
}

void init_all_nodes() {
    int cs_pins[NUM_NODES] = CS_PINS;

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    for (int i = 0; i < NUM_NODES; i++) {
        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = 1 * 1000 * 1000,
            .mode = 0,
            .spics_io_num = cs_pins[i],
            .queue_size = 1,
        };
        ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &node_spi[i]));
    }

    ESP_LOGI(TAG, "All %d SPI nodes initialized", NUM_NODES);
}

void csi_poll_task(void *arg) {
    uint8_t recvbuf[CSI_BUF_SIZE];
    spi_transaction_t t = {
        .length = CSI_BUF_SIZE * 8,
        .rx_buffer = recvbuf,
    };

    while (1) {
        for (int i = 0; i < NUM_NODES; i++) {
            memset(recvbuf, 0, CSI_BUF_SIZE);
            esp_err_t ret = spi_device_transmit(node_spi[i], &t);
            if (ret == ESP_OK) {
                process_csi_data(i, recvbuf, CSI_BUF_SIZE);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_ap();
    websocket_init();
    init_all_nodes();

    calibration_mode = true;

    // Contoh kirim perintah ke semua node (opsional)
    const char *start_cmd = "START_CSI";
    for (int i = 0; i < NUM_NODES; i++) {
        send_command_to_node(i, (const uint8_t *)start_cmd, strlen(start_cmd));
    }

    xTaskCreate(csi_poll_task, "csi_poll_task", 4096, NULL, 5, NULL);
}
