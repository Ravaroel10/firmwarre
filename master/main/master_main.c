#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "string.h"
#include <math.h>

static const char *TAG = "MASTER";

// SPI pin config 
#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define CSI_BUF_SIZE 256 // 64 subcarrier x 2 (real+imag) x 2 byte = 256
#define UART_PORT UART_NUM_1
#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define BUF_SIZE 1024

bool calibration_mode = false;
int16_t phase_offset[64][2] = {0}; // [real, imag] offset per subcarrier

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

void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void send_csi_over_uart(int16_t *csi, int num_subcarriers) {
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
    sprintf(output, "{\"esp_id\":0,\"amplitudes\":[%s]}\n", json);
    uart_write_bytes(UART_PORT, output, strlen(output));
}

void apply_phase_correction(int16_t *data, int num_subcarriers) {
    for (int i = 0; i < num_subcarriers; i++) {
        data[i * 2] -= phase_offset[i][0];     // real
        data[i * 2 + 1] -= phase_offset[i][1]; // imag
    }
}

void calibrate_phase_reference(const int16_t *data, int num_subcarriers) {
    for (int i = 0; i < num_subcarriers; i++) {
        phase_offset[i][0] = data[i * 2];     // real
        phase_offset[i][1] = data[i * 2 + 1]; // imag
    }
    ESP_LOGI(TAG, "Phase offset calibrated.");
}

void process_csi_data(const uint8_t *data, size_t length_bytes) {
    int num_subcarriers = length_bytes / 4;
    int16_t *csi = (int16_t *)data;

    if (calibration_mode) {
        calibrate_phase_reference(csi, num_subcarriers);
    } else {
        apply_phase_correction(csi, num_subcarriers);
        for (int i = 0; i < num_subcarriers; i++) {
            ESP_LOGI(TAG, "CSI[%d] = %d + j%d", i, csi[i * 2], csi[i * 2 + 1]);
        }
        send_csi_over_uart(csi, num_subcarriers);
    }
}

void spi_slave_task(void *arg) {
    uint8_t recvbuf[CSI_BUF_SIZE];
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = CSI_BUF_SIZE * 8; // in bits
    t.rx_buffer = recvbuf;

    while (1) {
        esp_err_t ret = spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Received CSI over SPI: %d bytes", t.trans_len / 8);
            process_csi_data(recvbuf, t.trans_len / 8);
        }
    }
}

void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_ap();
    uart_init();

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

    calibration_mode = true;

    xTaskCreate(spi_slave_task, "spi_slave_task", 4096, NULL, 5, NULL);
}
