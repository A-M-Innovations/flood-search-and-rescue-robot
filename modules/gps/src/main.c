#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

static const int RX_BUFFER_SIZE = 128;

void uart_init(void) {

    const uart_config_t uart_config = {
        .baud_rate  = UART_DATA_8_BITS,
        .data_bits = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0
    };

    uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);    
    uart_driver_install(UART_NUM_0, RX_BUFFER_SIZE, 0, 0, NULL, 0);
}

static void rx_uart_data(){

    const char *RX_UART_TAG = "RX UART"
    esp_log_level_set(RX_UART_TAG, ESP_LOG_INFO);
    uint8_t* data = malloc((RX_BUFFER_SIZE+1)*sizeof(uint8_t))
    while (1){

        int rx_bytes = uart_read_bytes(UART_NUM_0, data, RX_BUFFER_SIZE, 200 / portTICK_RATE_MS);
        if (rx_bytes > 0){
            data[rx_bytes] = 0;
            ESP_LOGI(RX_UART_TAG, ("Read %d bytes: '%s'", rx_bytes, data));
        }
    }
    free(data);
}

void main(void) {

    uart_init();
    xTaskCreate(rx_uart_data, "rx_uart_data_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}