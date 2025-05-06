#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "../../lib/uart/uart.h"

void uart_init_8_bits(const int UART_NUM, const int UART_RX_PIN, const int UART_TX_PIN, const int RX_BUFFER_SIZE, const int BAUD_RATE) {

    const uart_config_t uart_config = {
        .baud_rate  = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, RX_BUFFER_SIZE, 0, 0, NULL, 0));
}
