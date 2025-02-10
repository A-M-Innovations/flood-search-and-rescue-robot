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

void main(void) {

    uart_init();

}