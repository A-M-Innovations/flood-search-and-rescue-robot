#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "../../lib/uart/uart.h"
#include "../../lib/gps/gps.h"

void app_main() {
    uart_init_8_bits(UART_NUM_2, UART_RX_PIN_16, UART_TX_PIN_17, RX_BUFFER_SIZE_256, BAUD_RATE_9600);
    xTaskCreate(rx_gps_data, "rx_uart_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
