#ifndef UART_H
#define UART_H
#include "driver/uart.h"

#define RX_BUFFER_SIZE_256 256
#define UART_RX_PIN_16     16
#define UART_TX_PIN_17     17
#define BAUD_RATE_9600     9600

void uart_init_8_bits(const int UART_NUM_2, const int UART_RX_PIN, const int UART_TX_PIN, const int RX_BUFFER_SIZE, const int BAUD_RATE);
int rx_uart_data(const int UART_NUM_2, const int RX_BUFFER_SIZE, uint8_t* data, int ms);
#endif
