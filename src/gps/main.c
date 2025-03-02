#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

const int uart_num = UART_NUM_2;

static const int RX_BUFFER_SIZE = 256;

void uart_init(void) {

    const uart_config_t uart_config = {
        .baud_rate  = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));    
    ESP_ERROR_CHECK(uart_driver_install(uart_num, RX_BUFFER_SIZE, 0, 0, NULL, 0));
}

static void rx_uart_data(){

    const char *GPGGA_FIELDS = "GPGGA Field";
    esp_log_level_set(GPGGA_FIELDS, ESP_LOG_INFO);
    uint8_t* data = malloc((RX_BUFFER_SIZE+1)*sizeof(uint8_t));
    char *data_str = malloc((RX_BUFFER_SIZE+1)*sizeof(uint8_t));
    char *p;
    char *GPGGA_vals[15];

    while (1){
        int rx_bytes = uart_read_bytes(uart_num, data, RX_BUFFER_SIZE, 200 / portTICK_PERIOD_MS);
        if (rx_bytes > 0){
            data[rx_bytes] = 0;
            sprintf(data_str,"%s",data);
            p = strsep(&data_str, ",\n");

            while(p != NULL){
                p = strsep(&data_str, ",\n");
                if (strcmp(p, "$GPGGA") == 0){
                    for (int i = 0; i < 15; i++){
                        GPGGA_vals[i] = p;
                        ESP_LOGI(GPGGA_FIELDS, "%s", GPGGA_vals[i]);
                        p = strsep(&data_str, ",\n");
                    }
                }
            }
                }
    }
    free(data);
    free(data_str);
    
}

void app_main() {

    uart_init();
    xTaskCreate(rx_uart_data, "rx_uart_data_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}