#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "../../lib/uart/uart.h"
#include "../../lib/gps/gps.h"


void gpgga_tokenizer(int rx_bytes, uint8_t* gps_data){
    const char *GPGGA_FIELDS = "GPGGA Field";
    char *gps_data_str = malloc((RX_BUFFER_SIZE_256+1)*sizeof(uint8_t));
    char *p;
    char *GPGGA_vals[15];

    esp_log_level_set(GPGGA_FIELDS, ESP_LOG_INFO);

    while (1){
        
        if (rx_bytes > 0){
            sprintf(gps_data_str,"%s",gps_data);
            p = strsep(&gps_data_str, ",\n");

            while(p != NULL){
                p = strsep(&gps_data_str, ",\n");
                if (strcmp(p, "$GPGGA") == 0){
                    for (int i = 0; i < 15; i++){
                        GPGGA_vals[i] = p;
                        ESP_LOGI(GPGGA_FIELDS, "%s", GPGGA_vals[i]);
                        p = strsep(&gps_data_str, ",\n");
                    }
                }
            }
                }
    }
    free(gps_data_str);
}

void rx_gps_data(void *rx_rate_ms){
    uint8_t* data = malloc((RX_BUFFER_SIZE_256+1)*sizeof(uint8_t));
    int rx_bytes = 0;
    int rx_rate_ms_int = *((int *)rx_rate_ms);

    while (1){        
        rx_uart_data(UART_2, RX_BUFFER_SIZE_256, &rx_bytes, data, rx_rate_ms_int);
        if (rx_bytes > 0){
            data[rx_bytes] = 0;
            gpgga_tokenizer(rx_bytes, data);
        }
    }
    free(data);
}
