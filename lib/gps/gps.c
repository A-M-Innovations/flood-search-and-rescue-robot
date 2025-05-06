#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "../../lib/uart/uart.h"
#include "../../lib/gps/gps.h"

void gpgga_tokenizer(int rx_bytes, uint8_t* gps_data){
    const char *GPGGA_FIELD = "GPGGA Field";
    char *gps_data_str = malloc((RX_BUFFER_SIZE_256+1)*sizeof(uint8_t));
    char *p;
    char *GPGGA_vals[15];

    esp_log_level_set(GPGGA_FIELD, ESP_LOG_INFO);

    while (1){
        
        if (rx_bytes > 0){
            sprintf(gps_data_str,"%s",gps_data);
            p = strsep(&gps_data_str, ",\n");

            while(p != NULL){
                p = strsep(&gps_data_str, ",\n");
                if (strcmp(p, "$GPGGA") == 0){
                    for (int i = 0; i < 15; i++){
                        GPGGA_vals[i] = p;
                        ESP_LOGI(GPGGA_FIELD, "%s", GPGGA_vals[i]);
                        p = strsep(&gps_data_str, ",\n");
                    }
                }
            }
                }
    }
    free(gps_data_str);
}

void rx_gps_data(){

    static const int RX_BUFFER_SIZE = RX_BUFFER_SIZE_256;
    uint8_t* data = malloc((RX_BUFFER_SIZE_256+1)*sizeof(uint8_t));
    int rx_bytes = 0;

    while (1){
        int rx_bytes = uart_read_bytes(UART_NUM_2, data, RX_BUFFER_SIZE, 200 / portTICK_PERIOD_MS);
        if (rx_bytes > 0){
            data[rx_bytes] = 0;
            gpgga_tokenizer(rx_bytes, data);
        }
    }
    free(data);
}
