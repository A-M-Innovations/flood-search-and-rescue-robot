#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "../../lib/uart/uart.h"
#include "../../lib/gps/gps.h"

void gpgga_tokenizer(int rx_bytes, uint8_t* gps_data){
    char *tokens[15];
    char *gps_data_str = malloc((RX_BUFFER_SIZE_256+1)*sizeof(uint8_t));
    // later strsep, modifies the gps_data_str pointer so this variable keeps
    // the original pointer value to free it at the end of the function
    char *gps_data_str_temp = gps_data_str;
    char *token;

        if (rx_bytes > 0){
            sprintf(gps_data_str,"%s",gps_data);
            token = strsep(&gps_data_str, ",\n");
            while(token != NULL){
                if (strcmp(token, "$GPGGA") == 0){
                    for (int i = 0; i < 15; i++){
                        tokens[i] = token;
                        printf("%s\n", tokens[i]);
                        token = strsep(&gps_data_str, ",\n");
                    }
                break;
                }
                token = strsep(&gps_data_str, ",\n");
            }
        }
    free(gps_data_str_temp);
}

void rx_gps_data(){
    static const int RX_BUFFER_SIZE = RX_BUFFER_SIZE_256;
    uint8_t* data = malloc((RX_BUFFER_SIZE_256+1)*sizeof(uint8_t));
    int rx_bytes = 0;

    while (1){
        rx_bytes = uart_read_bytes(UART_NUM_2, data, RX_BUFFER_SIZE, 200 / portTICK_PERIOD_MS);
        if (rx_bytes > 0){
            data[rx_bytes] = 0;
            gpgga_tokenizer(rx_bytes, data);
        }
    }
    free(data);
}
