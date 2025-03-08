#ifndef GPS_H
#define GPS_H
#include <stdio.h>

void gpgga_tokenizer(int rx_bytes, uint8_t* gps_data);
void rx_gps_data(void *rx_rate_ms);
#endif