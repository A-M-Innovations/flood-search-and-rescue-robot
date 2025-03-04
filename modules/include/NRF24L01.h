/*
 * NRF24L01.h
 *
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

void nrf24_write(uint8_t reg_addr, uint8_t data);
void nrf24_write_multibyte(uint8_t reg_addr, uint8_t *data, uint8_t size);
uint8_t nrf24_read(uint8_t reg_addr);
void nrf24_read_multibyte(uint8_t reg_addr, uint8_t *data, uint8_t size);
void nrf24_cmd(uint8_t cmd);
void nrf24_init(void);
void setup_tx_mode(uint8_t *tx_address, uint8_t tx_channel);
uint8_t tx(uint8_t *data);
void setup_rx_mode(uint8_t *rx_address, uint8_t rx_channel);
uint8_t data_available(int pipe_num);
void rx(uint8_t *data);
void NRF24_ReadAll (uint8_t *data);
void nrf24_reset();



//Memory Map for NRF24L01 registers

#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

//Commands
#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xA0
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
#define ACTIVATE            0x50
#define R_RX_PL_WID         0x60
#define W_ACK_PAYLOAD       0xA8
#define W_TX_PAYLOAD_NOACK  0x58
#define NOP                 0xFF




#endif /* INC_NRF24L01_H_ */
