/*
 * NRF24L01.c
 *
 *  This library is used to initialize the NRF24L01, set it up in either tx or rx mode,
 *  and read/write it's registers
 */

#include "stm32f4xx_hal.h"
#include "../../../../lib/NRF24L01.h"

extern SPI_HandleTypeDef hspi1;

#define NRF24_SPI &hspi1

#define NFR24_CS_PORT GPIOB
#define NFR24_CS_PIN GPIO_PIN_6

#define NFR24_CE_PORT GPIOC
#define NFR24_CE_PIN GPIO_PIN_7


//Sets Chip Select pin high to deselect the NRF24L01
void cs_deselect()
{
    HAL_GPIO_WritePin (NFR24_CS_PORT, NFR24_CS_PIN, GPIO_PIN_SET);
}

//Sets Chip Select pin low to select the NRF24L01
void cs_select()
{
    HAL_GPIO_WritePin (NFR24_CS_PORT, NFR24_CS_PIN, GPIO_PIN_RESET);
}

//Sets the CE pin high and activates the chip in RX or TX mode
void ce_enable()
{
    HAL_GPIO_WritePin (NFR24_CE_PORT, NFR24_CE_PIN, GPIO_PIN_SET);
}

//Sets the CE pin low and deactivates the chip
void ce_disable()
{
    HAL_GPIO_WritePin (NFR24_CE_PORT, NFR24_CE_PIN, GPIO_PIN_RESET);
}

//Used to send 1 byte of data to the NRF24
void nrf24_write(uint8_t reg_addr, uint8_t data)
{
    uint8_t buf[2];

    //need to select the device to write to it
    cs_select();


    buf[0] = reg_addr | (1<<5); //5th bit is 1 to perform write command on the passed in register address
    buf[1] = data; //data written to the register

    //The protocol requires the first byte to be a command and the second byte to be data
    HAL_SPI_Transmit (NRF24_SPI, buf, 2, 1000);

    //deselect the device to stop writing to it
    cs_deselect();

}

//Used to send multiple byte of data to the NRF24
void nrf24_write_multibyte(uint8_t reg_addr, uint8_t *data, uint8_t size)
{
    uint8_t buf[2];

    //need to select the device to write to it
    cs_select();

    buf[0] = reg_addr|1 << 5; //5th bit is 1 to perform write command on the passed in register address

    //The protocol requires the first byte to be a command and the subsequent bytes to be data
    HAL_SPI_Transmit (NRF24_SPI, buf, 1, 100);
    HAL_SPI_Transmit (NRF24_SPI, data, size, 1000);

    //deselect the device to stop writing to it
    cs_deselect();
}

//Used to read 1 byte of data to the NRF24
uint8_t nrf24_read(uint8_t reg_addr)
{
    uint8_t data =0;

    //need to select the device to read it
    cs_select();

    //passing in the register address translates to a read command
    HAL_SPI_Transmit (NRF24_SPI, &reg_addr, 1, 100);
    HAL_SPI_Receive (NRF24_SPI, &data, 1, 100);

    //deselect the device to stop reading it
    cs_deselect();

    return data;
}

//Used to read multiple bytes of data to the NRF24
void nrf24_read_multibyte(uint8_t reg_addr, uint8_t *data, uint8_t size)
{

    //need to select the device to read it
    cs_select();


    HAL_SPI_Transmit (NRF24_SPI, &reg_addr, 1, 100);
    HAL_SPI_Receive (NRF24_SPI, data, size, 1000); //stores data in data array variable

    //deselect the device to stop reading it
    cs_deselect();


}

//writes other commands to the NRF24 besides the read/write commands
void nrf24_cmd(uint8_t cmd)
{

    //need to select the device to write to it
    cs_select();


    HAL_SPI_Transmit (NRF24_SPI, &cmd, 1, 100);

    //need to deselect the device to stop writing to it
    cs_deselect();
}

//reset NRF24 registers to their default values at system start up
void reset()
{
    nrf24_write(CONFIG, 0x08);
    nrf24_write(EN_AA, 0x3F);
    nrf24_write(EN_RXADDR, 0x03);
    nrf24_write(SETUP_AW, 0x03);
    nrf24_write(SETUP_RETR, 0x03);
    nrf24_write(RF_CH, 0x02);
    nrf24_write(RF_SETUP, 0x0E);
    nrf24_write(STATUS, 0x0E);
    nrf24_write(OBSERVE_TX, 0x00);
    nrf24_write(CD, 0x00);
    uint8_t rx_addr_p0_rst_val[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    nrf24_write_multibyte(RX_ADDR_P0, rx_addr_p0_rst_val, 5);
    uint8_t rx_addr_p1_rst_val[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
    nrf24_write_multibyte(RX_ADDR_P1, rx_addr_p1_rst_val, 5);
    nrf24_write(RX_ADDR_P2, 0xC3);
    nrf24_write(RX_ADDR_P3, 0xC4);
    nrf24_write(RX_ADDR_P4, 0xC5);
    nrf24_write(RX_ADDR_P5, 0xC6);
    uint8_t tx_addr_rst_val[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    nrf24_write_multibyte(TX_ADDR, tx_addr_rst_val, 5);
    nrf24_write(RX_PW_P0, 0);
    nrf24_write(RX_PW_P1, 0);
    nrf24_write(RX_PW_P2, 0);
    nrf24_write(RX_PW_P3, 0);
    nrf24_write(RX_PW_P4, 0);
    nrf24_write(RX_PW_P5, 0);
    nrf24_write(FIFO_STATUS, 0x11);
    nrf24_write(DYNPD, 0);
    nrf24_write(FEATURE, 0);
}


//Initialize the NRF24 with desired settings
void nrf24_init(void)
{

    //set CE low to prevent the module from going into transmit or receive mode
    ce_disable();

    reset();

    nrf24_write(CONFIG, 0x00);
    nrf24_write(EN_AA, 0x00); //No Auto ACK
    nrf24_write(EN_RXADDR, 0x00); //disable all data pipes. Will be configured in RX function
    nrf24_write(SETUP_AW, 0x03); //RX/TX Address width is set to 5 bytes
    nrf24_write(SETUP_RETR, 0x00); //Disable retransmit since not using auto ack
    nrf24_write(RF_CH, 0x00);// set up the channel in RX and TX functions
    nrf24_write(RF_SETUP, 0x0E); //Power of 0dB and Data Rate of 2Mbps


    ce_enable();

}

//Set the NRF24 to TX mode, and assigns it a channel and address
void setup_tx_mode(uint8_t *tx_address, uint8_t tx_channel)
{
    //set CE low to prevent the module from going into transmit or receive mode
    ce_disable();

    nrf24_write(RF_CH, tx_channel);
    nrf24_write_multibyte(TX_ADDR, tx_address,5);

    //assign the current config register data to a variable and use it to set the PWR_UP bit high
    uint8_t config_reg = nrf24_read(CONFIG);

    config_reg = config_reg | (1<<1); //sets PWR_UP bit high to enable TX mode
    nrf24_write(CONFIG, config_reg);

    ce_enable();
}



uint8_t tx(uint8_t *data)
{
    uint8_t status_reg = 0;
    uint8_t cmd = 0;
    uint8_t clr_tx_ds_irq = 0;

    //data transfer
    cs_select();
    cmd = W_TX_PAYLOAD;
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
    HAL_SPI_Transmit (NRF24_SPI, data, 32, 1000);
    cs_deselect(); //finished writing to device so deselect it.
    HAL_Delay(1); //1ms

    status_reg = nrf24_read(STATUS);

    //check if the status registers tx_ds irq bit is high to confirm data is transmitted
    // empty all data currently stored in TX FIFO and clear tx_ds irq
    if (status_reg & (1<<5))
    {
        cmd = FLUSH_TX;
        nrf24_cmd(cmd); //empty tx fifo
        clr_tx_ds_irq = status_reg | (1<<5); //write 1 to tx_ds irq bit to clear it
        nrf24_write(STATUS, clr_tx_ds_irq);
        if (!(nrf24_read(STATUS) & (1<<5))) return 1; //check that the tx_ds irq bit is cleared
    }

    return 0;

    }

//Set the NRF24 to RX mode, and assigns it a channel and address
void setup_rx_mode(uint8_t *rx_address, uint8_t rx_channel)
{
    //set CE low to prevent the module from going into transmit or receive mode
    ce_disable();

    nrf24_write(RF_CH, rx_channel);

    uint8_t en_data_pipe = nrf24_read(EN_RXADDR);
    en_data_pipe = en_data_pipe | (1<<1);
    nrf24_write(EN_RXADDR, en_data_pipe); //enable data pipe 1.

    nrf24_write_multibyte(RX_ADDR_P1, rx_address,5); //configured to use 5 byte address

    nrf24_write(RX_PW_P1, 32); //configured to use 32 byte payload for pipe 0

    uint8_t config_reg = nrf24_read(CONFIG);
    config_reg = config_reg | (1<<1) |(1<<0); //sets PWR_UP and PRIM_RX bits high to setup RX Mode

    nrf24_write(CONFIG, config_reg);

    ce_enable();
}

//Checks that data is available in the data pipe that was enabled
uint8_t data_available(int pipe_num)
{
    uint8_t status_reg = nrf24_read(STATUS);

    //check rx_dr irq is set high and the pipe that was enabled is available for reading from RX_FIFO
    if ((status_reg & (1<<6)) && (status_reg & (pipe_num << 1)))
    {
        uint8_t clear_rx_dr = 1<<6;
        nrf24_write(STATUS, clear_rx_dr);
        return 1;
    }

    return 0;

}

//Assigns data from the RX FIFO to a data buffer
void rx(uint8_t *data)
{
    uint8_t cmd = 0;

    cs_select();

    cmd = R_RX_PAYLOAD;
    HAL_SPI_Transmit (NRF24_SPI, &cmd, 1, 100);
    HAL_SPI_Receive (NRF24_SPI, data, 32, 1000);

    cs_deselect(); //finished writing to device so deselect it.

    HAL_Delay(1); //1ms

    cmd = FLUSH_RX;
    nrf24_cmd(cmd);

}





