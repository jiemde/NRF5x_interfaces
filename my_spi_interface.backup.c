#include "my_spi_interface.h"
#include "nrf52.h"
#include "nrf_gpio.h"

/* WARNING: Uncomment function to implement it */

/* WARNING: The ss pin is -1 if it is not used and must be used as variable
            for external libraries which use spi */

uint32_t sck;
uint32_t mosi;
uint32_t miso;
int32_t ss;

void SPI_Init(uint32_t sck_pin,
              uint32_t mosi_pin,
              uint32_t miso_pin,
              int32_t ss_pin,
              uint8_t mode,
              uint8_t bit_order){

        sck = sck_pin;
        mosi = mosi_pin;
        miso = miso_pin;
        ss = ss_pin;

        /* configure the sck pin */
        if (mode <= 1){
            nrf_gpio_pin_clear(sck);
        }else{
            nrf_gpio_pin_set(sck);
        }
        nrf_gpio_cfg(sck,   NRF_GPIO_PIN_DIR_OUTPUT,
                            NRF_GPIO_PIN_INPUT_CONNECT,
                            NRF_GPIO_PIN_NOPULL,
                            NRF_GPIO_PIN_S0S1,
                            NRF_GPIO_PIN_NOSENSE);

        /* configure mosi pin */
        nrf_gpio_pin_clear(mosi);
        nrf_gpio_cfg_output(mosi);

        /* configure miso pin */
        nrf_gpio_cfg_input(miso, NRF_GPIO_PIN_NOPULL);

        /* configure pin registers */
        NRF_SPI1->PSELSCK = sck;
        NRF_SPI1->PSELMOSI = mosi;
        NRF_SPI1->PSELMISO = miso;

        /* set frequency 500 kbps */
        NRF_SPI1->FREQUENCY = 0x08000000;

        /* select bit order and mode */
        if (bit_order == 0){

            /* select MSB_First bit order */
            NRF_SPI1->CONFIG = 0;

        }else{

            /* select LSB_First bit order */
            NRF_SPI1->CONFIG = 1;
        }

        /* select mode */
        switch (mode){
            default:
            case 0:
                NRF_SPI1->CONFIG |= (0 << 2);
                NRF_SPI1->CONFIG |= (0 << 1);
                break;
            case 1:
                NRF_SPI1->CONFIG |= (0 << 2);
                NRF_SPI1->CONFIG |= (1 << 1);
                break;
            case 2:
                NRF_SPI1->CONFIG |= (1 << 2);
                NRF_SPI1->CONFIG |= (0 << 1);
                break;
            case 3:
                NRF_SPI1->CONFIG |= (1 << 2);
                NRF_SPI1->CONFIG |= (1 << 1);
                break;
        }

        /* enable the spi module */
        NRF_SPI1->ENABLE = 0x1UL;

        return;
}


/*******************************************************************************
* Function Name  : SPI_write_byte
* Description    : Writes a byte to a certain address using SPI
* Input          : reg_addrss register address, buffer=value to be written
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t SPI_write_byte(uint8_t reg_addr, uint8_t buffer){
    SPI_write_bytes(reg_addr, &buffer, 1);
    return 0;
}

/*******************************************************************************
* Function Name  : SPI_write_bytes
* Description    : Writes multiple bytes to a certain address using SPI
* Input          : reg_addrss register address, buffer=holds the values, len=length
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t SPI_write_bytes(uint8_t reg_addr, uint8_t *buffer, uint8_t len){
    
    uint8_t nop = 0x00;

    /* set ss pin to low */
    SPI_set_cs_low();

    /* clear events ready register */
    NRF_SPI1->EVENTS_READY = 0x0UL;

    /* send the register address */
    NRF_SPI1->TXD = (uint32_t)reg_addr;

    /* wait for transaction to happen */
    while (NRF_SPI1->EVENTS_READY == 0x0UL);

    /* read the rxd register to continue */
    nop = NRF_SPI1->RXD;

    /* clear the events ready register */
    NRF_SPI1->EVENTS_READY = 0x0UL;

    /* send the bytes in buffer */
    for (uint32_t i = 0 ; i < len ; i++){
    
        /* fill the txd register */
        NRF_SPI1->TXD = buffer[i];

        /* wait for transaction complete */
        while (NRF_SPI1->EVENTS_READY == 0x0UL);

        /* read the rxd register */
        nop = NRF_SPI1->RXD; nop = nop;

        /* clear the events ready register */
        NRF_SPI1->EVENTS_READY = 0x0UL;
    }

    /* set the ss pin to high */
    SPI_set_cs_high();

    return 0;
}

/*******************************************************************************
* Function Name  : SPI_read_byte
* Description    : Reads a byte from a certain address using SPI
* Input          : reg_addrss register address
* Output         : None
* Return         : Byte read
*******************************************************************************/
uint8_t SPI_read_byte(uint8_t reg_addr){
    uint8_t buffer = 0x00;
    SPI_read_bytes(reg_addr, &buffer, 1);
    return buffer;
}

/*******************************************************************************
* Function Name  : SPI_read_bytes
* Description    : Reads bytes from a certain address using SPI
* Input          : reg_addrss register address, len=length to be read
* Output         : buffer holds the values read from address
* Return         : Error code
*******************************************************************************/
uint8_t SPI_read_bytes(uint8_t reg_addr, uint8_t *buffer, uint8_t len){

    uint8_t nop = 0x00;

    /* set ss pin to low */
    SPI_set_cs_low();

    /* clear ready event */
    NRF_SPI1->EVENTS_READY = 0x0UL;

    /* write the reg address in txd */
    NRF_SPI1->TXD = (uint32_t)reg_addr;

    /* wait for transaction complete */
    while (NRF_SPI1->EVENTS_READY == 0x0UL);

    /* read the rxd register to take the next transaction */
    nop = NRF_SPI1->RXD; nop = nop;

    /* clear the events ready register */
    NRF_SPI1->EVENTS_READY = 0x0UL;

    /* read bytes in buffer */
    for (uint32_t i = 0 ; i < len ; i++){

        /* fill the tx buffer with 0 to create transaction*/
        NRF_SPI1->TXD = 0x00;

        /* wait for transaction complete */
        while (NRF_SPI1->EVENTS_READY == 0x0UL);

        /* read a byte in a buffer location */
        buffer[i] = (uint8_t)NRF_SPI1->RXD;

        /* clear the events ready register */
        NRF_SPI1->EVENTS_READY = 0x0UL;
    }

    /* set ss pin to high */
    SPI_set_cs_high();

    return 0;
}

/*******************************************************************************
* Function Name  : SPI_send_byte
* Description    : Sends a byte to the peripheral device
* Input          : buffer holds the value to be sent
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t SPI_send_byte(uint8_t buffer){
    SPI_send_bytes(&buffer, 1);
    return 0;
}

/*******************************************************************************
* Function Name  : SPI_send_bytes
* Description    : Sends bytes to the peripheral device
* Input          : buffer holds the values to be sent and len the length
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t SPI_send_bytes(uint8_t *buffer, uint8_t len){

    uint8_t nop = 0;

    /* set ss pin to low */
    SPI_set_cs_low();

    /* clear the events ready register */
    NRF_SPI1->EVENTS_READY = 0x0UL;

    /* send the bytes in buffer */
    for (uint32_t i = 0 ; i < len ; i++){
    
        /* fill the txd register */
        NRF_SPI1->TXD = buffer[i];

        /* wait for transaction complete */
        while (NRF_SPI1->EVENTS_READY == 0x0UL);

        /* read the rxd register */
        nop = NRF_SPI1->RXD; nop = nop;

        /* clear the events ready register */
        NRF_SPI1->EVENTS_READY = 0x0UL;
    }

    /* set ss pin to high */
    SPI_set_cs_high();

    return 0;
}

/*******************************************************************************
* Function Name  : SPI_receive_byte
* Description    : Receives a byte from the peripheral device
* Input          : None
* Output         : None
* Return         : Byte received
*******************************************************************************/
uint8_t SPI_receive_byte(){
    uint8_t buffer = 0x00;
    SPI_receive_bytes(&buffer, 1);
    return buffer;
}

/*******************************************************************************
* Function Name  : SPI_receive_bytes
* Description    : Receives bytes from the peripheral device
* Input          : buffer holds the values being received and len the length
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t SPI_receive_bytes(uint8_t *buffer, uint8_t len){

    /* set ss pin to low */
    SPI_set_cs_low();

    /* clear ready event */
    NRF_SPI1->EVENTS_READY = 0x0UL;

    /* read bytes in buffer */
    for (uint32_t i = 0 ; i < len ; i++){

        /* fill the tx buffer with 0 to create transaction*/
        NRF_SPI1->TXD = 0x00;

        /* wait for transaction complete */
        while (NRF_SPI1->EVENTS_READY == 0x0UL);

        /* read a byte in a buffer location */
        buffer[i] = (uint8_t)NRF_SPI1->RXD;

        /* clear the events ready register */
        NRF_SPI1->EVENTS_READY = 0x0UL;
    }

    /* set ss pin to high */
    SPI_set_cs_high();

    return 0;
}

/*******************************************************************************
* Function Name  : SPI_set_cs_low()
* Description    : Sets the chip select pin to low
* Input          : None
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t SPI_set_cs_low(){
    if (ss != -1){
        nrf_gpio_pin_clear(ss);
    } 
    return 0;
}

/*******************************************************************************
* Function Name  : SPI_set_cs_high()
* Description    : Sets the chip select pin to high()
* Input          : None
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t SPI_set_cs_high(){
    if (ss != -1){
        nrf_gpio_pin_set(ss);
    } 
    return 0;
}

