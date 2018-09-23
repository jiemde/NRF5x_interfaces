#ifndef MY_SPI_INTERFACE_H
#define MY_SPI_INTERFACE_H

#include "nrf52.h"
#include "nrf_gpio.h"
#include "my_delay.h"
#include <stdint.h>

void SPI_Init(uint8_t mode, uint8_t bit_order);
uint8_t SPI_read_byte(uint32_t ss_pin, uint8_t reg_addr);
void SPI_read_bytes(uint32_t ss_pin, uint8_t reg_addr, uint8_t* buffer, uint32_t len);
void SPI_write_byte(uint32_t ss_pin, uint8_t reg_addr, uint8_t value);
void SPI_write_bytes(uint32_t ss_pin, uint8_t reg_addr, uint8_t* buffer, uint32_t len);

/*******************************************************************************
* Function Name  : SPI_Init
* Description    : //TODO
* Input          : //TODO
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_Init(uint8_t mode, uint8_t bit_order){

    /* bit_order : 0 MSB_First, 1 LSB_First */

    /* PIN SETUP */
    uint32_t sck = 4;
    uint32_t mosi = 5;
    uint32_t miso = 6;

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
* Function Name  : SPI_read_byte
* Description    : //TODO
* Input          : //TODO
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t SPI_read_byte(uint32_t ss, uint8_t reg_addr){
    uint8_t ret_data = 0x00;
    SPI_read_bytes(ss, reg_addr, &ret_data, 1);
    return ret_data;
}

/*******************************************************************************
* Function Name  : SPI_read_bytes
* Description    : //TODO
* Input          : //TODO
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_read_bytes(uint32_t ss_pin, uint8_t reg_addr, uint8_t* buffer, uint32_t len){

    uint8_t nop = 0x00;

    /* set ss pin to low */
    nrf_gpio_pin_clear(ss_pin);

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
    nrf_gpio_pin_set(ss_pin);

    return;
}

/*******************************************************************************
* Function Name  : SPI_write_byte
* Description    : //TODO
* Input          : //TODO
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_write_byte(uint32_t ss_pin, uint8_t reg_addr, uint8_t value){
    uint8_t buff = 0x00;
    SPI_write_bytes(ss_pin, reg_addr, &buff, 1);
    return;
}

/*******************************************************************************
* Function Name  : SPI_write_bytes
* Description    : //TODO
* Input          : //TODO
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_write_bytes(uint32_t ss_pin, uint8_t reg_addr, uint8_t* buffer, uint32_t len){

   uint8_t nop = 0x00;

    /* set ss pin to low */
    nrf_gpio_pin_clear(ss_pin);

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
    nrf_gpio_pin_set(ss_pin);

    return;
}

#endif
