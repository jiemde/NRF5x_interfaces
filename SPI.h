#ifndef SPI_H
#define SPI_H

#include <stdbool.h>
#include <stdint.h>
#include "PIN.h"
#include "ABSTRACT_SPI.h"
#include "nrf_gpio.h"

class SPI : public ABSTRACT_SPI{

public:

	/* mode and bit order usually 0,0 */
	/* ss must be -1 if it is not used */
	void init(uint32_t sck,
			  uint32_t mosi,
			  uint32_t miso,
			  int32_t ss,
			  uint8_t mode,
			  uint8_t bit_order){

		this->sck = sck;
		this->mosi = mosi;
		this->miso = miso;
		this->ss = ss;

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

    uint8_t write_byte(uint8_t reg_addr, uint8_t buffer){
    	write_bytes(reg_addr, &buffer, 1);
		return 0;
	}

    uint8_t write_bytes(uint8_t reg_addr, uint8_t *buffer, uint8_t len){

		uint8_t nop = 0x00;

		/* set ss pin to low */
		set_cs_low();

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
		set_cs_high();

		return 0;
	}

    uint8_t read_byte(uint8_t reg_addr, uint8_t *buffer){
	    read_bytes(reg_addr, buffer, 1);
		return 0;
	}

    uint8_t read_bytes(uint8_t reg_addr, uint8_t *buffer, uint8_t len){

		uint8_t nop = 0x00;

		/* set ss pin to low */
		set_cs_low();

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
		set_cs_high();

		return 0;
	}

    uint8_t send_byte(uint8_t buffer){return 0;}

    uint8_t send_bytes(uint8_t *buffer, uint8_t len){return 0;}

    uint8_t receive_byte(uint8_t *buffer){return 0;}

    uint8_t receive_bytes(uint8_t *buffer, uint8_t len){return 0;}

    uint8_t set_cs_low(){
		if (this->ss != -1){
			nrf_gpio_pin_clear(this->ss);
		}
		return 0;
	}

    uint8_t set_cs_high(){
		if (this->ss != -1){
			nrf_gpio_pin_set(this->ss);
		}
		return 0;
	}

    uint8_t uninit(){return 0;}

private:
	uint32_t sck;
	uint32_t mosi;
	uint32_t miso;
	int32_t ss;

};

#endif
