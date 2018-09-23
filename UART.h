#ifndef UART_H
#define UART_H

#include "ABSTRACT_UART.h"

class UART : public ABSTRACT_UART{

public:

	void init(uint32_t tx, uint32_t rx){
		/* set TX pin to output */
		NRF_GPIO->DIR |= (1 << tx);
		/* set RX pin to input */
		NRF_GPIO->DIR &= ~(1 << rx);
		NRF_UART0->PSELTXD = tx;
		/* set rx_pin */
		NRF_UART0->PSELRXD = rx;
		/* enable uart */
		NRF_UART0->ENABLE = 0x00000004;
		/* enable UART tranmitter */
		NRF_UART0->TASKS_STARTRX = 1;
		/* enable UART receiver */
		NRF_UART0->TASKS_STARTTX = 1;
		/* set the baud rate */
        /*
        BAUD1200 0x0004F000 1200 baud
        BAUD2400 0x0009D000 2400 baud
        BAUD4800 0x0013B000 4800 baud
        BAUD9600 0x00275000 9600 baud
        BAUD14400 0x003B0000 14400 baud
        BAUD19200 0x004EA000 19200 baud
        BAUD28800 0x0075F000 28800 baud
        BAUD38400 0x009D5000 38400 baud
        BAUD57600 0x00EBF000 57600 baud
        BAUD76800 0x013A9000 76800 baud
        BAUD115200 0x01D7E000 115200 baud
        BAUD230400 0x03AFB000 230400 baud
        BAUD250000 0x04000000 250000 baud
        BAUD460800 0x075F7000 460800 baud
        BAUD1M 0x10000000 1 megabaud
        */
		NRF_UART0->BAUDRATE = 0x00275000;
		/* parity_bit */
		NRF_UART0->CONFIG |= 0x00000000;
	}

    uint8_t rx(uint8_t *buffer){
		return 0;
	}

    uint8_t tx(uint8_t c){
		/* fill tx register */
		NRF_UART0->TXD = (uint8_t)c & 0xFF;

		/* wait for data to be transmitted */
		while (NRF_UART0->EVENTS_TXDRDY != 1){}

		/* clear data tx sent register */
		NRF_UART0->EVENTS_TXDRDY = 0;
		return 0;
	}

    uint8_t tx_string(char* text, int len){

		char buffer[len + 1];
		
		/* set the buffer to 0 */
		memset(buffer, 0, len + 1);
		strcpy(buffer, text);

		/* send carriage return, line feed and data */
		tx(13); //CR
		tx(10); //LF
		for (int i = 0 ; i < len + 1 ; i++)
		    tx((uint8_t)buffer[i]);
		tx(13); //CR
		tx(10); //LF
		return 0;
	}

    uint8_t enable_interrupt(){return 0;}

    uint8_t disable_interrupt(){return 0;}

    uint8_t enable(){return 0;}

    uint8_t disable(){return 0;}

};

#endif
