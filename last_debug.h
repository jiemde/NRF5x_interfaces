#ifndef MY_DEBUG_H
#define MY_DEBUG_H

#include "my_delay.h"
#include "my_uart_interface.h"
#include "my_led.h"

void debug();
void print(uint32_t reg);

/*******************************************************************************
* Function Name  : debug
* Description    : Lights up the board LED for 2 seconds
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void debug(){
    LED_high();
    delay_ms(2000);
    LED_low();
}

/*******************************************************************************
* Function Name  : print
* Description    : Prints an uint32_t on the serial port in binary for debug
* Input          : reg - 32 bit register name
* Output         : None
* Return         : None
*******************************************************************************/
void print(uint32_t reg){
    char buffer[BUFLEN];
    uint8_t byte = reg;
    sprintf(buffer, "%c%c%c%c %c%c%c%c",   (byte & 0x80 ? '1' : '0'), \
                                          (byte & 0x40 ? '1' : '0'), \
                                          (byte & 0x20 ? '1' : '0'), \
                                          (byte & 0x10 ? '1' : '0'), \
                                          (byte & 0x08 ? '1' : '0'), \
                                          (byte & 0x04 ? '1' : '0'), \
                                          (byte & 0x02 ? '1' : '0'), \
                                          (byte & 0x01 ? '1' : '0'));
    for (int i = 0 ; i < 100 ; i++){
        UART_tx_char(13);
        UART_tx_char(10);
        UART_tx_text(buffer);
        UART_tx_char(13);
        UART_tx_char(10);
        delay_ms(250);
    }
}

#endif
