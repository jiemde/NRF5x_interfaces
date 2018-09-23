#ifndef MY_UART_INTERFACE_H
#define MY_UART_INTERFACE_H

#include "nrf52.h"
#include "nrf_gpio.h"
#include "my_delay.h"
#include <string.h>
#include <stdio.h>

#define BUFLEN 10

void UART_Init();
void UART_tx_char(char c);
void UART_tx_text(char* text);

/*******************************************************************************
* Function Name  : LED_low
* Description    : Sets the LED that is on the board to low
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART_Init(){

    uint32_t tx = 19;
    uint32_t rx = 20;

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
    NRF_UART0->BAUDRATE = 0x00275000;
    /* parity_bit */
    NRF_UART0->CONFIG |= 0x00000000;
}

/*******************************************************************************
* Function Name  : UART_tx_char
* Description    : Sends a character on the UART
* Input          : c is the character that is sent on the UART
* Output         : None
* Return         : None
*******************************************************************************/
void UART_tx_char(char c){
    /* fill tx register */
    NRF_UART0->TXD = (uint8_t)c & 0xFF;

    /* wait for data to be transmitted */
    while (NRF_UART0->EVENTS_TXDRDY != 1){}

    /* clear data tx sent register */
    NRF_UART0->EVENTS_TXDRDY = 0;
}

/*******************************************************************************
* Function Name  : UART_tx_text
* Description    : Sends a text on the UART
* Input          : text (char*) that will be sent on the UART
* Output         : None
* Return         : None
*******************************************************************************/
void UART_tx_text(char* text){
    char buffer[BUFLEN];
    
    /* set the buffer to 0 */
    memset(buffer, 0, BUFLEN);
    strcpy(buffer, text);

    /* send carriage return, line feed and data */
    //UART_tx_char(13); //CR
    //UART_tx_char(10); //LF
    for (int i = 0 ; i < BUFLEN ; i++)
        UART_tx_char((uint8_t)buffer[i]);
    //UART_tx_char(13); //CR
    //UART_tx_char(10); //LF
}

#endif
