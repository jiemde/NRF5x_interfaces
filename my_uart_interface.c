#include "my_uart_interface.h"
#include "nrf52.h"
#include "my_delay.h"
#include "my_pin.h"
#include <string.h>

/* WARNING: Uncomment the function in order to implement it */

/*******************************************************************************
* Function Name  : UART_rx
* Description    : Receives a byte from the UART interface
* Input          : None
* Output         : None
* Return         : Byte received
*******************************************************************************/
/*
uint8_t UART_rx(){
    return 0;
}
*/

uint8_t UART_Init(uint32_t tx_pin, uint32_t rx_pin, uint32_t rts_pin, uint32_t cts_pin){

    /* Disable UART */
    /* The pins can only be configured while the UART is disabled */
    UART_disable();

    /* Configure GPIO Pins as output and input */
    /* 
        RXD INPUT
        CTS INPUT
        RTS OUTPUT
        TXD OUTPUT
    */
    PIN_set_pin_input(rx_pin);
    PIN_set_pin_input(cts_pin);
    PIN_set_pin_output(rts_pin);
    PIN_set_pin_output(tx_pin);

    /* Set PSEL registers for the Pins. Value 0xFFFFFFFF makes the signal
        unconnected from the physical pin */

    NRF_UART0->PSELRXD = rx_pin;
    NRF_UART0->PSELCTS = cts_pin;
    NRF_UART0->PSELRTS = rts_pin;
    NRF_UART0->PSELTXD = tx_pin;

    UART_disable_interrupts();
    UART_set_baudrate(9600);
    UART_enable_hardware_flow_control();
    UART_enable();

    return 0;
}

uint8_t UART_available(){
    if (NRF_UART0->EVENTS_RXDRDY == 0x1UL){
        return 1;
    }
    return 0;
}

uint8_t UART_start_receiving(){
    NRF_UART0->TASKS_STARTRX = 0x1UL;
    return 0;
}

uint8_t UART_receive_byte(){
    uint8_t data = 0x00;
    NRF_UART0->EVENTS_TXDRDY = 0x0UL;
    NRF_UART0->EVENTS_ERROR  = 0x0UL;

    while ((NRF_UART0->EVENTS_RXDRDY ==0x0UL));

    data = NRF_UART0->RXD;

    NRF_UART0->EVENTS_RXDRDY = 0x0UL;
    
    return data;
}

uint8_t UART_stop_receiving(){
    NRF_UART0->TASKS_STOPRX = 0x1UL;
    return 0;
}

uint8_t UART_wait_for_RXTO(uint8_t* buffer){
    int i = 0;
    while (1){
        while ((NRF_UART0->EVENTS_RXTO == 0x0UL) || (NRF_UART0->EVENTS_RXDRDY == 0x0UL));
        if (NRF_UART0->EVENTS_RXTO == 0x1UL){
            NRF_UART0->EVENTS_RXTO = 0x1UL;
            return 0;
        }
        if (NRF_UART0->EVENTS_RXDRDY == 0x1UL){
            *(buffer + i) = NRF_UART0->RXD;
            NRF_UART0->EVENTS_RXDRDY = 0x0UL;
            i++;
        }
    }
}

uint8_t UART_send_byte(uint8_t data){
    NRF_UART0->EVENTS_TXDRDY = 0x0UL;
    NRF_UART0->EVENTS_RXDRDY = 0x0UL;
    NRF_UART0->EVENTS_ERROR  = 0x0UL;
    
    NRF_UART0->TASKS_STARTTX = 0x1UL;
    NRF_UART0->TXD = data;

    /* Wait until data is sent or an error occurs */
    
    while ((NRF_UART0->EVENTS_TXDRDY == 0x0UL));
    NRF_UART0->EVENTS_TXDRDY = 0x0UL;

    NRF_UART0->TASKS_STOPTX = 0x1UL;
 
    return 0;
}

uint8_t UART_send_bytes(uint8_t* data, int len){
    int i = 0;

    NRF_UART0->EVENTS_TXDRDY = 0x0UL;
    NRF_UART0->EVENTS_RXDRDY = 0x0UL;
    NRF_UART0->EVENTS_ERROR  = 0x0UL;
    
    NRF_UART0->TASKS_STARTTX = 0x1UL;

    for (i = 0 ; i < len ; i++){

        NRF_UART0->TXD = *(data + i);
        while ((NRF_UART0->EVENTS_TXDRDY == 0x0UL));
        NRF_UART0->EVENTS_TXDRDY = 0x0UL;
    }

    NRF_UART0->TASKS_STOPTX = 0x1UL;
 
    return 0;
}

uint8_t UART_print(char *str){
    UART_send_bytes((uint8_t*)str, strlen(str));
    return 0;
}

uint8_t UART_enable(){
    NRF_UART0->ENABLE = 0x04;
    return 0;
}

uint8_t UART_disable(){
    NRF_UART0->ENABLE = 0x00;
    return 0;
}

uint8_t UART_enable_CTS_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENSET;
    data |= (1 << 0);
    NRF_UART0->INTENSET = data;
    return 0;
}

uint8_t UART_enable_NCTS_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENSET;
    data |= (1 << 1);
    NRF_UART0->INTENSET = data;
    return 0;
}

uint8_t UART_enable_RXDRDY_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENSET;
    data |= (1 << 2);
    NRF_UART0->INTENSET = data;
    return 0;
}

uint8_t UART_enable_TXDRDY_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENSET;
    data |= (1 << 7);
    NRF_UART0->INTENSET = data;
    return 0;
}

uint8_t UART_enable_ERROR_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENSET;
    data |= (1 << 9);
    NRF_UART0->INTENSET = data;
    return 0;
}

uint8_t UART_enable_RXTO_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENSET;
    data |= (1 << 17);
    NRF_UART0->INTENSET = data;
    return 0;
}

uint8_t UART_disable_CTS_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENCLR;
    data |= (1 << 0);
    NRF_UART0->INTENCLR = data;
    return 0;
}

uint8_t UART_disable_NCTS_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENCLR;
    data |= (1 << 1);
    NRF_UART0->INTENCLR = data;
    return 0;
}

uint8_t UART_disable_RXDRDY_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENCLR;
    data |= (1 << 2);
    NRF_UART0->INTENCLR = data;
    return 0;
}

uint8_t UART_disable_TXDRDY_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENCLR;
    data |= (1 << 7);
    NRF_UART0->INTENCLR = data;
    return 0;
}

uint8_t UART_disable_ERROR_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENCLR;
    data |= (1 << 9);
    NRF_UART0->INTENCLR = data;
    return 0;
}

uint8_t UART_disable_RXTO_interrupt(){
    uint32_t data = 0x00;
    data = NRF_UART0->INTENCLR;
    data |= (1 << 17);
    NRF_UART0->INTENCLR = data;
    return 0;
}

uint8_t UART_enable_interrupts(){
    uint32_t data = 0x00;
    data |= (1 << 0);
    data |= (1 << 1);
    data |= (1 << 2);
    data |= (1 << 7);
    data |= (1 << 9);
    data |= (1 << 17);
    NRF_UART0->INTENSET = data;
    return 0;
}

uint8_t UART_disable_interrupts(){
    uint32_t data = 0x00;
    data |= (1 << 0);
    data |= (1 << 1);
    data |= (1 << 2);
    data |= (1 << 7);
    data |= (1 << 9);
    data |= (1 << 17);
    NRF_UART0->INTENCLR = data;
    return 0;
}

uint8_t UART_set_baudrate(uint32_t baud){
    uint32_t data = 0x00;
    if (baud == 1200){
        data = 0x0004F000;
    }else
    if (baud == 2400){
        data = 0x0009D000;
    }else
    if (baud == 4800){
        data = 0x0013B000;
    }else
    if (baud == 9600){
        data = 0x00275000;
    }else
    if (baud == 14400){
        data = 0x003B0000;
    }else
    if (baud == 19200){
        data = 0x004EA000;
    }else
    if (baud == 28800){
        data = 0x0075F000;
    }else
    if (baud == 38400){
        data = 0x009D5000;
    }else
    if (baud == 57600){
        data = 0x00EBF000;
    }
    if (baud == 76800){
        data = 0x013A9000;
    }else
    if (baud == 115200){
        data = 0x01D7E000;
    }else
    if (baud == 230400){
        data = 0x03AFB000;
    }else
    if (baud == 250000){
        data = 0x04000000;
    }else
    if (baud == 460800){
        data = 0x075F7000;
    }else
    if (baud == 921600){
        data = 0x0EBED000;
    }else
    if (baud == 1000000){
        data = 0x10000000;
    }
    NRF_UART0->BAUDRATE = data;
    return 0;
}

uint8_t UART_enable_hardware_flow_control(){
    uint32_t data = 0x00;
    data = NRF_UART0->CONFIG;
    data |= (1 << 0);
    NRF_UART0->CONFIG = data;
    return 0;
}

uint8_t UART_disable_hardware_flow_control(){
    uint32_t data = 0x00;
    data = NRF_UART0->CONFIG;
    data &= ~(1 << 0);
    NRF_UART0->CONFIG = data;
    return 0;
}

uint8_t UART_enable_parity_bit(){
    uint32_t data = 0x00;
    data = NRF_UART0->CONFIG;
    data |= (0x07 << 1);
    NRF_UART0->CONFIG = data;
    return 0;
}

uint8_t UART_disable_parity_bit(){
    uint32_t data = 0x00;
    data = NRF_UART0->CONFIG;
    data &= ~(0x07 << 1);
    NRF_UART0->CONFIG = data;
    return 0;
}

uint8_t UART_read_and_send_line(){
    UART_start_receiving();
    char buffer[100];
    memset(buffer, 0, sizeof(buffer));
    int i = 0;
    while(1){
        char a;
        a = UART_receive_byte();
        if (a == 13){
            UART_send_byte(10);
            UART_send_byte(13);
            UART_send_bytes((uint8_t*)buffer, sizeof(buffer));
            i++;
            buffer[i] = '\0';
            UART_send_byte(10);
            UART_send_byte(13);
            i = 0;
            memset(buffer, 0, sizeof(buffer));
            break;
        }else{
            buffer[i] = a;
            i++;
            UART_send_byte((uint8_t) a);
        }
    }
    return 0;
}

uint8_t UART_read_line(uint8_t* buffer, int len){
    UART_start_receiving();
    memset(buffer, 0, len);
    int i = 0;
    while(1){
        char a;
        a = UART_receive_byte();
        if (a == 13){
            i++;
            buffer[i] = '\0';
            break;
        }else{
            buffer[i] = a;
            i++;
            UART_send_byte((uint8_t) a);
        }
    }
    return 0;
}

uint8_t UART_println(uint8_t* buffer, int len){
    UART_send_byte(10);
    UART_send_byte(13);
    UART_send_bytes((uint8_t*)buffer, len);
    buffer[len-1] = '\0';
    UART_send_byte(10);
    UART_send_byte(13);
    return 0;
}
