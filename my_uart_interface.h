#ifndef MY_UART_INTERFACE_H
#define MY_UART_INTERFACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* WARNING: Uncomment the define and function name after implemented in .c file */

uint8_t UART_Init(uint32_t tx_pin, uint32_t rx_pin, uint32_t rts_pin, uint32_t cts_pin);

uint8_t UART_available();

uint8_t UART_start_receiving();

uint8_t UART_receive_byte();

uint8_t UART_stop_receiving();

uint8_t UART_wait_for_RXTO(uint8_t* buffer);

uint8_t UART_send_byte(uint8_t data);

uint8_t UART_send_bytes(uint8_t* data, int len);

uint8_t UART_print(char* str);

uint8_t UART_enable();

uint8_t UART_disable();

uint8_t UART_enable_CTS_interrupt();

uint8_t UART_enable_NCTS_interrupt();

uint8_t UART_enable_RXDRDY_interrupt();

uint8_t UART_enable_TXDRDY_interrupt();

uint8_t UART_enable_ERROR_interrupt();

uint8_t UART_enable_RXTO_interrupt();

uint8_t UART_disable_CTS_interrupt();

uint8_t UART_disable_NCTS_interrupt();

uint8_t UART_disable_RXDRDY_interrupt();

uint8_t UART_disable_TXDRDY_interrupt();

uint8_t UART_disable_ERROR_interrupt();

uint8_t UART_disable_RXTO_interrupt();

uint8_t UART_enable_interrupts();

uint8_t UART_disable_interrupts();

uint8_t UART_set_baudrate(uint32_t baud);

uint8_t UART_enable_hardware_flow_control();

uint8_t UART_disable_hardware_flow_control();

uint8_t UART_enable_parity_bit();

uint8_t UART_disable_parity_bit();


#ifdef __cplusplus
}
#endif

#endif
