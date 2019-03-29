/* Copyright (c) 2018 Robert Rotsching */

#ifndef MY_SPI_INTERFACE_H
#define MY_SPI_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* WARNING: Uncomment each define and function after they are implemented in
    the .c file */

/* WARNING: Set the ss pin to -1 if you want to not be used automatically */

void SPI_Init(uint32_t sck,
              uint32_t mosi,
              uint32_t miso,
              uint8_t mode,
              uint8_t bit_order);

#define SPI_WRITE_BYTE
uint8_t SPI_write_byte(uint8_t reg_addr, uint8_t buffer);

#define SPI_WRITE_BYTES
uint8_t SPI_write_bytes(uint8_t reg_addr, uint8_t *buffer, uint8_t len);

#define SPI_READ_BYTE
uint8_t SPI_read_byte(uint8_t reg_addr);

#define SPI_READ_BYTES
uint8_t SPI_read_bytes(uint8_t reg_addr, uint8_t *buffer, uint8_t len);

#define SPI_SEND_BYTE
uint8_t SPI_send_byte(uint8_t buffer);

#define SPI_SEND_BYTES
uint8_t SPI_send_bytes(uint8_t *buffer, uint8_t len);

#define SPI_RECEIVE_BYTE
uint8_t SPI_receive_byte();

#define SPI_RECEIVE_BYTES
uint8_t SPI_receive_bytes(uint8_t *buffer, uint8_t len);
/*
#define SPI_SET_CS_LOW
uint8_t SPI_set_cs_low();

#define SPI_SET_CS_HIGH
uint8_t SPI_set_cs_high();
*/
#ifdef __cplusplus
}
#endif

#endif
