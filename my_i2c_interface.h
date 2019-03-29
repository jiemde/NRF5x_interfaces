/* Copyright (c) 2018 Robert Rotsching */

#ifndef MY_I2C_INTERFACE_H
#define MY_I2C_INTERFACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//UNCOMMENT the defines after the functions are implemented in
//device_specific_libraries directory

void I2C_Init(uint32_t scl, uint32_t sda);

#define I2C_WRITE_BYTE
uint8_t I2C_write_byte(uint8_t slave_addr,
                    uint8_t reg_addr,
                    uint8_t buffer);

#define I2C_WRITE_BYTES
uint8_t I2C_write_bytes(uint8_t slave_addr,
                            uint8_t reg_addr,
                            uint8_t *buffer,
                            uint8_t len);

#define I2C_READ_BYTE
uint8_t I2C_read_byte(uint8_t slave_addr,
                            uint8_t reg_addr);

#define I2C_READ_BYTES
uint8_t I2C_read_bytes(uint8_t slave_addr,
                                uint8_t reg_addr,
                                uint8_t *buffer,
                                uint8_t len); 

#ifdef __cplusplus
}
#endif

#endif
