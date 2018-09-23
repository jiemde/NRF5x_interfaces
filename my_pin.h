#ifndef MY_PIN_H
#define MY_PIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PIN_SET_PIN_OUTPUT
uint8_t PIN_set_pin_output(int pin);

#define PIN_SET_PIN_INPUT
uint8_t PIN_set_pin_input(int pin);

#define PIN_ENABLE_PULLUP
uint8_t PIN_enable_pullup(int pin);

#define PIN_ENABLE_PULLDOWN
uint8_t PIN_enable_pulldown(int pin);

#define PIN_ENABLE_NOPULL
uint8_t PIN_enable_nopull(int pin);

#define PIN_SET_PIN_HIGH
uint8_t PIN_set_pin_high(int pin);

#define PIN_SET_PIN_LOW
uint8_t PIN_set_pin_low(int pin);

#define PIN_READ_PIN
uint8_t PIN_read_pin(int pin);

#ifdef __cplusplus
}
#endif

#endif
