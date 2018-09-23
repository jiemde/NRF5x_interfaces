#ifndef MY_CHIP_TEMPERATURE_H
#define MY_CHIP_TEMPERATURE_H

#include <stdint.h>
#include "nrf52.h"

#ifdef __cplusplus
extern "C" {
#endif

int8_t CHIP_get_temperature();

#ifdef __cplusplus
}
#endif

#endif
