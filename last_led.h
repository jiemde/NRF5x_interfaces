#ifndef MY_LED_H
#define MY_LED_H

#include "nrf52.h"
#include "nrf_gpio.h"
#include "my_delay.h"

void LED_Init();
void LED_high();
void LED_toggle();
void LED_low();

/*******************************************************************************
* Function Name  : LED_Init
* Description    : Initializes the LED on the board
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LED_Init(){
    /* Set pin to output */
    NRF_GPIO->DIR = 0x0 | (1 << 24);

    /* Set pin to low */
    NRF_GPIO->OUTSET &= ~(1 << 24);
}

/*******************************************************************************
* Function Name  : LED_high
* Description    : Sets the LED that is on the board to high
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LED_high(){
    nrf_gpio_pin_write(24, 1);
}

/*******************************************************************************
* Function Name  : LED_toggle
* Description    : toggle the LED located on the board on pin 24
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LED_toggle(){
    nrf_gpio_pin_toggle(24);
}

/*******************************************************************************
* Function Name  : LED_low
* Description    : Sets the LED that is on the board to low
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LED_low(){
    nrf_gpio_pin_write(24, 0);
}

#endif
