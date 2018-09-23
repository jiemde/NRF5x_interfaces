#include "my_pin.h"
#include "nrf_gpio.h"

/* Uncomment each function for implementation */

/*******************************************************************************
* Function Name  : PIN_set_pin_output
* Description    : Sets the desired pin to output
* Input          : pin = the pin desired to be set to output
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t PIN_set_pin_output(int pin){
    nrf_gpio_cfg_output(pin);
    return 0;
}

/*******************************************************************************
* Function Name  : PIN_set_pin_input
* Description    : Sets the desired pin to input
* Input          : pin = the pin desired to be handled
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t PIN_set_pin_input(int pin){
    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_NOPULL);
    return 0;
}

/*******************************************************************************
* Function Name  : PIN_enable_pullup
* Description    : Enables the internal pull-up resistor on the pin
* Input          : pin = the pin desired to be handled
* Output         : None
* Return         : Error code
*******************************************************************************/

uint8_t PIN_enable_pullup(int pin){
    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLUP);
    return 0;
}


/*******************************************************************************
* Function Name  : PIN_enable_pulldown
* Description    : Enables the internal pull-down resistor on the pin
* Input          : pin = the pin desired to be handled
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t PIN_enable_pulldown(int pin){
    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLDOWN);
    return 0;
}

/*******************************************************************************
* Function Name  : PIN_enable_nopull
* Description    : Disables any pull-up or pull-down resistor on the pin
* Input          : pin = the pin desired to be handled
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t PIN_enable_nopull(int pin){
    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_NOPULL);
    return 0;
}

/*******************************************************************************
* Function Name  : PIN_set_pin_high
* Description    : Sets the desired pin to high
* Input          : pin = the pin desired to be handled
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t PIN_set_pin_high(int pin){
    nrf_gpio_pin_set(pin);
    return 0;
}

/*******************************************************************************
* Function Name  : PIN_set_pin_low
* Description    : Sets the desired pin to low
* Input          : pin = the pin desired to be handled
* Output         : None
* Return         : Error code
*******************************************************************************/
uint8_t PIN_set_pin_low(int pin){
    nrf_gpio_pin_clear(pin);
    return 0;
}

/*******************************************************************************
* Function Name  : PIN_read_pin
* Description    : Reads the value on the pin
* Input          : pin = the pin desired to be handled
* Output         : None
* Return         : The value read on the pin
*******************************************************************************/
uint8_t PIN_read_pin(int pin){
    return (uint8_t)nrf_gpio_pin_read(pin);
}
