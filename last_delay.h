#ifndef MY_DELAY_H
#define MY_DELAY_H

#include "nrf_delay.h"

/*******************************************************************************
* Function Name  : delay_ms
* Description    : Delay the program for the amount of time specified
* Input          : time : the time period for the program to halt
* Output         : None
* Return         : None
*******************************************************************************/
void delay_ms(uint32_t time){
    nrf_delay_ms(time);
}

#endif
