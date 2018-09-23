#include "my_delay.h"
#include "nrf_delay.h"

/* Uncomment this function and implement it on device_specific_library */
/* file must be copied to device_specific_libraries directory and renamed
to  my_delay.h and implemented in my_delay.c*/


/*******************************************************************************
* Function Name  : DELAY_delay_ms
* Description    : Delay the program for the amount of time specified in milisecs
* Input          : time : the time period for the program to halt
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t DELAY_delay_ms(int time){
    nrf_delay_ms(time);
    return 0;
}
