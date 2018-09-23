#include "my_chip_temperature.h"

/*******************************************************************************
* Function Name  : CHIP_get_temperature
* Description    : Measures the chip's internal temperature
* Input          : Nonde
* Output         : None
* Return         : The temeperature measured
*******************************************************************************/
int8_t CHIP_get_temperature(){

    uint8_t ret_data = 0x00;

    /* set start_task register to 0 */
    NRF_TEMP->TASKS_START = 0x0UL;

    /* set task stop register to 0 */
    NRF_TEMP->TASKS_STOP = 0x0UL;

    /* clear the event ready register to 0 */
    NRF_TEMP->EVENTS_DATARDY = 0x0UL;

    /* start the temperature measurement */
    NRF_TEMP->TASKS_START = 0x1UL;

    /* wait for transaction complete */
    while (NRF_TEMP->EVENTS_DATARDY == 0x0UL);

    /* stop the temperature measurement */
    NRF_TEMP->TASKS_STOP = 0x1UL;

    /* read temperature into register */
    ret_data = (int8_t)NRF_TEMP->TEMP / 4;

    /* clear tasks_start register */
    NRF_TEMP->TASKS_START = 0x0UL;

    /* clear tasks_stop register */
    NRF_TEMP->TASKS_STOP = 0x0UL;

    /* clear events ready register */
    NRF_TEMP->EVENTS_DATARDY = 0x0UL;

    return ret_data;
}

