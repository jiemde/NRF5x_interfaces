#include "my_chip_uuid.h"
#include <string.h>

uint16_t CHIP_get_uuid(){
    uint16_t uuid = 0x0000;
    uint8_t uuid_vect[2];
    uuid_vect[0] = NRF_FICR->DEVICEID[0];
    uuid_vect[1] = NRF_FICR->DEVICEID[1];
    memcpy(&uuid, uuid_vect, sizeof(uuid));
    return uuid;
}


