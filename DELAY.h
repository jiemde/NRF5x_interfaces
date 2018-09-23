#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>
#include "ABSTRACT_DELAY.h"
#include "nrf_delay.h"

class DELAY : public ABSTRACT_DELAY{

public:

    uint8_t delay_ms(int time){
        nrf_delay_ms(time);
        return 0;
    }

};

#endif
