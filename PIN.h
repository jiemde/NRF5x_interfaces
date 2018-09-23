#ifndef PIN_H
#define PIN_H

#include <stdint.h>
#include "ABSTRACT_PIN.h"

class PIN : public ABSTRACT_PIN{

public:

    uint8_t set_pin_output(){return 0;}

    uint8_t set_pin_input(){return 0;}

    uint8_t enable_pullup(){return 0;}

    uint8_t enable_pulldown(){return 0;}

    uint8_t enable_nopull(){return 0;}

    uint8_t set_pin_high(){return 0;}

    uint8_t set_pin_low(){return 0;}

    uint8_t read_pin(){return 0;}

};

#endif
