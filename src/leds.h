#include "Arduino.h"

class Leds {
public:
    static uint8_t changed;

    Leds();
    void start();
    uint8_t state();

};

extern Leds leds;