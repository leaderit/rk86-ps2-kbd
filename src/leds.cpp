
#include "leds.h"

uint8_t Leds::changed = 2;

#ifdef PS2_ATMEGA_MT8816

#define LED_0_PIN 14
#define LED_1_PIN 15
#define LED_2_PIN 16
#define LED_3_PIN 17

/*
    LEDS 0-3 changed Interrupt handler
 */
ISR(PCINT1_vect) {
   if ( Leds::changed < 4 ) Leds::changed++;
}
#endif      

#ifdef PS2_ATMEGA

#define LED_0_PIN 14
#define LED_1_PIN 15
#define LED_2_PIN 16
#define LED_3_PIN 17

/*
 RUS/LAT LED Interrupt handler
 */
ISR(INT0_vect) {
    if ( Leds::changed < 4 ) Leds::changed++;  
}
#endif  


Leds::Leds()
{

}

void Leds::start()
{
    pinMode(LED_0_PIN, INPUT_PULLUP);
    pinMode(LED_1_PIN, INPUT_PULLUP);
    pinMode(LED_2_PIN, INPUT_PULLUP);
    pinMode(LED_3_PIN, INPUT_PULLUP);
  
    #ifdef PS2_ATMEGA_MT8816
    // Setup interrupt vectors PCINT0..3
    PCMSK1 = 0x0f;
    bitSet( PCICR, PCIE1 );
    #endif   

    #ifdef PS2_ATMEGA
    // Interrupt on Rus/LAT Led any changed
    bitSet( EICRA, ISC00 );
    bitClear( EICRA, ISC01 );
    bitSet( EIMSK, INT0 );
    #endif   

}

uint8_t Leds::state()
{
    return 
    digitalRead(LED_0_PIN) << 0 |
    digitalRead(LED_1_PIN) << 1 |
    digitalRead(LED_2_PIN) << 2 |
    digitalRead(LED_3_PIN) << 3;

}


Leds leds;
