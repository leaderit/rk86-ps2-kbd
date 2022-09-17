/*
  PS/2 protocol 
  https://www.youtube.com/watch?v=7C4zeHbLxNg
 */
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include "log.h"
#include "ps2keyboard.h"
#include "keycodes.h"
#include "matrix.h"
#include "leds.h"

bool led_rus_lat;
bool radio86rk_shift = false;

/*
  Reset/Restart system setup
 */
void setup() {
  LOG_BEGIN();
  // Start keyboard matrix
  matrix.start();
  // start leds interrupt
  leds.start();
  // Start PS/2 keyboard
  keyboard.start();

  digitalWrite(RADIO_86KR_RL_KEY, HIGH);
  digitalWrite(RADIO_86KR_UC_KEY, HIGH);
  digitalWrite(RADIO_86KR_CC_KEY, HIGH);
  pinMode( RADIO_86KR_RL_KEY, OUTPUT);
  pinMode( RADIO_86KR_UC_KEY, OUTPUT);
  pinMode( RADIO_86KR_CC_KEY, OUTPUT);
    
  // Reset keyboard on reboot
  keyboard.reset(); 
  keyboard.setCodeTable( 2 ); 
  // Maximum delays
  keyboard.setTypematic( 3, 3, 7 );
  delay(200);
}

uint8_t x = 0, y = 0;
bool press = false;

void loop() {
    uint16_t scan;
    uint8_t key;
    bool keyUp;

    if ( Leds::changed ) {
      led_rus_lat = ( leds.state() & 0b1 ) == 0;

      if ( led_rus_lat ) keyboard.setLeds( keyboard.leds() | PS2Keyboard::Leds::CapsLock );
      else keyboard.setLeds( keyboard.leds() & ~PS2Keyboard::Leds::CapsLock );
      Leds::changed--;
      LOG_LN("LED R/L changed to: ", led_rus_lat, DEC );
      LOG_LN("LEDS = ", leds.state(), BIN );      
      radio86rk_ru_mode = led_rus_lat ^ radio86rk_shift;
    }

    scan = keyboard.get();
    if ( scan ) {
      LOG_HEX_BIN("SCAN: ", scan );
      keyUp = ( scan & PS2Keyboard::Flags::KeyUp ) != 0;

      key = stateKey( scan );
      if ( key != 0xFF ) {
        LOG_TIME(""); LOG_LN("State key: ", key, DEC);
        digitalWrite( key, keyUp? HIGH:LOW);
        if ( key ==  RADIO_86KR_CC_KEY ) {
          radio86rk_shift = !keyUp;
          radio86rk_ru_mode = led_rus_lat ^ radio86rk_shift;
        }
      } else {
        key = scanToMatrix( scan );
        uint8_t x, y;
        x = key >> 4;
        y = key & 0xf;
        LOG(" Key:", key, HEX);
        LOG(" Key Press:", !keyUp, DEC);
        LOG(" Matrix x=", x, DEC);
        LOG_LN(" y=", y, DEC);
        if ( key != 0xFF ) matrix.set( key >> 4, key & 0xf, !keyUp );        
      }
      if ( keyboard.isParityError() || keyboard.error() > 0 ) keyboard.reset();
    }
}
