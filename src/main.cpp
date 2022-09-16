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
        LOG_HEX_BIN("State key: ", key);
        if ( keyUp ) bitSet( RADIO_86KR_CTRL_PORT, key);
        else bitClear( RADIO_86KR_CTRL_PORT, key );
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
        LOG(" x=", x, DEC);
        LOG_LN(" y=", y, DEC);
        // if ( key != 0xFF ) setMatrix( key >> 4, key & 0xf, !keyUp );

        if ( key != 0xFF ) matrix.set( key >> 4, key & 0xf, !keyUp );        
      }

// // #ifdef LOG_ON      
//       if ( keyboard.isParityError() ) LOG_TEXT("? ");
//       LOG_HEX_BIN("", scan );

//       if ( scan == 0x0858 ) keyboard.setLeds( keyboard.leds() ^ PS2Keyboard::Leds::CapsLock ); 
//       if ( scan == 0x0877 ) keyboard.setLeds( keyboard.leds() ^ PS2Keyboard::Leds::NumLock ); 
//       if ( scan == 0x087E ) keyboard.setLeds( keyboard.leds() ^ PS2Keyboard::Leds::ScrollLock );

      // if ( scan == 0x0876 ) keyboard.reset();
      
//       if ( scan == 0x080A ) {
//         press = !press;
//         setMatrix(x, y, press);
//         if ( !press ) {
//           y++;
//           if (y>7) {
//             y = 0; x++;
//             if (x>7) x=0;
//           }
//         }
//       }
//       if ( scan == 0xF083 ) {
//         setMatrix(0, 0, true);
//         setMatrix(0, 4, true);
//         setMatrix(1, 2, true);
//         setMatrix(2, 3, true);        
//         setPortFromMatrix( 0b01111111 );        
//         setPortFromMatrix( 0b10111111 );        
//         setPortFromMatrix( 0b00111111 );        
//         setPortFromMatrix( 0b00000000 );        
//       }

//       if ( scan == 0x0801 ) keyboard.setTypematic( 3, 3, 7 ); 
//       if ( scan == 0x0809 ) keyboard.setTypematic( 0, 0, 0 );

//       if ( scan == 0x0805 ) keyboard.id();
//       if ( scan == 0x0803 ) { delay(100); LOG("PORTB=", PINB, BIN);  LOG_LN(" PCINT0 Count=", pcint0_count, DEC); }
// #endif
      if ( keyboard.isParityError() || keyboard.error() > 0 ) keyboard.reset();
    }
    // delay(100);
}