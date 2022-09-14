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

// #define LED 7

#define SCAN_PORT PORTB

#define RUSLAT_LED_PORT PORTD
#define RUSLAT_LED_PIN 2

uint8_t matrix[256];
bool led_rus_lat;

uint8_t led_rus_lat_chaged = 2;
bool radio86rk_shift = false;
/*
  Set scan matrix in according of PS/2 pressed/released button
 */
void setMatrix_atmega88( uint8_t x, uint8_t y, bool press = false )
{
  uint8_t idx;

  x &= 0x7;
  y &= 0x7;
  idx = 0xFF & ~(0x1 << ( x ));
cli();
  // if ( press ) matrix[ idx ] &= ~( 0x1 << y );
  // else matrix[ idx ] |= ( 0x1 << y );
  if ( press ) matrix[ idx ] = uint8_t ( 0xff & ~( 0x1 << y ));
  else matrix[ idx ] = 0xff;
sei();
  // LOG_LN(" MATRIX=", matrix[ idx ], BIN);
}


void setMatrix( uint8_t x, uint8_t y, bool press = false )
{
  uint8_t idx;

  x &= 0x7;
  y &= 0x7;
  idx = 0xFF & ~(0x1 << ( x ));
cli();
  // if ( press ) matrix[ idx ] &= ~( 0x1 << y );
  // else matrix[ idx ] |= ( 0x1 << y );
  if ( press ) matrix[ idx ] = uint8_t ( 0xff & ~( 0x1 << y ));
  else matrix[ idx ] = 0xff;
sei();
  // LOG_LN(" MATRIX=", matrix[ idx ], BIN);
}

/* 
  Set matrix data to Output port based on Scan port
 */
// void setOutPort( uint8_t data )
// {
//   PORTC = (data & 0x3F) | 0x3f;
//   PORTD = (data & 0xC0) | (PIND & 0x3F);
// }

/*
 RUS/LAT LED Interrupt handler
 */
ISR(INT0_vect) {
  led_rus_lat_chaged++;
}

/*
  Retro PC Keyboard Scan Port interrupt
  PORTB pins change interrupt
 */
int pcint0_count = 0;
uint8_t port_b_value = 0;

#define PORTC_MASK 0x3f
#define PORTD_MASK 0xc0


ISR(PCINT0_vect)
{

  // uint8_t data = matrix[ PINB ];
  uint8_t data = 0xff;
  // cli();

  if ( PINB == 0b11111011 ) data = ~( 0x1 << 1);
  // if ( PINB == 0b10111111 ) data = ~( 0x1 << 1 );

  // Set matrix data to Output port based on Scan port
  // PORTC = (data & 0x3F); // | 0b00000010;
  // PORTD = (data & 0xC0) | (PIND & 0x3F);

  PORTC = (data & PORTC_MASK) | (PINC & ~PORTC_MASK);
  PORTD = (data & PORTD_MASK) | (PIND & ~PORTD_MASK);
  // _NOP();
  // sei();
}

/*
  Reset/Restart system setup
 */
void setup() {
  // clear keyboard matrix
  memset( matrix, 0xFF, sizeof( matrix ));
  LOG_BEGIN();

  // Setup Scan Input port
  DDRB = 0;
  PORTB = 0xff;

  // Setup interrupt vectors PCINT0..7
  PCMSK0 = 0xff;
  bitSet( PCICR, PCIE0 );

  // Setup output scan ports PORTC0..5, PORTD6..7
  DDRC = 0x3f;
  PORTC |= 0x3f;
  
  // Debug Serial port pins usin for CC and UC keys
#ifndef LOG_ON
  DDRD = 0xE3;
  PORTD |= 0xE3;
#else
  DDRD = 0xE0;
  PORTD |= 0xFF;

  // DDRD = 0x00;
  // PORTD &= ~0xE0; 
#endif

  // Interrupt on Rus/LAT Led any changed
  bitSet( EICRA, ISC00 );
  bitClear( EICRA, ISC01 );
  bitSet( EIMSK, INT0 );

  // Start keyboard
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

    if ( led_rus_lat_chaged ) {
      led_rus_lat = ( PIND & _BV(RUSLAT_LED_PIN)) == 0;
      if ( led_rus_lat ) keyboard.setLeds( keyboard.leds() | PS2Keyboard::Leds::CapsLock );
      else keyboard.setLeds( keyboard.leds() & ~PS2Keyboard::Leds::CapsLock );
      led_rus_lat_chaged--;
      LOG_LN("LED R/L changed to: ", led_rus_lat, DEC );
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
        if ( key != 0xFF ) setMatrix( key >> 4, key & 0xf, !keyUp );
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