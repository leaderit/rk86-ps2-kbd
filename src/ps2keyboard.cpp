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

#define PS2_CLOCK 3
#define PS2_DATA  4

#define BUFFER_SIZE 50
static volatile uint8_t buffer[BUFFER_SIZE];
static volatile uint8_t head, tail;
static volatile bool parity_error = false;
static volatile uint8_t data_parity = 0;
static volatile uint8_t leds = 0;

static volatile bool host_to_keyboard_mode = false;
uint8_t data_for_send;
uint8_t data_ack;

PS2Keyboard keyboard;

/*
  PS/2 CLOCK Interrupt in host read mode
 */
void data_read(void) 
{
	static uint8_t bitcount = 0;
	static uint8_t data_byte = 0;
	static uint32_t prev_ms = 0;
    static uint8_t parity = 1;
	uint32_t now_ms;
	uint8_t data_bit;

	now_ms = millis();
	if (now_ms - prev_ms > 250) {
		bitcount = 0;
		data_byte = 0;
    parity = 1;
	}
	prev_ms = now_ms;
	bitcount++;
  // Next bit
	data_bit = digitalRead( PS2_DATA );
  // START bit always zero + 8 Data bits
  if ( bitcount <= 9) {
    data_byte = data_byte >> 1 | data_bit << 7;
    parity ^= data_bit;
	}
  // PARITY bit
  if ( bitcount == 10 ) parity_error = ( parity != data_bit );
  // STOP bit
	if (bitcount == 11) {
		uint8_t i = head + 1;
		if (i >= BUFFER_SIZE) i = 0;
		if (i != tail) {
			buffer[i] = data_byte;
			head = i;
		}
		bitcount = 0;
		data_byte = 0;
        parity = 1;
	}
}

/*
  PS/2 CLOCK Interrupt in host write mode
 */
void data_write(void)
{
	static uint8_t bitcount=0;
	static uint8_t data_byte=0;
    static uint8_t parity = 1;
	uint8_t data_bit;

    // START bit
    if ( bitcount == 0 ) {
        bitcount = 0;
        parity = 1;
        data_byte = data_for_send;
        digitalWrite( PS2_DATA, LOW );
    }
    bitcount++;
    // Next bit
    data_bit = data_byte & 0x1;
    // Start bit always zero + 8 Data bits
    if ( (bitcount > 1) && (bitcount <= 9)) {
        digitalWrite( PS2_DATA, data_bit );
        data_byte = data_byte >> 1 | data_bit << 7;
        parity ^= data_bit;
        }
    // PARITY bit
    if ( bitcount == 10 ) {
        data_parity = parity;
        digitalWrite( PS2_DATA, parity );
    }
    // STOP bit
    if (bitcount == 11) {
        digitalWrite( PS2_DATA, HIGH );
    }
    // ACK bit from keyboard
    if (bitcount == 12) {
        pinMode( PS2_DATA, INPUT_PULLUP );
        uint8_t ack = digitalRead( PS2_DATA );
        data_ack = ack;
        bitcount = 0;
        data_byte = 0;
        parity = 1;
        host_to_keyboard_mode = false;
    }
}

/*
 PS/2 CLOCK Interrupt handler
 */
ISR(INT1_vect) {
    if ( host_to_keyboard_mode ) data_write();
    else data_read();
}

// static PS2Keyboard::_parityError = 0;
/*

 */
PS2Keyboard::PS2Keyboard()
:_error(0)
{

}

uint16_t PS2Keyboard::id()
{
    uint16_t _id, id;

    writeByte( 0xf2 ); 
    id = get( true );
    _id = get( true );
    id = id << 8 | _id;
    return id;
}

/*

 */
void PS2Keyboard::reset()
{
    uint16_t ack;

    _error = 0;
    writeByte( 0xff ); 
    ack = get( true );
    if ( ack != 0xAA ) _error = ack;    // Reset error
    _leds = 0;
}

/*

 */
void PS2Keyboard::setLeds( uint8_t new_leds )
{
    writeByte( 0xED ); // SET LEDS command
    writeByte( new_leds );
    _leds = new_leds;
}

/*

 */
uint8_t PS2Keyboard::leds()
{
    return _leds;
}


/*

 */
uint16_t PS2Keyboard::get( bool wait )
{
    uint16_t scan = 0;

    if ( wait ) while (( scan = getScanCode()) == 0 ) delay(1);
    else scan = getScanCode();
    return scan;
}

/*

 */
void PS2Keyboard::start()
{
    pinMode( PS2_CLOCK, INPUT_PULLUP );
    pinMode( PS2_DATA, INPUT_PULLUP );  

    bitSet( EICRA, ISC11 );
    bitClear( EICRA, ISC10 );
    bitSet( EIMSK, INT1 );
}

/*

 */
bool PS2Keyboard::isParityError()
{
    return parity_error;
}

uint8_t PS2Keyboard::error(){
    return _error;
}

/*

 */
void PS2Keyboard::setCodeTable( uint8_t table )
{
    if (table < 4) {
        writeByte( 0xF0 ); // SET TABLE command
        writeByte( table );
    }
}


/*

 */
void PS2Keyboard::setTypematic(uint8_t repeat_delay, uint8_t period, uint8_t period_k )
{
    writeByte( 0xF3 ); // SET TABLE command
    writeByte( ((repeat_delay & 0x3) << 5 ) | ((period & 0x3) << 3) | ( period_k & 0x7 ) );
}

/*
    Internal function for sending commands to keyboard
 */
void PS2Keyboard::writeByte(uint8_t b)
{
    uint8_t ack;

    _error = 0;
    data_for_send = b;
    host_to_keyboard_mode = true;
    
    digitalWrite( PS2_CLOCK, HIGH );
    digitalWrite( PS2_DATA, HIGH );  
    pinMode( PS2_CLOCK, OUTPUT );
    pinMode( PS2_DATA,  OUTPUT );
    digitalWrite( PS2_CLOCK, LOW );
    delayMicroseconds( 10 );
    digitalWrite( PS2_DATA, LOW );  
    delayMicroseconds( 60 );
    digitalWrite( PS2_CLOCK, HIGH );
    pinMode( PS2_CLOCK, INPUT_PULLUP );
    
    while ( host_to_keyboard_mode ) delay(1);
    delay(1);
    ack = get( true );
    if ( ack != 0xFA ) _error = ack;  // Send error
}

uint16_t PS2Keyboard::getScanCode()
{
	uint8_t i;
    uint16_t c;
    static bool up = false;
    static bool ext0 = false;
    static bool ext1 = false;

	i = tail;
	if (i == head) return 0;
	i++;
	if (i >= BUFFER_SIZE) i = 0;
	c = buffer[i];
	tail = i;

    if ( c == 0xF0 ) {
        up = true;
        return 0;
    }
    if ( c == 0xE0 ) {
        ext0 = true;
        return 0;
    }
    if ( c == 0xE1 ) {
        ext1 = true;
        return 0;
    }    
    // Check flags
    if ( ext0 ) {
        ext0 = false;
        c |= PS2Keyboard::Flags::Ext0; 
    }
    if ( ext1 ) {
        ext1 = false;
        // Hack for Pause/Break key
        if ( c == 0x14 || ( c == 0x14 && up )) ext1 = true;
        c |= PS2Keyboard::Flags::Ext1; 
    }
    if ( up ) {
        up = false;
        c |= PS2Keyboard::Flags::KeyUp;
    }   
	return c;
}
