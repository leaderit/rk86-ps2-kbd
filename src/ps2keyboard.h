#ifndef _PS2KEYBOARD_H_
#define _PS2KEYBOARD_H_
#include <avr/io.h>

/*
    PS2 Keyboard implementation
    PS/2 Clock -> PORTD3
    PS/2 Data  -> PORTD4
 */
class PS2Keyboard {
public:
    // Keyboard Leds
    enum Leds: uint8_t {
        ScrollLock     = 0x1,
        NumLock        = 0x2,
        CapsLock       = 0x4
    };

    // Scancode Flags
    enum Flags: uint16_t {
        KeyUp = 0x0800,
        Ext0  = 0x0100,
        Ext1  = 0x0200
    };

    PS2Keyboard();

    uint16_t id();
    void reset();
    void setLeds( uint8_t new_leds );
    uint8_t leds();
    uint16_t get( bool wait = false );
    void start();
    bool isParityError();
    uint8_t error();
    void setCodeTable(uint8_t table = 2);
    void setTypematic(uint8_t delay, uint8_t period, uint8_t period_k );
private:
    uint8_t _leds;
    uint8_t _error;

    void writeByte(uint8_t b);
    uint16_t getScanCode();
};

extern PS2Keyboard keyboard;

#endif