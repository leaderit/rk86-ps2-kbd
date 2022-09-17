#ifndef _KEYCODES_H_
#define _KEYCODES_H_
#include <avr/io.h>
#include "log.h"

#ifdef PS2_ATMEGA
// Defenitions for Radio 86RK retro PC state keys PORTD pins
#define RADIO_86KR_CTRL_PORT PORTD
// #define RADIO_86KR_CTRL_PORT DDRD
#ifdef LOG_ON
    #define RADIO_86KR_DEBUG_KEY    5
    #define RADIO_86KR_RL_KEY       RADIO_86KR_DEBUG_KEY
    #define RADIO_86KR_UC_KEY       RADIO_86KR_DEBUG_KEY
    #define RADIO_86KR_CC_KEY       RADIO_86KR_DEBUG_KEY
#else
    #define RADIO_86KR_RL_KEY   5
    #define RADIO_86KR_UC_KEY   1
    #define RADIO_86KR_CC_KEY   0
#endif
#endif

#ifdef PS2_ATMEGA_MT8816
    #define RADIO_86KR_RL_KEY   18
    #define RADIO_86KR_UC_KEY   19
    #define RADIO_86KR_CC_KEY   5
    #define CONTROL_KEY3        6
#endif


extern bool radio86rk_ru_mode;
// return bit flag if scancode is state change key, like "Control", "Alt" etc
uint8_t stateKey( uint16_t scan);

// return matrix row and column for the scan code
uint8_t scanToMatrix( uint16_t scan );

#endif