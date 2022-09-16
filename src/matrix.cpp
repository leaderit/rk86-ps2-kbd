#include "Arduino.h"
#include "matrix.h"
#include "log.h"

#ifdef PS2_ATMEGA_MT8816

#endif

Matrix::Matrix()
{

}

void Matrix::start()
{
    reset();
}


void Matrix::reset()
{
    memset( buf, 0xFF, sizeof( buf ));
}

/*
  Set scan matrix in according of PS/2 pressed/released button
 */
void Matrix::set( uint8_t x, uint8_t y, bool press )
{

}

/*
    Matrix based on Atmega88 20MHz Crystall 
 */
#ifdef PS2_ATMEGA
void Matrix::set( uint8_t x, uint8_t y, bool press = false )
{
  uint8_t idx;

  x &= 0x7;
  y &= 0x7;
  idx = 0xFF & ~(0x1 << ( x ));
cli();
  // if ( press ) matrix_array[ idx ] &= ~( 0x1 << y );
  // else matrix_array[ idx ] |= ( 0x1 << y );
  if ( press ) matrix_array[ idx ] = uint8_t ( 0xff & ~( 0x1 << y ));
  else matrix_array[ idx ] = 0xff;
sei();
  // LOG_LN(" MATRIX=", matrix_array[ idx ], BIN);
}
#endif

#define MT8816_AX0 8
#define MT8816_AX1 9
#define MT8816_AX2 10
#define MT8816_AX3 11

#define MT8816_AY0 12
#define MT8816_AY1 13
#define MT8816_AY2 20

#define MT8816_RESET    2
#define MT8816_DATA     21
#define MT8816_STB      7


Matrix_MT8816::Matrix_MT8816()
{

}


void Matrix_MT8816::start()
{
    pinMode(MT8816_AX0, OUTPUT);
    pinMode(MT8816_AX1, OUTPUT);
    pinMode(MT8816_AX2, OUTPUT);
    pinMode(MT8816_AX3, OUTPUT);
    pinMode(MT8816_AY0, OUTPUT);
    pinMode(MT8816_AY1, OUTPUT);
    pinMode(MT8816_AY2, OUTPUT);
    digitalWrite(MT8816_RESET, LOW);
    digitalWrite(MT8816_STB, LOW);
    digitalWrite(MT8816_DATA, LOW);
    pinMode(MT8816_RESET, OUTPUT);
    pinMode(MT8816_STB, OUTPUT);
    pinMode(MT8816_DATA, OUTPUT);
    reset();
}


void Matrix_MT8816::reset()
{
    digitalWrite(MT8816_RESET, HIGH);
    delay(1);
    digitalWrite(MT8816_RESET, LOW);    
    LOG_TIME("MATRIX RESETED\n");
}



void Matrix_MT8816::set(  uint8_t x, uint8_t y, bool press )
{
    x &= 0x7;
    y &= 0x7;

    // ON/OFF
    digitalWrite(MT8816_DATA, press? HIGH:LOW );
    // X
    digitalWrite(MT8816_AX0, (x & 0b0001)? HIGH:LOW );
    digitalWrite(MT8816_AX1, (x & 0b0010)? HIGH:LOW );
    digitalWrite(MT8816_AX2, (x & 0b0100)? HIGH:LOW );
    digitalWrite(MT8816_AX3, (x & 0b1000)? HIGH:LOW );
    // Y
    digitalWrite(MT8816_AY0, (y & 0b001)? HIGH:LOW );
    digitalWrite(MT8816_AY1, (y & 0b010)? HIGH:LOW );
    digitalWrite(MT8816_AY2, (y & 0b100)? HIGH:LOW );
    // STROB
    digitalWrite(MT8816_STB, HIGH);
    delay(1);
    digitalWrite(MT8816_STB, LOW);
    LOG("MATRIX X=", x, DEC);      
    LOG_LN(" Y=", y, DEC);
}

#ifdef PS2_ATMEGA_MT8816
    Matrix_MT8816 matrix;
#elif PS2_ATMEGA
    Matrix matrix;
#elif PS2_STM32
    Matrix matrix;
#else
    Matrix matrix;
#endif