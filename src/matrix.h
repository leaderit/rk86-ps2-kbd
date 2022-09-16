/*
    Keyboard matrix abstraction
*/
#include <Arduino.h>

/*
    Abstract mKeyboard matrix class
 */
class Matrix {

public:
    Matrix();

    virtual void start();
    virtual void reset();
    virtual void set( uint8_t x, uint8_t y, bool press = false );

private:
    uint8_t buf[256];

};

/*
    MT8816 keyboard matrix
 */
class Matrix_MT8816: Matrix {
public:

    Matrix_MT8816();
    virtual void start();
    virtual void reset();
    virtual void set( uint8_t x, uint8_t y, bool press = false );
private:

};

// extern Matrix matrix;

#ifdef PS2_ATMEGA_MT8816
    extern Matrix_MT8816 matrix;
#elif PS2_ATMEGA
    extern Matrix matrix;
#elif PS2_STM32
   extern  Matrix matrix;
#else
   extern  Matrix matrix;
#endif
