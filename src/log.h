/* 
    For debug mode define LOG_ON before include the header
    #define LOG_ON
*/
#ifndef _LOG_H_
#define _LOG_H_

#define LOG_ON

#include <Arduino.h>
#define LogSerial Serial
#define DEBUG_SERIAL_SPEED  9600

#ifndef LOG_ON
    #define LOG_BEGIN(x)
    #define LOG_TIME( text ) 
    #define LOG(descr, var, format)
    #define LOG_TEXT( text )
    #define LOG_LN(descr, var, format)
    #define LOG_END()
    #define LOG_HEX_BIN( descr, b )
#else
    #define LOG_BEGIN() LogSerial.begin(DEBUG_SERIAL_SPEED)

    #define LOG_TIME( text )  \
            LogSerial.print(millis()); \
            LogSerial.print(F(" ms : ")); \
            LogSerial.print( text ); 
    #define LOG(descr, var, format)  \
            LogSerial.print( descr ); \
            LogSerial.print( var, format );
    #define LOG_TEXT( text ) LogSerial.print( text ); 
    #define LOG_LN(descr, var, format)  \
            LogSerial.print( descr ); \
            LogSerial.println( var, format );
    #define LOG_HEX_BIN(descr,  b ) \
            LOG_TIME(descr); \
            LOG( "", b, HEX); \
            LOG_LN( " ", b, BIN);
#endif

#endif