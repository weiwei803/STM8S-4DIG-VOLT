#ifndef DELAY_H
    #define DELAY_H
    #define F_CPU 22000000 // was a 22000000
#include <stdint.h>
    #include "stm8s.h"
    #ifndef F_CPU
        #error "F_CPU not defined!"
    #endif
    
    
    #define US(x) \
    (unsigned int)((((x*(F_CPU/1000000.0))<=6)*6 + ((x*(F_CPU/1000000.0))>6)*(x*(F_CPU/1000000.0)) - 2)/4)
    
    #define MS_SHORT_MAX  ((262140000UL/F_CPU))
    
 #pragma inline //=forced
 void delay(uint16_t __cycle_count)
    {
        __asm("loop:  \n"
              "decw x \n"
              "jrne loop \n"
               " nop \n");
    }
    
    #define delay_us(x) delay(US(x))
    
     #pragma inline =forced
 void delay_ms(uint16_t ms)
    {
        if(ms < MS_SHORT_MAX)
            {
                delay(US((uint16_t)ms*1000));
            }
            else
            {
                while(ms--) delay(US(999));
            }
    }


#endif  // #ifndef DELAY_H