#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avrLib.h>
#include <LCD.h>


int main(){

    SETUP();
    FUNCTION_SET();
    DISPLAY_ON_OFF();
    CLEAR_DISPLAY();
    ENTRY_MODE();
    
    
    WRITE_STRING("Booting up", 0x00);
    _delay_ms(1000);
    CLEAR_DISPLAY();

    WRITE_STRING("BP: Stavanger",0x00);
    WRITE_STRING("BD: 10.10.1010", 0x40);
        
    while(1);
    return 0;
}