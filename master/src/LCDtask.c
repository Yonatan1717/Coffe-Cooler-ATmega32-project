#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avrLib.h>
#include <LCD.h>


int main(){
    
    INIT_LCD();
    FUNCTION_SET();
    DISPLAY_ON_OFF();
    CLEAR_DISPLAY();
    ENTRY_MODE();
    //testing 2
    
    
    WRITE_STRING("Booting up", 0x00);
    _delay_ms(1000);
    CLEAR_DISPLAY();

    WRITE_STRING("Birth Place: Somewhere",0x00);
    WRITE_STRING("Birthdate: ", 0x40);
    WRITE_NUMBER_noAddr(10202023);
    
    ADC_config();
    
    while(1);
    return 0;
}