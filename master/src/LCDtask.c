#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avrLib.h>
#include <LCD.h>


int main(){

    LCD_setup();
    LCD_function_set();
    LCD_display_on();
    LCD_clear_display();
    LCD_entry_mode();
    //testing
    
    
    LCD_write_string("Booting up", 0x00);
    _delay_ms(1000);
    LCD_clear_display();

    LCD_write_string("BP: Stavanger",0x00);
    LCD_write_string("BD: 10.10.1010", 0x40);
        
    while(1);
    return 0;
}
