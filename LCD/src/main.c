#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <util/delay.h>
#include <avrLib.h>
#include <LCD.h>


int main(){

	INIT_LCD();
	LCD_function_set();
	LCD_display_on();
	LCD_clear_display();
	LCD_entry_mode();
	
    _delay_ms(500);

	while(1){
		_delay_ms(500);
		LCD_wrap_aroun_effect(4,7);
	}	
}