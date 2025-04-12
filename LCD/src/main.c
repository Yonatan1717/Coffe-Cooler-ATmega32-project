#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <util/delay.h>
#include <avrLib.h>
#include <LCD.h>


int main(){

	INIT_LCD();
	FUNCTION_SET();
	DISPLAY_ON_OFF();
	CLEAR_DISPLAY();
	ENTRY_MODE();
	
    _delay_ms(500);

	while(1){
		_delay_ms(500);
		WRAP_AROUND(4,7);
	}	
}