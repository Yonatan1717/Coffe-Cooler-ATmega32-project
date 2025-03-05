#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>

ISR(TIMER2_COMP_vect)
{
	if(OCR0>0){ // if OCR0 is lager than 0 when the interrupt occurs than decrease it by 17
		OCR0 -= 17;
	}
  	else OCR0 = 255; // else reset OCR0 back to 255
}


void interruptConfig(){
	sei();
	TCCR2 |= (1<<WGM21); // activate CTC of clock 2 8bit
	TCCR2 |= (1<<CS20) | (1<<CS21) | (1<<CS22); // set clock bit description to clkI/O/1024 (From prescaler)
	TIMSK |= (1<<OCIE2); // enable output compare A match interrupt enable 

  	uint16_t ocr1aCount = 244; /* set max count that we whis to set to OCR2 register to 244. 
								Setting it to 244 when we count each 1024 nano seconds will give us about 250 ms.*/
	OCR2 = ocr1aCount;
	

	
	TCCR0 |= (1<<WGM01) | (1<<WGM00); // activate Fast PWM
	TCCR0 |= (1<<COM01) | (1<<COM00); // activate Clear OC0 on compare match, set OC0 at Bottom, (inverting mode)
	TCCR0 |= (1<<CS01) | (1<<CS00); // set clock bit description to clkI/O/64 (From prescaler)
	
	uint16_t countComp = 255;
	OCR0 = countComp; // set OCR0 to 255
}

int main(void)
{
	interruptConfig(); // configuration
	DDRB |= (1<<PB3); // set output on PB3
	
  	while (1);
}

