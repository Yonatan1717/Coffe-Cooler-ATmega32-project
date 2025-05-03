#define F_CPU 1000000UL 
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>
#include <ADC.h>
#include <math.h>
#include <USART.h>

volatile uint8_t recivedData = 0;

void Timer_config();

ISR(USART_RXC_vect){
  PORTB |= 1;
  recivedData = UDR;
  if(recivedData >= 255) {
    OCR0 = 1;
    OCR2 = 255;
  }
  else {
    OCR0 = 255-recivedData;
    OCR2 = recivedData;
  }
}

ISR(INT0_vect) {  
  if (debounce(&PIND, PD2)) { 
    PORTB ^= (1<<PB0);
  }  
} 

int main(){
  DDRB |= 1;
  USART_config(1);
  Timer_config();
  interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE();
  while(1);
}

void Timer_config(){
  // TCCR1A |= (1<<WGM11) | (1<<COM1A1);
  // TCCR1B |= (1<<WGM12) | (1<<WGM13); 
  // Clock_Select_Description_for_a_Timer_Counter_n(1,64);
  // ICR1 = (uint8_t) 255;
  // uint8_t Top = 50;
  // OCR1A = Top;
  // DDRD |= (1<<PD5);

  TCCR0 |= (1<<WGM01) | (1<<WGM00);
  TCCR0 |= (1<<COM01);
  Clock_Select_Description_for_a_Timer_Counter_n(0,64);
  uint8_t Top= 50;
  OCR0 = Top;
  DDRB |= (1<<PB3);

  TCCR2 |= (1<<WGM21) | (1<<WGM20);
  TCCR2 |= (1<<COM21);
  Clock_Select_Description_for_a_Timer_Counter_n(2,64);
  uint8_t Top2= 50;
  OCR2 = Top2;
  DDRD |= (1<<PB7);
  
}