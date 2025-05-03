#define F_CPU 1000000UL 
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include "avrLib.h"
#include "USART.h"

#define FAN_POWER_PORT PORTB
#define FAN_POWER_DDRx DDRB
#define FAN_POWER_PIN PB0

volatile uint8_t receivedData = 0;

void TIMER_config();

ISR(USART_RXC_vect) {
  // alot must be fixed
  receivedData = UDR;
  if(receivedData >= 254) {
    OCR0 = 1;
    OCR2 = 254;
  }
  else {
    OCR0 = 255-receivedData;
    OCR2 = receivedData;
  }

  if(receivedData == 255) {
    // turn off signal
    CLEAR_PORT(FAN_POWER_PORT, FAN_POWER_PIN); 
    // back to power down mode
    SLEEP_enter_power_down();
  }
}

ISR(INT0_vect) {  
  DB_start_timer();
} 

ISR(TIMER2_COMP_vect) {
  if ((PIND & (1<< PD2)) == 0) { 
    TOGGLE_PORT(FAN_POWER_PORT, FAN_POWER_PIN); 
  }  
  DB_stop_timer();
}

int main(){
  USART_config();
  TIMER_config();
  FAN_POWER_DDRx |= (1<<FAN_POWER_PIN);
  INT0_config_onlow();
  DB_config_timer2();
  
  SLEEP_enter_power_down();

  while(1);
}

void TIMER_config(){
  // TCCR1A |= (1<<WGM11) | (1<<COM1A1);
  // TCCR1B |= (1<<WGM12) | (1<<WGM13); 
  // TIMER_perscalar_selct(1,64);
  // ICR1 = (uint8_t) 255;
  // uint8_t Top = 50;
  // OCR1A = Top;
  // DDRD |= (1<<PD5);

  TCCR0 |= (1<<WGM01) | (1<<WGM00);
  TCCR0 |= (1<<COM01);
  TIMER_perscalar_selct(0,64);
  uint8_t Top= 50;
  OCR0 = Top;
  DDRB |= (1<<PB3);

  TCCR2 |= (1<<WGM21) | (1<<WGM20);
  TCCR2 |= (1<<COM21);
  TIMER_perscalar_selct(2,64);
  uint8_t Top2= 50;
  OCR2 = Top2;
  DDRD |= (1<<PD7);
  
}