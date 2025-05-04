#define F_CPU 1000000UL 
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <USART.h>

#define FACTOR 40

volatile uint8_t receivedData = 0;

void TIMER_config();

ISR(USART_RXC_vect){
  PORTB |= 1;
  recivedData = UDR;
  if (recivedData < 4) {
    OCR0 = 0;
    OCR2 = 255;
  } else if (recivedData >= 5 && recivedData <= 12) {
    uint8_t factorX = FACTOR*1;
    OCR0 = factorX;
    OCR2 = 255-factorX;
  } else if (recivedData >= 13 && recivedData <= 25) {
    uint8_t factorX = FACTOR*2;
    OCR0 = factorX;
    OCR2 = 255-factorX;
    PORTB &= ~(1<<PB0);
  } else if (recivedData >= 40 && recivedData <= 50) {
    uint8_t factorX = FACTOR*3;
    OCR0 = factorX;
    OCR2 = 255-factorX;
  } else if (recivedData >= 55 && recivedData <= 65) {
    uint8_t factorX = FACTOR*4;
    OCR0 = factorX;
    OCR2 = 255-factorX;
  } else if (recivedData > 70) {
    uint8_t factorX = FACTOR*5;
    OCR0 = factorX;
    OCR2 = 255-factorX;
  } else {
    OCR0 = 255;
    OCR2 = 0;
  }
}

      ISR(INT0_vect) {  
        DB_start_timer(1, 1024);
      } 

      ISR(TIMER2_COMP_vect) {
        TOGGLE_PORT(FAN_POWER_PORT, FAN_POWER_PIN); 
        DB_stop_timer(1);
      }

int main(){
  DDRB |= 1;
  USART_config(1);
  Timer_config();
  interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE();
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