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
  uint8_t factorX = 0;

  if (receivedData <= 25) {
    factorX = 0;
    CLEAR_PORT(FAN_POWER_PORT, FAN_POWER_PIN); 
  } else if (receivedData >= 26 && receivedData <= 70) {
    factorX = 50 + (uint8_t)(((receivedData - 26) * 197.0) / 44.0);  // 44 fordi 70-26 = 44 trinn
  } else {
    factorX = 255;
  }

  OCR0 = factorX;
  OCR2 = 255 - factorX;

}

ISR(INT0_vect) {  
  DB_start_timer(1, 1024);
} 

ISR(TIMER2_COMP_vect) {
  TOGGLE_PORT(FAN_POWER_PORT, FAN_POWER_PIN); 
  DB_stop_timer(1);
}

int main(){
  USART_config(1);
  TIMER_config();
  FAN_POWER_DDRx |= (1<<FAN_POWER_PIN);
  INT0_config_onlow();
  DB_config_timer(1);

  while(1) {
    SLEEP_enter_idle();
  };
}

void TIMER_config(){

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