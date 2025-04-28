#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>
#include <LCD.h>
#include <ADC.h>
#include <USART.h>

volatile uint16_t adc_resultat = 0;
volatile _Bool ready = 0;

void config();
void ADC_config();


ISR(ADC_vect) {
  static uint16_t count = 0;
  static uint32_t sum = 0;

  sum += ADC;
  count++;

  if (count >= 1000) {
    adc_resultat = sum / 1000; // Gjennomsnitt
    count = 0;
    sum = 0;
    ready = 1;
    USART_sendData((uint8_t) (round((float) (adc_resultat/3.9))));
  }

  ADCSRA |= (1<<ADSC);
}



int main(){
  // DDRD |= (1<<PD6)| (1<<PB5);
  USART_config();
  

  SETUP();
	FUNCTION_SET();
	DISPLAY_ON_OFF();
	CLEAR_DISPLAY();
	ENTRY_MODE();

  
  WRITE_STRING("Booting up...", 0x00);
  _delay_ms(1000);
  CLEAR_DISPLAY();
  
  ADC_config();

  while(1){
    if(ready){
      CLEAR_DISPLAY();
      WRITE_STRING("Temp: ", 0x00);
      WRITE_NUMBER_noAddr((adc_resultat*(5/10.24)));
      WRITE_STRING_noAddr(" ");
      WRITE_STRING_SINGLE_noAddr(0b11011111);
      WRITE_STRING_noAddr("C");
      ready = 0;
      _delay_ms(200);
    }
    
  };
}



void ADC_config(){
  sei();
  ADC_Noise_Reduse; // set ADC Noise Reduction
  ADC_Prescaler_Selections(32); // Select prescaler for ADC
  // ADMUX = (1<<REFS1)|(1<<REFS0); // Bruk intern 2.56V referanse
  ADCSRA |= (1<<ADEN) | (1<<ADIE) |(1<<ADSC);
}



