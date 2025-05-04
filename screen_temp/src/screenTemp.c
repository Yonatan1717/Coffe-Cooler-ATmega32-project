#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>
#include <LCD.h>
#include <ADC.h>
#include <USART.h>


#define ADC_SAMPLE_SIZE  16


// Simple moving average
volatile uint32_t adcAvg = 0;
volatile uint16_t adcBuffer[ADC_SAMPLE_SIZE];
#if ADC_SAMPLE_SIZE > 255
  volatile uint16_t adcBuffer_index = 0;
#else
  volatile uint8_t adcBuffer_index = 0;
#endif

volatile uint16_t adc_resultat = 0;
volatile _Bool readyFlag = 0;

void config();
void ADC_config();


ISR(ADC_vect) {
 
  SMA_update(ADC);
  USART_sendData((uint8_t) (round((float) (adc_resultat*(5/10.24)))));        
  ADCSRA |= (1<<ADSC);
}



int main(){
  // DDRD |= (1<<PD6)| (1<<PB5);
  USART_config(0);
  
  LCD_setup();
	LCD_function_set();
	LCD_display_on();
	LCD_clear_display();
	LCD_entry_mode();

  
  LCD_write_string("Booting up...", 0x00);
  _delay_ms(1000);
  LCD_clear_display();
  
  ADC_config();

  while(1){
    if(readyFlag){
      LCD_clear_display();
      LCD_write_string("Temp: ", 0x00);
      LDC_write_number_no_addr((adc_resultat*(5/10.24)));
      LDC_write_string_no_addr(" ");
      LCD_write_character_no_addr(0b11011111);
      LDC_write_string_no_addr("C");
      readyFlag = 0;
      _delay_ms(200);
    }

    SLEEP_enter_adc();

  };
      
}



void ADC_config(){
  sei();
  ADC_Prescaler_Selections(32); // Select prescaler for ADC
  // ADMUX = (1<<REFS1)|(1<<REFS0); // Bruk intern 2.56V referanse
  ADCSRA |= (1<<ADEN) | (1<<ADIE) |(1<<ADSC);
}

void SMA_update(uint16_t newADC) {
  uint16_t oldADC = adcBuffer[adcBuffer_index];
  adcBuffer[adcBuffer_index] = newADC;

  adcAvg = adcAvg + newADC-oldADC;
  adc_resultat = adcAvg/ADC_SAMPLE_SIZE;

  adcBuffer_index = (adcBuffer_index + 1) % ADC_SAMPLE_SIZE;
}
