#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>
#include <LCD.h>
#include <ADC.h>

volatile uint16_t adc_resultat = 0;
volatile _Bool ready = 0;

void config();
void Timer_config();


// ISR(TIMER1_COMPA_vect){
//   if(!ADC_STATUS){
//     ADCSRA |= (1<<ADSC);
//   }

//   // OCR0 = (round(ADC/4) - 1);
//   // OCR2 = 255-(round(ADC/4) - 1);

// }

ISR(ADC_vect) {
  static uint8_t count = 0;
  static uint32_t sum = 0;

  sum += ADC;
  count++;

  if (count >= 16) {
    adc_resultat = sum / 16; // Gjennomsnitt
    count = 0;
    sum = 0;
    ready = 1;
  }

  ADCSRA |= (1<<ADSC);
}



int main(){
  // DDRD |= (1<<PD6)| (1<<PB5);
  

  INIT_LCD();
	FUNCTION_SET();
	DISPLAY_ON_OFF();
	CLEAR_DISPLAY();
	ENTRY_MODE();

  
  WRITE_STRING("Starter opp...", 0x00);
  _delay_ms(1000);
  CLEAR_DISPLAY();
  
  Timer_config();

  while(1){
    if(ready){
      CLEAR_DISPLAY();
      WRITE_NUMBER((adc_resultat), 0x00);
      ready = 0;
      _delay_ms(100);
    }
    
  };
}



void Timer_config(){
  sei();
  // Clock_Select_Description_for_a_Timer_Counter_n(1,1024); 
  ADC_Noise_Reduse; // set ADC Noise Reduction
  ADMUX = (1<<REFS1)|(1<<REFS0); // Bruk intern 2.56V referanse
  ADCSRA |= (1<<ADEN) | (1<<ADIE) |(1<<ADSC);
  
  // TCCR1B |= (1<<WGM12);
  // TIMSK |= (1<<OCIE1A);
  // uint16_t top = 50;
  // OCR1A = top;

  // TCCR0 |= (1<<WGM01) | (1<<WGM00);
  // TCCR0 |= (1<<COM01);
  // Clock_Select_Description_for_a_Timer_Counter_n(0,64);
  // uint8_t Top= 50;
  // OCR0 = Top;
  // DDRB |= (1<<PB3);

  // TCCR2 |= (1<<WGM21) | (1<<WGM20);
  // TCCR2 |= (1<<COM21);
  // Clock_Select_Description_for_a_Timer_Counter_n(2,64);
  // uint8_t Top2= 50;
  // OCR2 = Top2;
  // DDRD |= (1<<PB7);
  
}