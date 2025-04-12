#define F_CPU 1000000UL 
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>
#include <ADC.h>
#include <math.h>

volatile uint8_t slave = 0;
volatile uint8_t latestData = 0;


void ADC_config();
void slave_handler(uint8_t VinPort, uint8_t slave_addr);
void end_conn();

ISR(INT2_vect){
  if(debounce(&PINB,PB2)){
    end_conn();
  }
}
  
ISR(INT0_vect) {  
  if (debounce(&PIND, PD2)) { 
    slave_handler(0,50);
  }  
} 

ISR(INT1_vect) {  
  if (debounce(&PIND, PD3)) { 
    slave_handler(1,18);
  }
}  


ISR(TWI_vect){
  switch (STATUS_CODE)
  {
  case 0x08:
    TWI_send_sla_w_or_r('w',slave);
    break;
  case 0x18:
    TWI_send_data(latestData,0);
    break;
  case 0x28:
    TWI_send_data(latestData,0);
    break;
  default:
    break;
  }
}

ISR(ADC_vect){
  static uint8_t counter = 0;
  static uint32_t sum = 0;

  sum+=ADC;
  ++counter;

  if(counter >= 16){
    latestData = (round(((float) (sum/16))/4));
    counter = 0;
    sum = 0;
  }

  ADCSRA |= (1<<ADSC);
}

void config();



int main(){
    DDRB |= (1<<PB2) |(1<<PB0) |(1<<PB1); 
    config(); ADC_config();
    interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE();
    interruptConfig_INT1_FULLY_READY_LOGICAL_CHANGE();
    while(1);
}




void ADC_config(){

  sei(); // set Globale Interrupt Enable

  ADC_Prescaler_Selections(16); // Select prescaler for ADC

  // uint16_t Selected_Clock_Bit = Clock_Select_Description_for_a_Timer_Counter_n(0,1024); /* 
  // select desired prescaler for desired Timer/Counter_n we will be using Timer/Counter0 with 
  // bit despcription 1024*/
  ADC_Noise_Reduse; // set ADC Noise Reduction
  ADMUX = (1<<REFS1)|(1<<REFS0); // Bruk intern 2.56V referanse
  ADCSRA |= (1<<ADEN) | (1<<ADIE) |(1<<ADSC);

  // TCCR0 |= (1<<WGM01);
  // TIMSK |= (1<<OCIE0);

  // uint16_t top = 4;
  // OCR0 = top;
}


void config(){
  sei();
  SET_PULL_UP_RESISTOR_ON_SDA_SCL;
  SET_SLAVE_ADRESS_7BIT(10);
  TWI_BIT_RATE_PRESCALER_1;
}

void slave_handler(const uint8_t VinPort,const uint8_t slave_addr){
  if(!slave){ 
    slave = slave_addr;
    TWI_START;
    ADMUX = VinPort;
    ADCSRA |=(1<<ADEN);
  }
  else if(slave){
    end_conn();
  } 
}

void end_conn(){
  slave = 0;
  ADCSRA &=~(1<<ADEN);
  TWI_STOP;
}