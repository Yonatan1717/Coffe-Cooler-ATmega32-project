#define F_CPU 1000000UL 
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>
#include <ADC.h>
#include <math.h>

uint8_t recived_data = 0;
volatile uint8_t slave = 0;
_Bool pressedJoyStick = 0;
uint8_t latestData = 0;

void ADC_config(){

  sei(); // set Globale Interrupt Enable

  ADC_Noise_Reduse; // set ADC Noise Reduction

  ADC_Prescaler_Selections(16); // Select prescaler for ADC

  uint16_t Selected_Clock_Bit = Clock_Select_Description_for_a_Timer_Counter_n(0,1024); /* 
  select desired prescaler for desired Timer/Counter_n we will be using Timer/Counter0 with 
  bit despcription 1024*/
  ADCSRA |=(1<<ADEN);

  TCCR0 |= (1<<WGM01);
  TIMSK |= (1<<OCIE0);

  uint16_t top = 4;
  OCR0 = top;
}

ISR(INT2_vect){
  if(debounce(&PINB,PB2)){
    pressedJoyStick = 1;
  }
}
  
ISR(INT0_vect) {  
  if (debounce(&PIND, PD2)) { 
    if(!slave){ 
      slave = 50;
      PORTB ^= (1<<PB0); // Debug
      TWI_START;
      ADCSRA |=(1<<ADEN);
    }
    else{
      slave = 0;
      ADCSRA &=~(1<<ADEN);
      PORTB ^= (1<<PD2); //debug      
      TWI_STOP;

    } 
    
  }  
} 


ISR(TWI_vect){
  switch (STATUS_CODE)
  {
  case 0x08:
    TWI_send_sla_w_or_r('w',50);
    break;
  case 0x18:
    if(pressedJoyStick){
      TWI_send_stop();
      pressedJoyStick = 0;
    }
    TWI_send_data(latestData,0);
    break;
  case 0x28:
    if(pressedJoyStick){
      TWI_send_stop();
      pressedJoyStick = 0;
    }
    TWI_send_data(latestData,0);
    break;
  default:
    break;
  }
  
}

ISR(TIMER0_COMP_vect){
  PORTB |= (1<<PB1);
  if(!ADC_STATUS){
    ADCSRA |= (1<<ADSC);
    latestData = round(ADC/8) - 2;
  }
}

void config(){
  sei();
  SET_PULL_UP_RESISTOR_ON_SDA_SCL;
  SET_SLAVE_ADRESS_7BIT(10);
  TWI_BIT_RATE_PRESCALER_1;
}



int main(){
    DDRB |= (1<<PB2) |(1<<PB0) |(1<<PB1); 
    config(); ADC_config();
    interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE();
    interruptConfig_INT1_FULLY_READY_LOGICAL_CHANGE();
    while(1);
}