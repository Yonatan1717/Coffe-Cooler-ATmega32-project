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

  ADC_Auto_Trigger_Enables_A_Lot_Of_Things_uT0(Selected_Clock_Bit, 100);
  // this function does all the nessesary configuration to for ADC Auto Trigger to work.
}

ISR(INT2_vect){
  if(debounce(&PINB,PB2)){
    pressedJoyStick = 1;
  }
}
  
ISR(INT0_vect) {  
  if (debounce(&PIND, PD2)) { 
    PORTD ^=(1<<PD7);
    if(!slave){ 
      slave = 50;
      PORTB ^= (1<<PB3); // Debug
      TWI_START;
      ADC_config();
    }
    else{
      slave = 0;
      PORTD ^= (1<<PD6); //debug      
      ADCSRA &= ~((1 << ADEN) | (1 << ADIE) | (1 << ADATE));
      SFIOR &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));
      TWI_STOP;

    } 
    
  }  
} 


ISR(TWI_vect){
  switch (STATUS_CODE)
  {
  case 0x08:
    TWI_send_sla_w_or_r('w',50);
    PORTB = 1;
    break;
  case 0x18:
    if(pressedJoyStick){
      TWI_send_stop();
      pressedJoyStick = 0;
    }
    PORTB = 2;
    TWI_send_data(latestData,0);
    break;
  case 0x28:
    if(pressedJoyStick){
      TWI_send_stop();
      pressedJoyStick = 0;
    }
    PORTB = 4;
    TWI_send_data(latestData,0);
    break;
  default:
    break;
  }
  
}

ISR(ADC_vect){
  latestData = round(ADC/4) - 1;
}

void config(){
  sei();
  SET_PULL_UP_RESISTOR_ON_SDA_SCL;
  SET_SLAVE_ADRESS_7BIT(10);
  TWI_BIT_RATE_PRESCALER_1;
}



int main(){
    DDRD |= (1<<PD7) |(1<<PD6);
    DDRB |= (1<<PB3);
    config();
    interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE();
    interruptConfig_INT1_FULLY_READY_LOGICAL_CHANGE();
    while(1);
}