#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>

ISR(INT0_vect) {  
  OCR1A = ADC_SINGLE_Vinput_RESULT;
  if(debounce(&PIND, PD2)) SERVO_ANGLE_MOVE_STARTS_AT_ACLOCKWISE_90d(OCR1A,(90+45)); 
}
ISR(INT1_vect) {  
  if(debounce(&PIND, PD3)) SERVO_ANGLE_MOVE_STARTS_AT_ACLOCKWISE_90d(OCR1A,45);
} 

ISR(ADC_vect){
  if( 0 <= ADC_SINGLE_Vinput_RESULT && ADC_SINGLE_Vinput_RESULT <= 250) PORTB = 1;
  else if( 251 <= ADC_SINGLE_Vinput_RESULT && ADC_SINGLE_Vinput_RESULT <= 500) PORTB = 2;
  else if( 501 <= ADC_SINGLE_Vinput_RESULT && ADC_SINGLE_Vinput_RESULT <= 750) PORTB = 4;
  else if( 751 <= ADC_SINGLE_Vinput_RESULT && ADC_SINGLE_Vinput_RESULT <= 1023) PORTB = 8;

}

void ADC_config(){

  sei(); // set Globale Interrupt Enable
  ADCSRA |= (1<<ADEN) | (1<<ADATE) | (ADIE);
  ADMUX = 0b00010000;
  ADC_Prescaler_Selections(16);
}

int main(){
  // DDRB = 0x0F;
  // ADC_AUTO_TRIGGER_FREERUNNING_MODE();
  ADC_config();
  PWM_CONFIG_TIMER_CLOCK_1_OCR1A(0,50,1);
  interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE();
  interruptConfig_INT1_FULLY_READY_LOGICAL_CHANGE();
  // ADC_Noise_Reduse;
  while(1);
 }