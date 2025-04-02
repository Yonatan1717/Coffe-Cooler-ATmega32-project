#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>


ISR(ADC_vect){
  SERVO_TURN_BASED_ON_ADC_RESULT(OCR1A,ADC);
}


int main(){
  ADC_AUTO_TRIGGER_FREERUNNING_MODE();
  PWM_CONFIG_TIMER_CLOCK_1_OCR1A(0,50,1);
  ADC_Noise_Reduse;
  while(1);
 }