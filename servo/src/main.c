#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>

ISR(INT0_vect) {  
  if (debounce(&PIND, PD2)) SERVO_ANGLE(OCR1A,(90+45)); 
}
ISR(INT1_vect) {  
  if (debounce(&PIND, PD3)) SERVO_ANGLE(OCR1A,45);
} 

int main(){
  PWM_CONFIG_TIMER_CLOCK_1_OCR1A(0,50,1);
  interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE();
  interruptConfig_INT1_FULLY_READY_LOGICAL_CHANGE();
  while(1);
 }