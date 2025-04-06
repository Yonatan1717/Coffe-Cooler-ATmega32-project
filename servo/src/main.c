#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>
uint8_t recivedData = 0;




ISR(TWI_vect){
  switch (STATUS_CODE)
  {
  case 0x60:
    PORTB = 1;
    TWI_SET_TWINT_ACK;
    break;
  case 0x80:
    PORTB = 2;
    recivedData = TWI_recived_data(1);
    break;
  case 0xA0:
    PORTB = 4;
    TWI_return_to_not_addressed_slave();
    break;
  default:
    break;
  }

  SERVO_TURN_BASED_ON_ADC_RESULT(OCR1A,((recivedData*4)+1)*2);
}

void config(){
  DDRB = 255;
  sei();
  SET_SLAVE_ADRESS_7BIT(50);
  TWCR = (1<<TWEA)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT);
}

int main(){
  config();
  PWM_CONFIG_TIMER_CLOCK_1_OCR1A(0,50,1);

  while(1){}
 }