#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>
volatile uint16_t recivedData = 0;
volatile uint8_t recivedCount = 0;
volatile _Bool ready = 0;


ISR(TWI_vect){
  static uint8_t high_byte = 0; 
  static uint8_t low_byte = 0;
  switch (STATUS_CODE)
  {
  case 0x60:
    PORTB = 1;
    TWI_SET_TWINT_ACK;
    break;
  case 0x80:
    PORTB = 2;
    if (recivedCount == 0) {
      high_byte = TWI_recived_data(1);
      ++recivedCount;
    }
    else if (recivedCount == 1) {
      low_byte = TWI_recived_data(1);
      recivedData = ((uint16_t)high_byte << 8) | low_byte;
      recivedCount = 0;
      ready = 1;
    }
    break;
  case 0xA0:
    PORTB ^= 1;
    recivedCount = 0;
    recivedData = 0;
    ready = 0;
    TWI_return_to_not_addressed_slave();
    break;
  default:
    break;
  }

  if(ready){
    SERVO_TURN_BASED_ON_ADC_RESULT(OCR1A,recivedData*2);
    ready = 0;
  }
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

  while(1);
 }