#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>

volatile uint16_t recivedData = 0;
volatile _Bool readyFlag = 0;
volatile _Bool isReciving = 0;


ISR(TWI_vect){
  if(STATUS_CODE == 0x60) isReciving = 1;
  if(STATUS_CODE == 0xA0) isReciving = 0;
  TWI_recive_continues_2byte_data_slave(&recivedData, 0, &readyFlag);

  if(readyFlag){
    SERVO_TURN_BASED_ON_ADC_RESULT(OCR1A,recivedData*2);
    readyFlag = 0;
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
  SERVO_config_timer1_nc(50,1);

  while(1) {
    if(!isReciving) SLEEP_enter_power_down();
  };
      
 }