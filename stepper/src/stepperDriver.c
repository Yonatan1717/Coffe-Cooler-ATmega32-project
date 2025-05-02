#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>

#define stepperDriverPORT PORTB
#define stepperDriverDDRx DDRB

#define dir PB0
#define en PB1
#define step PB2

#define dirHigh stepperDriverPORT |= (1<<dir)
#define enHigh stepperDriverPORT |= (1<<en)
#define stepHigh stepperDriverPORT |= (1<<step)

#define dirLow stepperDriverPORT &= ~(1<<dir)
#define enLow stepperDriverPORT &= ~(1<<en)
#define stepLow stepperDriverPORT &= ~(1<<step)


volatile _Bool on = 0;
volatile _Bool readyFlag = 0;
const uint16_t servoSpeed = 50;
volatile uint8_t recivedCount = 0;
volatile uint16_t recivedData = 500;

void config();
void TIMER_config();
void STP_control_negativ(volatile _Bool *on);
void STP_control_positiv(volatile _Bool *on);



ISR(TWI_vect){
  static uint8_t high_byte = 0; 
  static uint8_t low_byte = 0;
  switch (STATUS_CODE)
  {
  case 0x60:
    TWI_SET_TWINT_ACK;
    break;
  case 0x80:
    if (recivedCount == 0) {
      high_byte = TWI_recived_data(1);
      ++recivedCount;
    }
    else if (recivedCount == 1) {
      low_byte = TWI_recived_data(1);
      recivedData = ((uint16_t) high_byte << 8) | low_byte;
      recivedCount = 0;
      readyFlag = 1;
    }
    break;
  case 0xA0:
    recivedCount = 0;
    recivedData = 500;
    readyFlag = 0;
    on = 0;
    dirLow; // for test
    stepLow; // for test
    TWI_return_to_not_addressed_slave();
    break;
  default:
    break;
  }
}

ISR(TIMER0_COMP_vect){

  if(readyFlag)
  {
    if(recivedData >= 800){
      STP_control_positiv(&on);
      SERVO_CCW(OCR1A,26+servoSpeed);
    }
    else if(recivedData <= 200){
      STP_control_negativ(&on);
      SERVO_CW(OCR1A, 26+servoSpeed);
    }
    else{
      SERVO_STOP(OCR1A);
      dirLow;
      stepLow;
    };
    
    readyFlag = 0;
  }
  
}


int main()
{
  stepperDriverDDRx |= (1<<dir) | (1<<en) | (1<<step); 
  config();
  TIMER_config();
  while(1);
}



void STP_control_negativ(volatile _Bool *on){ 
  
  dirLow;
  if(!(*on)){
    stepHigh;
    *on = 1;
  } else if(*on){
    stepLow;
    *on = 0;
  }
}

void STP_control_positiv(volatile _Bool *on){ 
  dirHigh;
  if(!(*on)){
    stepHigh;
    *on = 1;
  } else if(*on){
    stepLow;
    *on = 0;
  }
}

void TIMER_config(){
  sei(); // set Globale Interrupt Enable
  TIMER_perscalar_selct(0,8);
  SERVO_config_timer1_c(); 
  TCCR0 |= (1<<WGM01);
  TIMSK |= (1<<OCIE0);
  uint16_t top = 5;
  OCR0 = top;
}

void config(){
  DDRB = 255;
  sei();
  SET_SLAVE_ADRESS_7BIT(18);
  TWCR = (1<<TWEA)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT);
}