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
volatile uint16_t recivedData = 500;
volatile uint8_t recivedCount = 0;
volatile _Bool ready = 0;

void stepperControlerNegatives(volatile _Bool *on);
void config();
void stepperControlerPositiv(volatile _Bool *on);
void Timer_config();



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
      recivedData = ((uint16_t)high_byte << 8) | low_byte;
      recivedCount = 0;
      ready = 1;
    }
    break;
  case 0xA0:
    recivedCount = 0;
    recivedData = 500;
    ready = 0;
    enLow; // for test
    dirLow; // for test
    stepLow; // for test
    TWI_return_to_not_addressed_slave();
    break;
  default:
    break;
  }
}

ISR(TIMER1_COMPA_vect){

  // PORTD ^= (1<<PD5)
  if(ready)
  {
    if(recivedData >= 1000){
      stepperControlerPositiv(&on);
    }
    else if(recivedData <= 200){
      stepperControlerNegatives(&on);
    }
    else{
      enLow;
      dirLow;
      stepLow;
    };
    
    ready = 0;
  }
  
}


int main()
{
  stepperDriverDDRx |= (1<<dir) | (1<<en) | (1<<step); 
  config();
  Timer_config();
  while(1);
}



void stepperControlerNegatives(volatile _Bool *on){ 
  enHigh;
  dirLow;
  if(!(*on)){
    stepHigh;
    *on = 1;
  } else if(*on){
    stepLow;
    *on = 0;
  }
}

void stepperControlerPositiv(volatile _Bool *on){ 
  enHigh;
  dirHigh;
  if(!(*on)){
    stepHigh;
    *on = 1;
  } else if(*on){
    stepLow;
    *on = 0;
  }
}

void Timer_config(){
  sei(); // set Globale Interrupt Enable
  Clock_Select_Description_for_a_Timer_Counter_n2(1,1024); 
  TCCR1B |= (1<<WGM12);
  TIMSK |= (1<<OCIE1A);
  uint16_t top = 500;
  OCR1A = top;
}

void config(){
  sei();
  SET_SLAVE_ADRESS_7BIT(18);
  TWCR = (1<<TWEA)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT);
}