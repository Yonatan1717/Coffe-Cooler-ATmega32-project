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

uint8_t recivedData = 70;
uint16_t dataStepper = 500;

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

  uint16_t Selected_Clock_Bit = Clock_Select_Description_for_a_Timer_Counter_n(1,1024); 

  TCCR1B |= (1<<WGM12);
  TIMSK |= (1<<OCIE1A);

  uint16_t top = 500;
  OCR1A = top;
}



ISR(TWI_vect){
  switch (STATUS_CODE)
  {
  case 0x60:
    TWI_SET_TWINT_ACK;
    break;
  case 0x80:
    PORTD |= (1<<PD6);
    recivedData = TWI_recived_data(1);
    break;
  case 0xA0:
    recivedData = 70;
    enLow; // for test
    dirLow; // for test
    stepLow; // for test
    PORTD &= ~(1<<PD6);
    dataStepper = 500;
    TWI_return_to_not_addressed_slave();
    break;
  default:
    break;
  }
}

ISR(TIMER1_COMPA_vect){

  // PORTD ^= (1<<PD5);
  if(recivedData != 70){
    dataStepper = ((recivedData*8)+2)*2;
  }

  if(dataStepper >= 1000){
    stepperControlerPositiv(&on);
  }
  else if(dataStepper <= 200){
    stepperControlerNegatives(&on);
  }
  else{
    enLow;
    dirLow;
    stepLow;
  };
}

void config(){
  DDRB = 255;
  sei();
  SET_SLAVE_ADRESS_7BIT(18);
  TWCR = (1<<TWEA)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT);
}



int main(){
  stepperDriverDDRx |= (1<<dir) | (1<<en) | (1<<step); 
  DDRD |= (1<<PD6)| (1<<PB5);
  config();
  Timer_config();
  while(1);
 }