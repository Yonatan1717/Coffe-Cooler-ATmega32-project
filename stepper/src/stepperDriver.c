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

#define TIMER2_TOP 5


volatile _Bool on = 0;
volatile _Bool reset = 0;
volatile _Bool readyFlag = 0;
volatile uint16_t recivedData = 500;
volatile _Bool enterSleepMode = 1;

void config();
void TIMER_config();
void STP_handle_data();
void STP_control_negativ(volatile _Bool *on);
void STP_control_positiv(volatile _Bool *on);



ISR(TWI_vect) {
  if (STATUS_CODE == 0x60) {
    PORTB ^= (1<<3);
    enterSleepMode = 0; // wake up 
    TIMER_perscalar_selct(2, 8); // Start Timer2
  }
  else if (STATUS_CODE == 0xA0) {
    PORTB ^= (1<<3);
    enterSleepMode = 1; // sleep
    TIMER_perscalar_selct(2, 0); // Stopp Timer2
  }

  TWI_recive_continues_2byte_data_slave(&recivedData, 500, &readyFlag); 
}

ISR(TIMER2_COMP_vect) {
  if(readyFlag) {
    STP_handle_data();
    readyFlag = 0;
  }
}


int main()
{ 
  config(); 
  TIMER_config();
  stepperDriverDDRx |= (1<<dir) | (1<<step); 

  while(1) {
    if(enterSleepMode) {
      SLEEP_enter_power_down();
    }
  };
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
  SERVO_config_timer1_c(); 
  TCCR2 |= (1<<WGM21);
  TIMSK |= (1<<OCIE2);
  uint16_t top = TIMER2_TOP;
  OCR2 = top;
}

void config(){
  DDRB = 255;
  sei();
  SET_SLAVE_ADRESS_7BIT(18);
  TWCR = (1<<TWEA)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT);
}

void STP_handle_data() {
  if(recivedData >= 800) {
    STP_control_negativ(&on);
    reset = 1;
  } else if(recivedData <= 200) {
    STP_control_positiv(&on);
    reset = 1;
  } else {
    if(reset) {
      dirLow;
      stepLow;
      on = 0;
      reset = 0;
    }   
  };
}