#define F_CPU 1000000UL 
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>
#include <ADC.h>
#include <math.h>

// constants 
#define ADC_SAMPLE_SIZE  16
#define SERVO_SLAVE_ADDRESS 50
#define STEPPER_SLAVE_ADDRESS 18
#define SERVO 0
#define STEPPER 1

// TWI communication
volatile uint8_t slave = 0;
volatile _Bool readyFlag = 0;
volatile uint8_t sendCount = 0;
volatile uint8_t buttonName = 0;
volatile uint16_t latestData = 0;

// Simple moving average
volatile uint32_t adcAvg = 0;
volatile uint8_t adcBuffer_index = 0;
volatile uint16_t adcBuffer[ADC_SAMPLE_SIZE];

// functions used in this AVR C code
void config();
void end_conn();
void ADC_config();
void SMA_update(); 
void DB_config_timer();
void communication_manager(volatile uint8_t buttonName);
void slave_handler(uint8_t VinPort, uint8_t slave_addr);


/////////////////////////// INTERRUPTS  start /////////////////////////

ISR(INT0_vect) {
  buttonName = SERVO;  
  DB_start_timer(2, 1024); 
} 

ISR(INT1_vect) { 
  buttonName = STEPPER;
  DB_start_timer(2,1024); 
}  

ISR(TIMER2_COMP_vect) {
  communication_manager(buttonName);
  DB_stop_timer(2);
}

ISR(TWI_vect){
  TWI_send_continues_2byte_data(slave, latestData,&sendCount, &readyFlag);
}

ISR(TIMER0_COMP_vect){
  if(!ADC_STATUS && !readyFlag){
    SMA_update(ADC);
  }
}

/////////////////////////// main() start/////////////////////////

int main(){
  DDRB |= (1<<PB2) |(1<<PB0) |(1<<PB1); 
  config(); 
  ADC_config();
  DB_config_timer();
  INT0_config_falling();
  INT1_config_falling();
  while(1);
}

/////////////////////////// FUNCTION start /////////////////////////

void ADC_config(){
  sei(); // set Globale Interrupt Enable
  ADC_Noise_Reduse; // set ADC Noise Reduction
  ADC_Prescaler_Selections(16); // Select prescaler for ADC
  // ADMUX = (1<<REFS1)|(1<<REFS0); // Bruk intern 2.56V referanse
  ADCSRA |= (1<<ADEN) |(1<<ADSC);

  TCCR0 |= (1<<WGM01);
  TIMSK |= (1<<OCIE0);

  uint16_t top = 4;
  OCR0 = top;
}


void config(){
  sei();
  SET_PULL_UP_RESISTOR_ON_SDA_SCL;
  SET_SLAVE_ADRESS_7BIT(10);
  TWI_BIT_RATE_PRESCALER_1;
}

void communication_manager(volatile uint8_t buttonName) {
  switch (buttonName)
  {
    case 0:
      if((PIND & (1<<PD2)) == 0) {
        slave_handler(0, SERVO_SLAVE_ADDRESS);
      }
      break;
    case 1:
      if((PIND & (1<<PD3)) == 0) {
        slave_handler(1, STEPPER_SLAVE_ADDRESS);
      }
      break;
    default:
      break;
  }
}

void slave_handler( uint8_t VinPort, uint8_t slave_addr){
  if(!slave){ 
    slave = slave_addr;
    PORTB ^= (1<<PB0);
    ADMUX = (ADMUX & 0xF0) | (VinPort & 0x0F);
    TIMER_perscalar_selct(0,1024); // turn on timer/clock 0 when needed 
    ADCSRA |=(1<<ADEN);
    TWI_START;
  }
  else if(slave){ 
    end_conn();
  } 
}

void end_conn(){
  slave = 0;
  readyFlag = 0;
  sendCount = 0;
  latestData = 0;
  ADCSRA &= ~(1<<ADEN);
  ADMUX &= 0xF0;
  TIMER_perscalar_selct(0,0); // turns off timer/clock 0 when not needed 
  TWI_STOP;
}

void DB_config_timer(){
  TCCR2 |= (1<<WGM21);
  TIMSK |= (1<<OCIE2);
  OCR2 = (uint8_t) 5;
}

void SMA_update(uint16_t newADC) {
  uint16_t oldADC = adcBuffer[adcBuffer_index];
  adcBuffer[adcBuffer_index] = newADC;

  adcAvg = adcAvg + newADC-oldADC;
  latestData = adcAvg/ADC_SAMPLE_SIZE;

  adcBuffer_index = (adcBuffer_index + 1) % ADC_SAMPLE_SIZE;
  readyFlag = 1;
}