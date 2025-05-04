#define F_CPU 1000000UL 
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>
#include <ADC.h>
#include <math.h>
#include <avr/sleep.h>

// constants 
#define STOP 2
#define SERVO 0
#define STEPPER 1
#define DB_TIMER_NUM 2
#define SERVO_ADC_PIN 0
#define ADC_TIMER_TOP 4
#define DB_PRESCALAR 1024
#define STEPPER_ADC_PIN 1
#define ADC_SAMPLE_SIZE  16
#define SERVO_SLAVE_ADDRESS 50
#define STEPPER_SLAVE_ADDRESS 18


// Sleep status
volatile _Bool sleepStatusAdc = 0;

// TWI communication
volatile uint8_t slave = 0;
volatile _Bool readyFlag = 0;
volatile uint8_t sendCount = 0;
volatile uint8_t buttonName = 0;
volatile uint16_t latestData = 0;

// Simple moving average
volatile uint32_t adcAvg = 0;
volatile uint16_t adcBuffer[ADC_SAMPLE_SIZE];
#if ADC_SAMPLE_SIZE > 255
  volatile uint16_t adcBuffer_index = 0;
#else
  volatile uint8_t adcBuffer_index = 0;
#endif

// functions used in this AVR C code
void setup();
void config();
void end_conn();
void ADC_config();
void SMA_update(); 
void DB_config_timer2();
void communication_manager(volatile uint8_t buttonName);
void slave_handler(volatile uint8_t VinPort, volatile uint8_t slave_addr);


/////////////////////////// INTERRUPTS  start /////////////////////////

// ISR(INT2_vect) {
//   sleepStatusAdc = 1;
//   buttonName = STOP;  
//   DB_start_timer(DB_TIMER_NUM, DB_PRESCALAR); 
// } 

// Exteral interrupt 
ISR(INT0_vect) {
  sleepStatusAdc = 1;
  buttonName = SERVO;  
  DB_start_timer(DB_TIMER_NUM, DB_PRESCALAR); 
} 

ISR(INT1_vect) {
  sleepStatusAdc = 1; 
  buttonName = STEPPER;
  DB_start_timer(DB_TIMER_NUM, DB_PRESCALAR); 
}  



// TWI interrupt
ISR(TWI_vect){ 
  TWI_send_continues_2byte_data(&slave, &latestData,&sendCount, &readyFlag);
}

// Timer interrupts 
ISR(TIMER2_COMP_vect) { // for debouncing
  communication_manager(buttonName);
  DB_stop_timer(DB_TIMER_NUM);
}

ISR(TIMER0_COMP_vect){ // for adc convertion start 
  PORTB ^= 2;
  if(!ADC_STATUS && !readyFlag){
    SMA_update(ADC); // read new ADC and update the average
    readyFlag = 1; // new data is ready

    ADCSRA |= (1<<ADSC); // start new conversion
  }
}  




/////////////////////////// main() start/////////////////////////

int main(){
  setup();
  INT0_config_onlow();
  INT1_config_onlow();
  // INT2_config_onlow();
      
  while(1) {
    if(sleepStatusAdc) {
      SLEEP_enter_adc();
    } else {
      SLEEP_enter_power_down();
    }
  };

}



/////////////////////////// FUNCTION start /////////////////////////


void setup() {
  config();  
  ADC_config();
  DB_config_timer2();
  DDRB |= (1<<PB2) |(1<<PB0) |(1<<PB1); 
}

// configs
void ADC_config() {
  sei(); // set Globale Interrupt Enable
  ADC_Prescaler_Selections(16); // Select prescaler for ADC
  // ADMUX = (1<<REFS1)|(1<<REFS0); // Bruk intern 2.56V referanse
  ADCSRA |= (1<<ADEN) | (1<<ADSC);
  TIMER_perscalar_selct(0,1024);
  TCCR0 |= (1<<WGM01);
  TIMSK |= (1<<OCIE0);
  uint16_t top = ADC_TIMER_TOP;
  OCR0 = top;
}


void config(){
  sei();
  SET_PULL_UP_RESISTOR_ON_SDA_SCL;
  SET_SLAVE_ADRESS_7BIT(10);
  TWI_BIT_RATE_PRESCALER_1;
}

// handlers
void communication_manager(volatile uint8_t buttonName) {
  switch (buttonName)
  {
    case 0:
        slave_handler(SERVO_ADC_PIN, SERVO_SLAVE_ADDRESS);
      break;
    case 1:
        slave_handler(STEPPER_ADC_PIN, STEPPER_SLAVE_ADDRESS);
      break;
    case 2:
        end_conn();
      break;
    default:
      break;
  }
}

void slave_handler( uint8_t VinPort, uint8_t slave_addr){
  if(!slave){
    PORTB ^= 1;
    setup();
    slave = slave_addr;
    ADMUX = (ADMUX & 0xF0) | (VinPort & 0x0F);
    ADCSRA |= (1<<ADEN);
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
  ADMUX &= 0xF0; 
  ADCSRA &= ~(1<<ADEN);
  TWI_STOP;

  sleepStatusAdc = 0;
}

// Simple Moving Average
void SMA_update(uint16_t newADC) {
  uint16_t oldADC = adcBuffer[adcBuffer_index];
  adcBuffer[adcBuffer_index] = newADC;

  adcAvg = adcAvg + newADC-oldADC;
  latestData = adcAvg/ADC_SAMPLE_SIZE;

  adcBuffer_index = (adcBuffer_index + 1) % ADC_SAMPLE_SIZE;
}
