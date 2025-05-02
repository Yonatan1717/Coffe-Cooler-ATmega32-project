#define F_CPU 1000000UL 
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
#include <I2C.h>
#include <ADC.h>
#include <math.h>

#define ADC_SAMPLE_SIZE  16
volatile uint8_t slave = 0;
volatile uint16_t latestData = 0;
volatile uint8_t sendCount = 0;
volatile _Bool ready = 0;
volatile uint8_t buttonNum = 0;
volatile uint16_t adcBuffer[ADC_SAMPLE_SIZE];
volatile uint8_t adcBuffer_index = 0;
volatile uint32_t adcAvg = 0;


void ADC_config();
void slave_handler(uint8_t VinPort, uint8_t slave_addr);
void end_conn();
void DB_config_timer();
void SMA_update(); 
  
ISR(INT0_vect) {
  buttonNum = 0;  
  DB_start_timer(2, 1024); 
} 

ISR(INT1_vect) { 
  buttonNum = 1;
  DB_start_timer(2,1024); 
}  

ISR(TIMER2_COMP_vect) {
  switch (buttonNum)
  {
    case 0:
      slave_handler(0,50);
      break;
    case 1:
      slave_handler(1,18);
      break;
    default:
      break;
  }
  DB_stop_timer(2);
}


ISR(TWI_vect){
  switch (STATUS_CODE)
  {
  case 0x08:
    TWI_send_sla_w_or_r('w',slave);
    break;
  case 0x18:
    if(sendCount == 0){
      TWI_send_data((latestData>>8),0);
      ++sendCount;
    } 
    else if(sendCount == 1){
      TWI_send_data((latestData & 0x00FF),0);
      sendCount = 0;
      ready = 0; 
    } 
    break;
  case 0x28:
    if(sendCount == 0){
      TWI_send_data((latestData>>8),0);
      ++sendCount;
    } 
    else if(sendCount == 1){
      TWI_send_data((latestData & 0x00FF),0);
      sendCount = 0;
      ready = 0; 
    }
    break;
  default:
    break;
  }
}

ISR(TIMER0_COMP_vect){
  if(!ADC_STATUS && !ready){
    SMA_update(ADC);
  }
}

void config();



int main(){
  DDRB |= (1<<PB2) |(1<<PB0) |(1<<PB1); 
  config(); 
  ADC_config();
  DB_config_timer();
  INT0_config_falling();
  INT1_config_falling();
  while(1);
}




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
  ready = 0;
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
}