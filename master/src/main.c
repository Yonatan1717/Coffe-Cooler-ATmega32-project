#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>

uint8_t requested_data = 0;

  
ISR(INT0_vect) {  
  // interrupt service routine using INT0 via port D2  
  if (debounce(&PIND, PD2)) {  
    requested_data = 0xAA;
    TWI_START; 
  }  
} 

ISR(INT1_vect) {  
  // interrupt service routine using INT0 via port D2  
  if (debounce(&PIND, PD3)) {  
    requested_data = 0xBB;
    TWI_START; 
  }  
} 



ISR(TWI_vect){
    // uint8_t slave_addr = 50;
    uint8_t recivedData = reciveData_REQUESTED_AND_THEN_CLOSE_CONNECTION_PR(18,requested_data);
    
    if(requested_data == 0xAA){
      switch (recivedData)
      {
        case 0xA0:
          PORTD &= ~(1<<PB7);
          break;
        case 0xA1:
          PORTD |= (1<<PB7);
          break;
        default:
          break;
      }
    }

    if(requested_data == 0xBB){
      switch (recivedData)
      {
        case 0xB0:
          PORTA &= ~(1<<PB7);
          break;
        case 0xB1:
          PORTA |= (1<<PB7);
          break;
        default:
          break;
      }
    }

}

int main(){
    DDRD = 0x80;
    DDRB = 255;
    sei();
    SET_PULL_UP_RESISTOR_ON_SDA_SCL;
    DDRA = 0b11111111; // kunn for debuging ikke nødvendign
    SET_SLAVE_ADRESS_7BIT(50);
    interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE();
    interruptConfig_INT1_FULLY_READY_LOGICAL_CHANGE();

    while(1);
    return 0;
}