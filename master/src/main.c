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
      if(recivedData == 0xA0) CLEAR_PORT(PORTB, PB0);
      else if(recivedData == 0xA1) SET_PORT(PORTB, PB0);
    }

    if(requested_data == 0xBB){
      if(recivedData == 0xB0) CLEAR_PORT(PORTB, PB1);
      else if(recivedData == 0xB1) SET_PORT(PORTB, PB1);
    }

}

void config(){
  sei();
  SET_PULL_UP_RESISTOR_ON_SDA_SCL;
  SET_SLAVE_ADRESS_7BIT(50);
}

int main(){
    config();
    interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE();
    interruptConfig_INT1_FULLY_READY_LOGICAL_CHANGE();

    uint8_t activate_B_ports[] = {0,1};
    ACTIVATE_OUTPUT_PORTS_m(DDRB, activate_B_ports);


    while(1);
    return 0;
}