#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>



uint8_t pressed(uint8_t pin_port, uint8_t bitPosition) {  
    static uint8_t buttonPressed = 1;  
    if ((pin_port & (1 << bitPosition)) == 0) {  
      if (buttonPressed == 1) {  
        buttonPressed = 0;  
        return 1;  
      }  
    } else {  
      buttonPressed = 1;  
    }  
    return 0;  
  }  
  
  uint8_t debounce(volatile uint8_t *pin_port, uint8_t bitPosition) {  
    // debounce function that takes in volatile variable uint8 pointer pin_port and integer bitPosition  
    uint32_t ms = 5;  
    if (pressed(*pin_port, bitPosition)) {  
      uint32_t counter = 0;  
      while (counter < ms * 1000 && (*pin_port & (1 << bitPosition)) == 0) {  
        // delay stops counting if the button is let go within 5ms  
        ++counter;  
      }  
      if ((*pin_port & (1 << bitPosition)) == 0) {  
        return 1;  
      }  
    }  
    return 0;  
  }  
  
  ISR(INT0_vect) {  
    // interrupt service routine using INT0 via port D2  
    if (debounce(&PIND, PD2)) {  
        TWI_START; 
    }  
  } 



ISR(TWI_vect){
    uint8_t slave_addr = 50;
    uint8_t recivedData = 0;
    
    
    switch (STATUS_CODE)
    {
        case 0x08:
            PORTA ^= (1<<PB0);
            TWI_SLA_R(slave_addr);
            TWI_SET_TWINT;
            break;
        case 0x10:
            PORTA ^= (1<<PB1);
            TWI_SET_TWINT;
            break;
        case 0x38:
            PORTA ^= (1<<PB2);
            TWI_SET_TWINT;
            break;
        case 0x40:
            PORTA ^= (1<<PB3);
            TWI_SET_TWINT;
            break;
        case 0x48:
            PORTA ^= (1<<PB4);
            TWI_SET_TWINT;
            break;
        case 0x50:
            
            recivedData = TWDR; 
            if(recivedData == 0xB) PORTA ^= (1<<PB5);
            TWI_STOP;
            break;
        case 0x58:
            PORTA ^= (1<<PB6);
            // TWI_STOP;
            // TWI_SET_TWINT;
            break;
        default:
            break;
    }

    

}

int main(){
    sei();
    SET_PULL_UP_RESISTOR_ON_SDA_SCL;
    DDRA = 0b01111111;
    SET_SLAVE_ADRESS_7BIT(18);
    interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE();

    while(1);
    return 0;
}