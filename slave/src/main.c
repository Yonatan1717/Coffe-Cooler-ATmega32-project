/*
 * I2C Slave.c
 *
 * Created: 20/03/2025 10:09:38
 * Author : Marshed Mohamed
 */ 
 #define F_CPU 1000000UL
 #include <avr/io.h>
 #include <util/delay.h>
 #include <avrLib.h>

 uint8_t LEDStatusA = 0xA0; 		//Initializing the initial status of LED_A (off)
 uint8_t LEDStatusB = 0xB0; 		//Initializing the initial status of LED_B (off)
 uint8_t presseddCounterA = 0;	//Initializing presseddCounter for button_A
 uint8_t presseddCounterB = 0;	//Initializing presseddCounter for button_B
 unsigned char receiveData;		//Initializing storage of received data
 unsigned char sendData = 0xFF;	//Initializing storage of data to be sent 

 ISR(INT0_vect) {  
  if (debounce(&PIND, PD2)) { 
    if (LEDStatusB == 0xB0)					//If the LED_B is OFF
    {
      PORTB |= (1<<PB1);					//Switch on LED_B
      LEDStatusB = 0xB1;					//Save status that the LED_B is on
    }
    else if (LEDStatusB == 0xB1)			//If the LED_B is ON
    {
      PORTB &= ~(1<<PB1);					//Switch off LED_B
      LEDStatusB = 0xB0;					//Save status that the LED_B is off
    }
  }  
  
} 

ISR(INT1_vect) {  
  if (debounce(&PIND, PD3)) {  
    if (LEDStatusA == 0xA0)					//If the LED_A is OFF 
    {
      PORTB |= (1<<PB0);					//Switch on LED_A
      LEDStatusA = 0xA1;					//Save status that LED_A is on
    } 
    else if (LEDStatusA == 0xA1)			//If the LED_A is ON
    {
      PORTB &= ~(1<<PB0);					//Switch off LED_A
      LEDStatusA = 0xA0;					//Save status that LED_A is off
    }
  }  
} 
 
 int main(void)
 {
   DDRB |= (1<<PB0)|(1<<PB1);		//Setting PB0 and PB1 as output
 
   PORTD |= (1<<PD2)|(1<<PD3);		//Setting pull up resistor at PD2 and PD3
   
   TWAR = (18<<TWA0);				//Setting the device address to 18 (first 7 MSB)
   TWCR = (1<<TWEA)|(1<<TWEN);		//Enabling TWI with ACK; own SLA will be recognized
 
 
   
   interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE();
   interruptConfig_INT1_FULLY_READY_LOGICAL_CHANGE();
   
   // hello
   
   while (1)
   { 
     if ((TWSR & 0xF8) == 0x60)					//If own SLA+W has been received
     {			
       TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN);	//Data byte will be received and ACK will be returned 
     }
     
     if ((TWSR & 0xF8) == 0x80)					//If we have received data ACK has been returned 
     {
       receiveData = TWDR;						//Read the data
       TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN);	//Data byte will be received and ACK will be returned
       
       if (receiveData == 0xAA)				//If the data received is status of LED_A
       {
         sendData = LEDStatusA;				//Store status of LED_A in the data to be sent
       } 
       else if (receiveData == 0xBB)			//If the data received is status of LED_B	
       {
         sendData = LEDStatusB;				//Store status of LED_B in the data to be sent
       }
     }
     
     if ((TWSR & 0xF8) == 0xA0)					//If we have received a repeated START or STOP
     {
       TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN);	//Switch to the not addressed Slave mode; own SLA will be recognized 
     }
     
     if ((TWSR & 0xF8) == 0xA8)					//If we have received own SLA+R
     {
       TWDR = sendData;						//Load data for transmission						
       TWCR = (1<<TWINT)|(1<<TWEN);			//Last data byte will transmitted and NOT ACK should be received
     }
     
     if ((TWSR & 0xF8) == 0xC0)					//If data byte has been transmitted and NOT ACK has been received
     {
       TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN);	//Switch to the not  addressed Slave mode; own SLA will be recognized
     }
 
   }
   
 }
 

 