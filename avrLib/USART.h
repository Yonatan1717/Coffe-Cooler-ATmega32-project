// biblotek for avr prosjekter
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <util/delay.h>


void USART_config(_Bool type_0_sender_1_reciver)
{
  // initialize and set baud rate to 9600 with 0.2% error margine
  UBRRL = 12;
  UCSRA |= (1<<RXEN) | (1<<TXEN) | (1<< U2X);
  
  if(type_0_sender_1_reciver) {
    sei();
    UCSRB |= (1<<RXCIE);
  }
}

void USART_sendData(uint8_t data){
  if (( UCSRA & (1<<UDRE))){
    UDR = data;
  }
}
