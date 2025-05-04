// biblotek for avr prosjekter
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <util/delay.h>


void USART_config(_Bool type)
{
  if(type) {
    sei();
    UCSRB = (1<<RXEN) | (1<<TXEN) | (1<<RXCIE);
  } else {
    UCSRB = (1<<RXEN) | (1<<TXEN);
  }
  
}

void USART_sendData(uint8_t data){
  if (( UCSRA & (1<<UDRE))){
    UDR = data;
  }
}
