// biblotek for avr prosjekter
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <util/delay.h>



void USART_config()
{
  sei();
  UCSRB = (1<<RXEN) | (1<<TXEN) | (1<<RXCIE);
}

void USART_sendData(uint8_t data){
  if (( UCSRA & (1<<UDRE))){
    UDR = data;
  }
}
