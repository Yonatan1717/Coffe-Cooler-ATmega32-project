//LCD
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <avr/io.h>
#include <util/delay.h>


#define RS PB0
#define RW PB1
#define E PB2

#define DB7 PA7
#define DB6 PA6
#define DB5 PA5
#define DB4 PA4
#define DB3 PA3
#define DB2 PA2
#define DB1 PA1
#define DB0 PA0

#define DDRCommand DDRB
#define PORTCommand PORTB

#define DDRData DDRD
#define PORTData PORTD 

//Disable RS/RW
#define RS_RW_D_D \
PORTCommand &= ~(1<<RS);\
PORTCommand &= ~(1<<RW);

//Enable RS, disable RW
#define RS_RW_E_D \
PORTCommand |= (1<<RS); \
PORTCommand &= ~(1<<RW);

//Enable RS, disable RW
#define RS_RW_D_E \
PORTCommand &= ~(1<<RS); \
PORTCommand |= (1<<RW);

//Enable RS, disable RW
#define RS_RW_E_E \
PORTCommand |= (1<<RS); \
PORTCommand |= (1<<RW);

#define E_WAIT_DISABLE_WAIT \
PORTCommand |= (1<<E); \
_delay_us(1); \
PORTCommand &= ~(1<<E); \
_delay_us(1530);


void PULS_CONTROL_ED(){
	RS_RW_E_D;
	E_WAIT_DISABLE_WAIT;
}

void PULS_CONTROL_DD(){
	RS_RW_D_D;
	E_WAIT_DISABLE_WAIT;
}

void SETUP(){
	//PB og PA porter
	DDRCommand |= (1<<RS)|(1<<RW)|(1<<E);//Command ports as output
	DDRData |= 0xFF; //Dataports as output
	_delay_us(1530);//IDK why
	//
}

void INIT_LCD(){
	DDRCommand |= (1<<RS)|(1<<RW)|(1<<E);
	DDRData |= 0xFF;
	_delay_us(1530);
}

void FUNCTION_SET(){
	PORTData = (1<<DB5)|(0b11100);
	PULS_CONTROL_DD();
}

void DISPLAY_ON_OFF(){
	//Display on/off
	PORTData = (1<<DB3)|(0b100);
	PULS_CONTROL_DD();
}

void ENTRY_MODE(){
	PORTData=(1<<DB2)|(0b10);
	PULS_CONTROL_DD();
}

void RETURN_HOME(){
	PORTData = (1<<DB1);
	PULS_CONTROL_DD();
}

void CLEAR_DISPLAY(){
	PORTData = (1<<DB0);
	PULS_CONTROL_DD();
}

void WRITE_STRING(char *str, uint8_t addr){
	PORTData = (1<<DB7)|(addr);
	PULS_CONTROL_DD();
	
	while(*str){
		PORTData = *str++;
		PULS_CONTROL_ED();
	}
}

void WRITE_NUMBER(int Number, uint8_t addr){
	//DDRAM_Adress
	PORTData = (1<<DB7)|(addr);
	PULS_CONTROL_DD();

	char buffer[16];

	itoa(Number,buffer,10);
	int i = 0;
	while(buffer[i]){
		PORTData= buffer[i];
		PULS_CONTROL_ED();
		++i;
	}
}

void WRITE_STRING_SINGLE(char charcter, uint8_t addr){
	PORTData = (1<<DB7)|(addr);
	PORTData = charcter;
	PULS_CONTROL_ED();
}

void SEND_COMMAND(uint8_t cmnd){
	PORTData = cmnd;
	PULS_CONTROL_DD();

}

void WRAP_AROUND(uint8_t len, uint8_t start){
			SEND_COMMAND(0x1c);

			static _Bool first = 1;
			static uint8_t counter = 16;
			if(first){
				first = 0;
				counter -= start;
			}
		
			static compare = 0;
			if(compare==counter){
				compare=0;
				for (int i=0;i<(40-len-16);i++)
				{
					SEND_COMMAND(0x1c);
				}
				counter = 16+len;
			}
			++compare;
			_delay_ms(50);
}
