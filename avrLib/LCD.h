//LCD
#define F_CPU 10000000UL
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

#define DDRData DDRA
#define PORTData PORTA

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
	RS_RW_D_D;
	E_WAIT_DISABLE_WAIT;
}

void DISPLAY_ON_OFF(){
	//Display on/off
	PORTData = (1<<DB3)|(0b100);
	RS_RW_D_D;
	E_WAIT_DISABLE_WAIT;
}

void ENTRY_MODE(){
	PORTData=(1<<DB2)|(0b10);
	RS_RW_D_D;
	E_WAIT_DISABLE_WAIT;
}

void RETURN_HOME(){
	PORTData = (1<<DB1);
	RS_RW_D_D;
	E_WAIT_DISABLE_WAIT;
}

void CLEAR_DISPLAY(){
	PORTData = (1<<DB0);
	RS_RW_D_D;
	E_WAIT_DISABLE_WAIT;
}

void WRITE_STRING(){
	char *str = "Ragnar Thomsen";
	int i;
	for (i=0;str[i]!=0;i++){
	
		PORTData = str[i];
		RS_RW_E_D;
		E_WAIT_DISABLE_WAIT;
	}
}

void WRITE_NUMBER(){

	//DDRAM_Adress
	PORTData = (1<<DB7)|(0x44);
	RS_RW_D_D;
	E_WAIT_DISABLE_WAIT;

	int Number = 2005;
	char N[16];
	//Enable RS, disable RW
	
	int i;
	itoa(Number,N,10);
	for (i=0;N[i]!=0;i++){
		PORTData=N[i];
		RS_RW_E_D;
		E_WAIT_DISABLE_WAIT;
	}
}

void WRITE_STRING_SINGLE(){
	PORTData = 'A';
	RS_RW_E_D;
	E_WAIT_DISABLE_WAIT;
}

void SEND_COMMAND(unsigned char cmnd){
	PORTData = cmnd;
	RS_RW_D_D;
	E_WAIT_DISABLE_WAIT;
}

void WRAP_AROUND(){
			SEND_COMMAND(0x1c);
			int compare;
			int counter=16;
			compare++;
			if(compare==counter){
				compare=0;
				for (int i=0;i<11;i++)
				{
					SEND_COMMAND(0x1c);
				}
				counter=29;
			}
			_delay_ms(50);
}
int main(){

	INIT_LCD();
	FUNCTION_SET();
	DISPLAY_ON_OFF();
	CLEAR_DISPLAY();
	ENTRY_MODE();
	WRITE_STRING();
	WRITE_NUMBER();
	
	while(1){
	}
	return 0;
	
}