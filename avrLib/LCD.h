//LCD
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <avr/io.h>
#include <util/delay.h>


#define RS PD4
#define RW PD5
#define E PD6

#define DB7 PA7
#define DB6 PA6
#define DB5 PA5
#define DB4 PA4
#define DB3 PA3
#define DB2 PA2
#define DB1 PA1
#define DB0 PA0

#define DDRCommand DDRD
#define PORTCommand PORTD

#define DDRData DDRB
#define PORTData PORTB

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


void LCD_puls_control_ed(){
	RS_RW_E_D;
	E_WAIT_DISABLE_WAIT;
}

void LCD_puls_control_dd(){
	RS_RW_D_D;
	E_WAIT_DISABLE_WAIT;
}

void LCD_setup(){
	//PB og PA porter
	DDRCommand |= (1<<RS)|(1<<RW)|(1<<E);//Command ports as output
	DDRData |= 0xFF; //Dataports as output
	_delay_us(1530);//IDK why
	//
}

void LCD_function_set(){
	PORTData = (1<<DB5)|(0b11100);
	LCD_puls_control_dd();
}

void LCD_display_on(){
	//Display on
	PORTData = (1<<DB3)|(0b100);
	LCD_puls_control_dd();
}

void LCD_entry_mode(){
	PORTData=(1<<DB2)|(0b10);
	LCD_puls_control_dd();
}

void LCD_return_home(){
	PORTData = (1<<DB1);
	LCD_puls_control_dd();
}

void LCD_clear_display(){
	PORTData = (1<<DB0);
	LCD_puls_control_dd();
}

void LCD_write_string(char *str, uint8_t addr){
	PORTData = (1<<DB7)|(addr);
	LCD_puls_control_dd();
	
	while(*str){
		PORTData = *str++;
		LCD_puls_control_ed();
	}
}

void LDC_write_number(int Number, uint8_t addr){
	//DDRAM_Adress
	PORTData = (1<<DB7)|(addr);
	LCD_puls_control_dd();

	char buffer[16];

	itoa(Number,buffer,10);
	int i = 0;
	while(buffer[i]){
		PORTData= buffer[i];
		LCD_puls_control_ed();
		++i;
	}
}

void LDC_write_string_no_addr(char *str){
	while(*str){
		PORTData = *str++;
		LCD_puls_control_ed();
	}
}

void LDC_write_number_no_addr(int Number){
	char buffer[16];

	itoa(Number,buffer,10);
	int i = 0;
	while(buffer[i]){
		PORTData= buffer[i];
		LCD_puls_control_ed();
		++i;
	}
}

void LCD_write_character(char charcter, uint8_t addr){
	PORTData = (1<<DB7)|(addr);
	PORTData = charcter;
	LCD_puls_control_ed();
}

void LCD_write_character_no_addr(char charcter){
	PORTData = charcter;
	LCD_puls_control_ed();
}

void LCD_send_command(uint8_t cmnd){
	PORTData = cmnd;
	LCD_puls_control_dd();

}

void LCD_wrap_aroun_effect(uint8_t len, uint8_t start){
			LCD_send_command(0x1c);

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
					LCD_send_command(0x1c);
				}
				counter = 16+len;
			}
			++compare;
			_delay_ms(50);
}
