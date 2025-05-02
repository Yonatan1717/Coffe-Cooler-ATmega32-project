#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SET_SLAVE_ADRESS_7BIT(value) TWAR |= (value << 1)

#define ACTIVATE_REGISTERS_m(DDRx, DDxn_liste) ACTIVATE_REGISTERS(&DDRx, DDxn_liste)

unsigned char ADMUX = 0;

unsigned char TCCR0 = 0;
unsigned char TCCR2 = 0;
unsigned char TCCR1B = 0;

unsigned char ADCH = 0;
unsigned char ADCL = 0;
unsigned char DDRB = 0;
unsigned char PORTB = 0;

unsigned char SFIOR = 7;
unsigned char TWAR = 0;




void Input_Channel_and_Gain_Selection_E(unsigned char ADCn_porter_du_Onsker_Aktivert[8]){
    // side 124, tabell 84, kan også brukes til å aktivere alle andre bits i ADMUX

    for(unsigned char i = 0; ADCn_porter_du_Onsker_Aktivert[i] != 0 || i == 0; i++) 
    {
        ADMUX |= (1<<ADCn_porter_du_Onsker_Aktivert[i]);
        printf("%i\n",ADCn_porter_du_Onsker_Aktivert[i]);
    }

}

void Input_Channel_and_Gain_Selection_D(unsigned char ADCn_porter_du_Onsker_Deaktivert[8]){
    // side 125, tabell 84, kan også brukes til å deaktivere alle andre bits i ADMUX

    for(unsigned char i = 0; ADCn_porter_du_Onsker_Deaktivert[i] != 0; i++) 
    {
        ADMUX ^= (1<<ADCn_porter_du_Onsker_Deaktivert[i]);
        printf("%i\n",ADCn_porter_du_Onsker_Deaktivert[i]);
    }
}

int Clock_Select_Description_for_a_Timer_Clock_n(unsigned char timer_clock_num, unsigned short int bit_description){
    // side 127 tabell 54 for clock 2
    // side 110 tabell 48 for clock 1
    // side 82 tabell 42 for clock 0

    // exit status code 1: ugyldig timer/clock Number
    // exit status code 2: ugyldig bit description
    if(timer_clock_num != 0 && timer_clock_num != 1 && timer_clock_num != 2){
        exit(1);
    }

    switch (bit_description)
    {
        case 1:
            if(timer_clock_num == 0){
                TCCR0 |= (1<<0);
            }else if(timer_clock_num == 1){
                TCCR1B |= (1<<0);
            }else if(timer_clock_num == 2){
                TCCR2 |= (1<<0);
            }
            break;
        case 8:
            if(timer_clock_num == 0){
                TCCR0 |= (1<<1);
            }else if(timer_clock_num == 1){
                TCCR1B |= (1<<1);
            }else if(timer_clock_num == 2){
                TCCR2 |= (1<<1);
            }
            break;
        case 64:
            if(timer_clock_num == 0){
                TCCR0 |= (1<<1) |(1<<0);
            }else if(timer_clock_num == 1){
                TCCR1B |= (1<<0) | (1<< 1);
            }else if(timer_clock_num == 2){
                TCCR2 |= (1<<2);
            }
            break;
        case 256:
            if(timer_clock_num == 0){
                TCCR0 |= (1<<2);
            }else if(timer_clock_num == 1){
                TCCR1B |= (1<<2);
            }else if(timer_clock_num == 2){
                TCCR2 |= (1<<2) | (1<<1);
            }
            break;
        case 1024:
            if(timer_clock_num == 0){
                TCCR0 |= (1<<0) | (1<<2);
            }else if(timer_clock_num == 1){
                TCCR1B |= (1<<0) | (1<<2);
            }else if(timer_clock_num == 2){
                TCCR2 |= (1<<0) | (1<<1) | (1<<2);
            }
            break;
        default:
            exit(2);
    }

    return bit_description;
}

short int ADC_differencial(){
    short int vADC = ADCL;
    vADC |= (ADCH << 8);
    vADC |= (ADCH<<8);

    if ((ADCH & (1<<1)))
    {
        vADC |= (0b11111100 <<8);
        
    }
    
    return vADC;
}



void LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION(short V_Difference, unsigned char *PORT_NAME,unsigned char PORT_NAMES[]){
    //ADDS 2500 to V_difference since SWITCH statements can't be zero
    unsigned char difference = round(((V_Difference+2500)/1000));
    printf("%i\n",difference);

        switch(difference){
        case 0:
            *PORT_NAME = (1<<PORT_NAMES[0]);
            break;
        case 1:
            *PORT_NAME = (1<<PORT_NAMES[1]);
            break;
        case 2:
            *PORT_NAME = (1<<PORT_NAMES[2]);
            break;
        case 3:
            *PORT_NAME = (1<<PORT_NAMES[3]);
            break;
        case 4:
            *PORT_NAME = (1<<PORT_NAMES[4]);
            break;
        }
}

void ACTIVATE_REGISTERS(unsigned char *DDRx_Register, unsigned char DDxn[]){ //E.g. DDRC, DDC0, DDC3, DDC5

    unsigned char size_DDxn = sizeof(DDxn); //DDR which you desire to set to

    for(unsigned char i = 0; i<size_DDxn; i++){
        *DDRx_Register |= (1<<DDxn[i]);
    }

}

int main(){

    // char *helllo = "hello0000";

    // printf("%i", sizeof(helllo)-1);
    int check = 0;
    check = !check;
    printf("%i           ", check);
    check = !check;
    printf("%i", check);

    return 0;
}


