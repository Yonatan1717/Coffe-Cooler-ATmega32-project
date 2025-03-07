// biblotek for avr prosjekter
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>

#define NoiseReduse MCUCR = (1<<SM0)
#define ACTIVATE_REGISTERS_m(DDRx, DDxn_liste) ACTIVATE_REGISTERS(&DDRx, DDxn_liste)
#define LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION_m(v_diff,PORT_NAME, PORTs) LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION(v_diff, &PORT_NAME,PORTs)
// 1
void ADC_Prescaler_Selections(uint8_t bit){
    // side 217, Table 85
    ADCSRA &= ~((1<<0));
    ADCSRA &= ~((1<<1));
    ADCSRA &= ~((1<<2));
    
    if(bit == 8) ADCSRA |= (1<<1) | (1<<0);
    else if(bit == 16) ADCSRA |= (1<<2);
    else exit(3);
}

// 2
void Input_Channel_and_Gain_Selection_E(uint8_t ADCn_porter_du_Onsker_Aktivert_i_Stignede_rekke_folge[]){
    // side 214, tabell 84, kan også brukes til å aktivere alle andre bits i ADMUX

    for(uint8_t i = 0; ADCn_porter_du_Onsker_Aktivert_i_Stignede_rekke_folge[i] != 0 || i == 0; i++) ADMUX |= (1<<ADCn_porter_du_Onsker_Aktivert_i_Stignede_rekke_folge[i]);
}

// 3
void Input_Channel_and_Gain_Selection_D(uint8_t ADCn_porter_du_Onsker_Deaktivert_i_Stignede_rekke_folge[]){
    // side 214, tabell 84, kan også brukes til å deaktivere alle andre bits i ADMUX
    for(uint8_t i = 0; ADCn_porter_du_Onsker_Deaktivert_i_Stignede_rekke_folge[i] != 0 || i == 0; i++) ADMUX ^= (1<<ADCn_porter_du_Onsker_Deaktivert_i_Stignede_rekke_folge[i]);
}

// 4
void ADC_Auto_Trigger_Enable_E_ADATE_E_SFIOR_T0_Compare_Match(uint16_t prescaler, uint16_t timeintervall_ms){
    
    ///     Sets T0 CTC mode and calculates OCRn value for compare match      ///

    //side 80, Table 38.

    //Regn ut n_OCRn for OCRn for å utføre compare match i Timer0
    uint16_t TOP = round((timeintervall_ms*1000)/prescaler);
    OCR0 = TOP;

    //OBS OBS!!! TCNT0 vil ikke bli høyere enn 255 ettersom 8 bit.
     
    //Enable CTC for T0
    TCCR0 = (1<<WGM01);
    
    ///     Enables Auto Trigger with T0 CTC as source    ///
    
    // side 216, Table 84

    //Auto trigger for ADC enable
    ADCSRA |= (1<<ADATE);

    //legg til auto trigger source

    //side 218, Table 86 ADC Auto Trigger Source
    SFIOR &= ~((1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0)); //Clears register
    SFIOR |= (1<<ADTS1)|(1<<ADTS0); //Enables T0 Compare match Trigger source



}

// 5
int Clock_Select_Description_for_a_Timer_Clock_n(uint8_t timer_clock_num, uint16_t bit_description){
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
                TCCR0 |= (1<<CS00);
            }else if(timer_clock_num == 1){
                TCCR1B |= (1<<CS10);
            }else if(timer_clock_num == 2){
                TCCR2 |= (1<<CS20);
            }
            break;
        case 8:
            if(timer_clock_num == 0){
                TCCR0 |= (1<<CS01);
            }else if(timer_clock_num == 1){
                TCCR1B |= (1<<CS11);
            }else if(timer_clock_num == 2){
                TCCR2 |= (1<<CS21);
            }
            break;
        case 64:
            if(timer_clock_num == 0){
                TCCR0 |= (1<<CS01) |(1<<CS00);
            }else if(timer_clock_num == 1){
                TCCR1B |= (1<<CS10) | (1<< CS11);
            }else if(timer_clock_num == 2){
                TCCR2 |= (1<<CS22);
            }
            break;
        case 256:
            if(timer_clock_num == 0){
                TCCR0 |= (1<<CS02);
            }else if(timer_clock_num == 1){
                TCCR1B |= (1<<CS12);
            }else if(timer_clock_num == 2){
                TCCR2 |= (1<<CS22) | (1<<CS21);
            }
            break;
        case 1024:
            if(timer_clock_num == 0){
                TCCR0 |= (1<<CS00) | (1<<CS02);
            }else if(timer_clock_num == 1){
                TCCR1B |= (1<<CS10) | (1<<CS12);
            }else if(timer_clock_num == 2){
                TCCR2 |= (1<<CS20) | (1<<CS21) | (1<<CS22);
            }
            break;
        default:
            exit(2);
    }

    return bit_description;
}

// 8
void ACTIVATE_REGISTERS(uint8_t *DDRx_Register, uint8_t DDxn[]){ //E.g. DDRC, DDC0, DDC3, DDC5

    uint8_t size_DDxn = sizeof(DDxn); //DDR which you desire to set to

    for(uint8_t i = 0; i<size_DDxn; i++){
        *DDRx_Register |= (1<<DDxn[i]);
    }

}

// 9
void LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION(uint8_t V_Difference, uint8_t *PORT_NAME,uint8_t PORT_NAMES[]){
    //ADDS 2500 to V_difference since SWITCH statements can't be zero
    uint8_t difference = ((V_Difference+2500)/1000);

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

// 10
int16_t ADC_differencial(uint16_t Vref, uint8_t bitsUsed_10_or_8){
    // side 217
    
    //add lavere byte av resultat til ADC 
    int16_t ADC_resultat = ADCL;
    //add høyre byte av resultat til ADC
    ADC_resultat |= (ADCH<<8);

    if ((ADC_resultat & (1<<9)))
    {
        ADC_resultat |= (0b11111100 <<8);
    }

    int16_t vq = (Vref/pow(2,(bitsUsed_10_or_8-1))) * (ADC_resultat+1/2);
    
    return vq;
}


