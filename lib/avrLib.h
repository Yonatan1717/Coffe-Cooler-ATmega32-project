// biblotek for avr prosjekter
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>

void ADC_Prescaler_Selections(uint8_t bit){
    // side 217, Table 85

    if(bit == 8) ADCSRA |= (1<<ADPS1) | (1<<ADPS0);
    else ADCSRA |= (1<<ADPS2);
}

void Input_Channel_and_Gain_Selection_E(uint8_t ADCn_porter_du_Onsker_Aktivert[]){
    // side 124, tabell 84, kan også brukes til å aktivere alle andre bits i ADMUX

    uint8_t size = sizeof(ADCn_porter_du_Onsker_Aktivert);
    for(uint8_t i = 0; i<size; i++) ADMUX |= (1<<ADCn_porter_du_Onsker_Aktivert[i]);
}
void Input_Channel_and_Gain_Selection_D(uint8_t ADCn_porter_du_Onsker_Deaktivert[]){
    // side 125, tabell 84, kan også brukes til å deaktivere alle andre bits i ADMUX

    uint8_t size = sizeof(ADCn_porter_du_Onsker_Deaktivert);
    for(uint8_t i = 0; i<size; i++) ADMUX ^= (1<<ADCn_porter_du_Onsker_Deaktivert[i]);
}

void ADC_Auto_Trigger_Enable_E_ADATE_E_SFIOR_T0_Compare_Match(uint16_t prescaler, uint16_t timeintervall_ms){
    //side 80, Table 38.

    //Regn ut n_OCRn for OCRn for å utføre compare match i Timer0
    uint16_t time_period = 1/(F_CPU/prescaler);
    uint16_t TCNT0 = round((timeintervall_ms/1000)/time_period);

    //OBS OBS!!! TCNT0 vil ikke bli høyere enn 255 ettersom 8 bit.
     
    //Enable CTC for T0
    TCCR0 = (1<<WGM01);
    

    ///     ///     ///
 

    // side 216, Table 84

    //legg til ønsket tidsintervall
    ADCSRA |= (1<<ADATE);

    //legg til auto trigger source

    //side 218, Table 86 ADC Auto Trigger Source
    SFIOR ^= (1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0); //Clears register
    SFIOR |= (1<<ADTS1)|(1<<ADTS0); //Enables T0 Compare match Trigger source


}

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



