#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <util/delay.h>

#define ADC_ACTIVATE_PORTS_USING_ADC(v_diff,PORT_NAME, PORTs) ADC_activate_ports_using_adc(v_diff, &PORT_NAME,PORTs)
#define ADC_STATUS (ADCSRA & (1<<ADSC))

// 1
void ADC_Prescaler_Selections(uint8_t bit){
    // side 217, Table 85
    ADCSRA &= ~((1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2));
    
    if(bit == 8) ADCSRA |= (1<<ADPS1) | (1<<ADPS0);
    else if(bit == 16) ADCSRA |= (1<<ADPS2);
    else if(bit == 32) ADCSRA |= (1<<ADPS2) | (1<<ADPS0);
    
}


// 2
void ADMUX_set(uint8_t ADMUX_bits[]){
    // side 214, tabell 84, kan også brukes til å aktivere alle andre bits i ADMUX
    ADMUX &= 0xF0;
    for(uint8_t i = 0; (ADMUX_bits[i] != 0 || i == 0) && i<8; i++) ADMUX |= (1<<ADMUX_bits[i]);
}
// 3
void ADMUX_clear(uint8_t ADMUX_bits[]){
    // side 214, tabell 84, kan også brukes til å deaktivere alle andre bits i ADMUX
    for(uint8_t i = 0; (ADMUX_bits[i] != 0 || i == 0) && i<8; i++) ADMUX &= ~(1<<ADMUX_bits[i]);
}
// 4
void ADC_adate_timer0(uint16_t prescaler, uint16_t timeintervall_ms){
    ///     Sets T0 CTC mode and calculates OCRn value for compare match      ///

    //side 80, Table 38.
    TCCR0 = (1<<WGM01);
    TIMSK = (1 << OCIE0);
    
    //Regn ut n_OCRn for OCRn for å utføre compare match i Timer0
    uint16_t TOP = round((timeintervall_ms*1000)/prescaler);
    if(TOP > 255) TOP = 255;
    OCR0 = (uint8_t) TOP;
    TCCR0 |= (1 << CS00) | (1 << CS02);
    
    ///     Enables Auto Trigger with T0 CTC as source    ///
    
    // side 216, Table 84
    SFIOR &= ~((1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0)); //Clears register
    SFIOR |= (1<<ADTS1)|(1<<ADTS0); //Enables T0 Compare match Trigger source
    
    //Auto trigger for ADC enable
    ADCSRA |= (1<<ADATE) | (1<<ADIE) | (1<< ADEN);

}
// 5
int16_t ADC_differencial(uint16_t Vref, uint8_t bitsUsed_10_or_8){
    // side 217
    
    //add høyre og lavere byte av resultat til ADC 
    int16_t ADC_resultat = (ADCH <<8) | ADCL;

    if ((ADC_resultat & (1<<9))) ADC_resultat |= 0xFC00;
    
    int16_t Vdiff =  ((ADC_resultat + 0.5) * Vref) / (512);
    
    return Vdiff;
}
// 6
void ADC_activate_ports_using_adc(int16_t V_Difference, volatile uint8_t *PORT_NAME,uint8_t PORT_NAMES[]){
    //ADDS 2500 to V_difference since SWITCH statements can't be zero
    uint8_t difference = round(((V_Difference+2500)/1000));

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
        default:
            ACTIVATE_output_ports(PORT_NAME,PORT_NAMES);
            break;
        }
}


void SLEEP_enter_adc() {
    set_sleep_mode(SLEEP_MODE_ADC);
    sleep_enable();
    sei();           // Enable global interrupts
    sleep_cpu();     // Enter sleep
    sleep_disable(); // Immediately disable after waking
}

// 9
void ADC_adate_freerunning(){
    sei();
    ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADSC) | (1<<ADATE);
    SFIOR &= ~(1 << ADTS2 | 1 << ADTS1 | 1 << ADTS0);
    ADC_Prescaler_Selections(16);
}