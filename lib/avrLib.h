// biblotek for avr prosjekter
#include <avr/io.h>
#include <avr/interrupt.h>

void ADC_Prescaler_Selections(uint8_t bit){
    // side 217, Table 85

    if(bit == 8) ADCSRA |= (1<<ADPS1) | (1<<ADPS0);
    else ADCSRA |= (1<<ADPS2);
}

void Input_Channel_and_Gain_Selection_E(uint8_t ADCn_port[]){
    uint8_t size = sizeof(ADCn_port);
    for(uint8_t i = 0; i<size; i++) ADMUX |= (1<<ADCn_port[i]);
}
void Input_Channel_and_Gain_Selection_D(uint8_t ADCn_port[]){
    uint8_t size = sizeof(ADCn_port);
    for(uint8_t i = 0; i<size; i++) ADMUX ^= (1<<ADCn_port[i]);
}

void ADC_Auto_Trigger_Enable_E_ADATE_E_SFIOR_T0_Compare_Match(){
    // side 216, Table 84

    //legg til ønsket tidsintervall
    ADCSRA |= (1<<ADATE);

    //legg til auto trigger source

    //side 218, Table 86 ADC Auto Trigger Source
    SFIOR ^= (1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0); //Clears register
    SFIOR |= (1<<ADTS1)|(1<<ADTS0); //Enables T0 Compare match Trigger source
}



