#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>
uint8_t enable_output_port_B[] = {0,1,2,3,4};
ISR(ADC_vect){
    int16_t diff_Result = ADC_differencial(5000,10);
    LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION_m(diff_Result,PORTB,enable_output_port_B);
}
int main(){
    NoiseReduse;
    uint8_t enable_ADCn_port[] = {4};
    Input_Channel_and_Gain_Selection_E(enable_ADCn_port);
    ADC_Prescaler_Selections(16);
    uint16_t Selected_Clock_Bit = Clock_Select_Description_for_a_Timer_Clock_n(0,1024);
    ADC_Auto_Trigger_Enable_E_ADATE_E_SFIOR_T0_Compare_Match(Selected_Clock_Bit, 263);
    ACTIVATE_REGISTERS_m(DDRB, enable_output_port_B);
    while (1);
    return 0;
}