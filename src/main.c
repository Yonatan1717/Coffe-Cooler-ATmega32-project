#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>

uint8_t enable_output_port_C[] = {0,1,2,3,4}; // the output ports that are going to be used
uint16_t Vref = 3000; // known refranse voltage 

ISR(ADC_vect){
    int16_t diff_Result = ADC_differencial(Vref,10);
    LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION_m(diff_Result,PORTC,enable_output_port_C);
}

int main()
{   sei(); // set Globale Interrupt Enable
    ADC_Noise_Reduse; // set ADC Noise Reduction
    //---------------------------------------------------------------------------------------
    ACTIVATE_REGISTERS_m(DDRC, enable_output_port_C);
    //---------------------------------------------------------------------------------------
    ADC_Prescaler_Selections(16);
    //---------------------------------------------------------------------------------------
    uint8_t ADCn_port_config_ADMUX[] = {4,6};
    Input_Channel_and_Gain_Selection_E_ADCn_ports(ADCn_port_config_ADMUX);
    //---------------------------------------------------------------------------------------
    uint16_t Selected_Clock_Bit = Clock_Select_Description_for_a_Timer_Clock_n(0,1024);
    ADC_Auto_Trigger_Enables_A_Lot_Of_Things_uT0(Selected_Clock_Bit, 262);
    //---------------------------------------------------------------------------------------
    while (1); return 0; // empty infinite while loop to keep the program going 
}