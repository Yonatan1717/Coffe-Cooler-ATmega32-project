#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avrLib.h>

uint8_t enable_output_port_c[] = {0,1,2,3,4}; // the output ports that are going to be used
uint16_t Vref = 4000; // known refranse voltage 

ISR(ADC_vect){
    int16_t diff_Result = ADC_differencial(Vref,10);
    LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION_m(diff_Result,PORTC,enable_output_port_c);
}

void ADC_config(){

    sei(); // set Globale Interrupt Enable

    //---------------------------------------------------------------------------------------
    ADC_Noise_Reduse; // set ADC Noise Reduction

    //---------------------------------------------------------------------------------------
    ADC_Prescaler_Selections(16); // Select prescaler for ADC

    //---------------------------------------------------------------------------------------
    uint8_t ADMUX_set_bits[] = {4,6}; /* set bit 4 and 6 in register ADMUX to enable
    "AVCC with external capacitor at AREF pin" and to use ADC0 port 
    as positiv differtiona input and ADC1 port as negativ differntial input. */

    Input_Channel_and_Gain_Selection_Set_ADMUX_bits(ADMUX_set_bits); //
    //---------------------------------------------------------------------------------------
    uint16_t Selected_Clock_Bit = Clock_Select_Description_for_a_Timer_Counter_n(0,1024); /* 
    select desired prescaler for desired Timer/Counter_n we will be using Timer/Counter0 with 
    bit despcription 1024*/

    //---------------------------------------------------------------------------------------
    ADC_Auto_Trigger_Enables_A_Lot_Of_Things_uT0(Selected_Clock_Bit, 262);
    // this function does all the nessesary configuration to for ADC Auto Trigger to work.

}

int main()
{   
    ADC_config();
    ACTIVATE_REGISTERS_m(DDRC, enable_output_port_c); // activate the desired output ports

    while (1);
    return 0; // empty infinite while loop to keep the program going 
}