// biblotek for avr prosjekter
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>

#define ADC_Noise_Reduse MCUCR = (1<<SM0) // side 32, tabell 13
#define ACTIVATE_REGISTERS_m(DDRx, DDxn_liste) ACTIVATE_REGISTERS(&DDRx, DDxn_liste)
#define LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION_m(v_diff,PORT_NAME, PORTs) LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION(v_diff, &PORT_NAME,PORTs)

// // Enable I2C
// #define TWI_ENABLE TWCR = (1<<TWEN)

// Set TWINT
#define TWI_SET_TWINT TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA)

// // Enable ACK
// #define TWI_ENABLE_ACK TWCR = (1<<TWEA)

// Send start condition
#define TWI_START TWCR = (1<<TWINT) | (1<<TWEN) | (1<< TWSTA)| (1<<TWIE)| (1<<TWEA)

// Transmit stop condition
#define TWI_STOP TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) | (1<<TWEA)

// // I2C enable interrupt
// #define TWI_ENABLE_INTERRUPT TWCR |= (1<<TWIE)



#define STATUS_CODE (TWSR & 0xF8)

#define TWI_BIT_RATE_PRESCALER_1 TWSR &= ~((1<<TWPS1)|(1<<TWPS0)) 
#define TWI_BIT_RATE_PRESCALER_8 TWSR = (1<<TWPS0)
#define TWI_BIT_RATE_PRESCALER_16 TWSR = (1<<TWPS1)
#define TWI_BIT_RATE_PRESCALER_64 TWSR = (1<<TWPS1) | (1<<TWPS0)

#define SET_SLAVE_ADRESS_7BIT(this_slave_addr_7bit) TWAR |= (this_slave_addr_7bit << 1)
#define TWI_SLA_W(slave_addr_7bit) TWDR = (slave_addr_7bit << 1)  
#define TWI_SLA_R(slave_addr_7bit) TWDR = (slave_addr_7bit << 1) | 1


#define SET_PULL_UP_RESISTOR_ON_SDA_SCL DDRC &= ~((1<<PC1)|(1<<PC0)); PORTC |= (1<<PC1) | (1<<PC0)

// 1
void ADC_Prescaler_Selections(uint8_t bit){
    // side 217, Table 85
    ADCSRA &= ~((1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2));
    
    if(bit == 8) ADCSRA |= (1<<ADPS1) | (1<<ADPS0);
    else if(bit == 16) ADCSRA |= (1<<ADPS2);
    else exit(3);
}

// 2
void Input_Channel_and_Gain_Selection_Set_ADMUX_bits(uint8_t ADMUX_bits[]){
    // side 214, tabell 84, kan også brukes til å aktivere alle andre bits i ADMUX
    ADMUX &= 0xF0;
    for(uint8_t i = 0; (ADMUX_bits[i] != 0 || i == 0) && i<8; i++) ADMUX |= (1<<ADMUX_bits[i]);
}

// 3
void Input_Channel_and_Gain_Selection_Clear_ADMUX_bits(uint8_t ADMUX_bits[]){
    // side 214, tabell 84, kan også brukes til å deaktivere alle andre bits i ADMUX
    for(uint8_t i = 0; (ADMUX_bits[i] != 0 || i == 0) && i<8; i++) ADMUX ^= (1<<ADMUX_bits[i]);
}

// 4
void ADC_Auto_Trigger_Enables_A_Lot_Of_Things_uT0(uint16_t prescaler, uint16_t timeintervall_ms){
    
    
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
int Clock_Select_Description_for_a_Timer_Counter_n(uint8_t timer_clock_num, uint16_t bit_description){
    // side 127 tabell 54 for clock 2
    // side 110 tabell 48 for clock 1
    // side 82 tabell 42 for clock 0

    // exit status code 1: ugyldig timer/clock Number
    // exit status code 2: ugyldig bit description
    if(timer_clock_num > 2){
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
void ACTIVATE_REGISTERS(volatile uint8_t *DDRx_Register, uint8_t *DDxn){ //E.g. DDRC, DDC0, DDC3, DDC5
    for(uint8_t i = 0; (DDxn[i] != 0 || i == 0) && i<8; i++){
        *DDRx_Register |= (1<<DDxn[i]);
    }
}

// 9
void LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION(int16_t V_Difference, volatile uint8_t *PORT_NAME,uint8_t PORT_NAMES[]){
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
            ACTIVATE_REGISTERS(PORT_NAME,PORT_NAMES);
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

    if ((ADC_resultat & (1<<9))) ADC_resultat |= (0b11111100 <<8);
    
    int16_t Vdiff =  ((ADC_resultat + 0.5) * Vref) / (512);
    
    return Vdiff;
}



//////////////////////////////////////// funksjoner for A5 //////////////////////////////////////////////////////

uint8_t resiveData(uint8_t dest_slave_addr_7bit){
    uint8_t recivedData = 0;
 

    return recivedData;
}




void interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE() {  
  // configuration for the interrupt  
  GICR |= (1 << INT0); // external interrupt request 0 enabled (INT0, not INT1)  
  MCUCR |= (1 << ISC00); // set ISC00 as one so that any logical change on INT0 generates an interrupt request  
  MCUCR &= ~(1 << ISC01); //clear ISC01 to make it a low level interrupt  

  DDRD &= ~(1 << PD2); // Set PD2 as input  
  PORTD |= (1 << PD2);  // Enable pull-up resistor on PD2  
} 