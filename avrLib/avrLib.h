// biblotek for avr prosjekter
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <util/delay.h>


#define cmd_ddrx DDRC
#define cmd_port PORTC
#define E PC1
#define RS PC0

#define data_ddrx DDRD
#define data_port PORTD

#define LCD_cmd_OM cmd_ddrx |= (1<<E) | (1<<RS)
#define LCD_data_OM data_ddrx = 0xFF

#define LCD_EE cmd_port |= (1<<E)
#define LCD_DE cmd_port &= ~(1<<E)

// nyttige macros //
#define ACTIVATE_OUTPUT_PORTS_m(DDRx, DDxn_liste) ACTIVATE_OUTPUT_PORTS(&DDRx, DDxn_liste)
#define SET_PORTS_m(PORTx, Pxn_list) ACTIVATE_OUTPUT_PORTS(&PORTx, Pxn_list)
#define SET_PORT(PORTx, Pxn) PORTx |= (1<<Pxn)
#define CLEAR_PORT(PORTx, Pxn) PORTx &= ~(1<<Pxn)
#define TOGGLE_PORT(PORTx, Pxn) PORTx ^= (1<<Pxn)


// A4 //
#define ADC_Noise_Reduse MCUCR = (1<<SM0) // side 32, tabell 13
#define LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION_m(v_diff,PORT_NAME, PORTs)LED_ACTIVATE_DESIRED_PORTS_ADC_CONVERSION(v_diff, &PORT_NAME,PORTs)
#define ADC_SINGLE_Vinput_RESULT ((ADCH<<8) | (ADCL))

// servo
#define SERVO_MIDDLE(OCR1x) OCR1x = 1500 - 1
#define SERVO_R_90d_CLOCKWISE_FROM_MIDDLE(OCR1x) OCR1x = 500 - 1 
#define SERVO_R_90d_ANTI_CLOCKWISE_FROM_MIDDLE(OCR1x) OCR1x = 2500 - 1
#define SERVO_ANGLE_MOVE_STARTS_AT_ACLOCKWISE_90d(OCR1x, angle) if(OCR1x != 500-1 + (2000/180)*angle) OCR1x = 500-1 + (2000/180)*angle; else SERVO_MIDDLE(OCR1x);
#define SERVO_TURN_BASED_ON_ADC_RESULT(OCR1x, result) OCR1x = result + 500


////////////////////////////////////// funksjoner for A4 /////////////////////////////////////////

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
void ACTIVATE_OUTPUT_PORTS(volatile uint8_t *DDRx_Register, uint8_t *DDxn){ //E.g. DDRC, DDC0, DDC3, DDC5
    for(uint8_t i = 0; (DDxn[i] != 0 || i == 0) && i<8; i++){
        *DDRx_Register |= (1<<DDxn[i]);
    }
}

// 7
void SET_PORTS(volatile uint8_t *PORTx_Register, uint8_t *Pxn){ //E.g. DDRC, DDC0, DDC3, DDC5
    for(uint8_t i = 0; (Pxn[i] != 0 || i == 0) && i<8; i++){
        *PORTx_Register |= (1<<Pxn[i]);
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
            ACTIVATE_OUTPUT_PORTS(PORT_NAME,PORT_NAMES);
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
void interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE() {
    sei();  
  // configuration for the interrupt  
  GICR |= (1 << INT0); // external interrupt request 0 enabled (INT0, not INT1)  
  MCUCR |= (1 << ISC00); // set ISC00 as one so that any logical change on INT0 generates an interrupt request  
  MCUCR &= ~(1 << ISC01); //clear ISC01 to make it a low level interrupt  

  DDRD &= ~(1 << PD2); // Set PD2 as input  
  PORTD |= (1 << PD2);  // Enable pull-up resistor on PD2  
} 

void interruptConfig_INT1_FULLY_READY_LOGICAL_CHANGE() { 
    sei(); 
    // configuration for the interrupt  
    GICR |= (1 << INT1); // external interrupt request 0 enabled (INT0, not INT1)  
    MCUCR |= (1 << ISC10); // set ISC10 as one so that any logical change on INT0 generates an interrupt request  
    MCUCR &= ~(1 << ISC11); //clear ISC01 to make it a low level interrupt  
  
    DDRD &= ~(1 << PD3); // Set PD2 as input  
    PORTD |= (1 << PD3);  // Enable pull-up resistor on PD2  
  }

////////////////////// debounce /////////////////////////////

uint8_t pressed(uint8_t pin_port, uint8_t bitPosition) {  
    static uint8_t buttonPressed = 1;  
    if ((pin_port & (1 << bitPosition)) == 0) {  
      if (buttonPressed == 1) {  
        buttonPressed = 0;  
        return 1;  
      }  
    } else {  
      buttonPressed = 1;  
    }  
    return 0;  
  }  
  
uint8_t debounce(volatile uint8_t *pin_port, uint8_t bitPosition) {  
// debounce function that takes in volatile variable uint8 pointer pin_port and integer bitPosition  
if (pressed(*pin_port, bitPosition)) {  
    _delay_ms(5);
    if ((*pin_port & (1 << bitPosition)) == 0) {  
    return 1;  
    }  
}  
return 0;  
}  


///////////////////// for servo motor ////////////////////////

uint32_t PWM_CONFIG_TIMER_CLOCK_1_OCR1A(uint8_t type_0_fast_1_phase_correct, uint16_t frequency, uint16_t prescaler){
    uint32_t TOP = 0;
    if(!type_0_fast_1_phase_correct){
        TCCR1A |= (1<<WGM11);
        TCCR1B |= (1<<WGM12) | (1<<WGM13);
        TCCR1A |= (1<<COM1A1); TCCR1A &= ~(1<<COM1A0);
        DDRD |= (1<<PD5);
    
        Clock_Select_Description_for_a_Timer_Counter_n(1,1);
        TOP = round((F_CPU/(prescaler*frequency)) - 1);
        ICR1 = TOP;
        SERVO_MIDDLE(OCR1A);
    }

    return TOP;
}

void ADC_AUTO_TRIGGER_FREERUNNING_MODE(){
    sei();
    ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADSC) | (1<<ADATE);
    SFIOR &= ~(1 << ADTS2 | 1 << ADTS1 | 1 << ADTS0);
    ADC_Prescaler_Selections(16);
}


//LCD
void  LCD_RS_RW(uint8_t Rs){
    cmd_port &= ~(1<<RS);
    cmd_port |= (Rs << RS);
}

void wait(){
    LCD_EE;
    _delay_us(1);
    LCD_DE;
    _delay_ms(2);
}

void LCD_data(uint8_t data) {
    LCD_RS_RW(1); 
    data_port = data;
    wait();
    
}



void LCD_ins(uint8_t cmd) {
    LCD_RS_RW(0); 
    data_port = cmd;
    wait();    
}
