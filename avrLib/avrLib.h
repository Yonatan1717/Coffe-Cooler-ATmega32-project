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


// servo
#define SERVO_MIDDLE(OCR1x) OCR1x = 1500 - 1
#define SERVO_R_90d_CLOCKWISE_FROM_MIDDLE(OCR1x) OCR1x = 500 - 1 
#define SERVO_R_90d_ANTI_CLOCKWISE_FROM_MIDDLE(OCR1x) OCR1x = 2500 - 1
#define SERVO_ANGLE_MOVE_STARTS_AT_ACLOCKWISE_90d(OCR1x, angle) if(OCR1x != 500-1 + (2000/180)*angle) OCR1x = 500-1 + (2000/180)*angle; else SERVO_MIDDLE(OCR1x);
#define SERVO_TURN_BASED_ON_ADC_RESULT(OCR1x, result) OCR1x = result + 500

// 1
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

// 2
void ACTIVATE_OUTPUT_PORTS(volatile uint8_t *DDRx_Register, uint8_t *DDxn){ //E.g. DDRC, DDC0, DDC3, DDC5
    for(uint8_t i = 0; (DDxn[i] != 0 || i == 0) && i<8; i++){
        *DDRx_Register |= (1<<DDxn[i]);
    }
}

// 3
void SET_PORTS(volatile uint8_t *PORTx_Register, uint8_t *Pxn){ //E.g. DDRC, DDC0, DDC3, DDC5
    for(uint8_t i = 0; (Pxn[i] != 0 || i == 0) && i<8; i++){
        *PORTx_Register |= (1<<Pxn[i]);
    }
}

// 4
void interruptConfig_INT0_FULLY_READY_LOGICAL_CHANGE() {
    sei();  
  // configuration for the interrupt  
  GICR |= (1 << INT0); // external interrupt request 0 enabled (INT0, not INT1)  
  MCUCR |= (1 << ISC00); // set ISC00 as one so that any logical change on INT0 generates an interrupt request  
  MCUCR &= ~(1 << ISC01); //clear ISC01 to make it a low level interrupt  

  DDRD &= ~(1 << PD2); // Set PD2 as input  
  PORTD |= (1 << PD2);  // Enable pull-up resistor on PD2  
} 

// 5
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
// 6
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

// 7
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
// 8
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

// 9
void ADC_AUTO_TRIGGER_FREERUNNING_MODE(){
    sei();
    ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADSC) | (1<<ADATE);
    SFIOR &= ~(1 << ADTS2 | 1 << ADTS1 | 1 << ADTS0);
    ADC_Prescaler_Selections(16);
}





































// //LCD
// void  LCD_RS_RW(uint8_t Rs){
//     cmd_port &= ~(1<<RS);
//     cmd_port |= (Rs << RS);
// }

// void wait(){
//     LCD_EE;
//     _delay_us(1);
//     LCD_DE;
//     _delay_ms(2);
// }

// void LCD_data(uint8_t data) {
//     LCD_RS_RW(1); 
//     data_port = data;
//     wait();
    
// }

// void LCD_ins(uint8_t cmd) {
//     LCD_RS_RW(0); 
//     data_port = cmd;
//     wait();    
// }
