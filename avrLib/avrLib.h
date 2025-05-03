// biblotek for avr prosjekter
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>

// error returns //
#define TIMER_INVALID_ID 11
#define TIMER_INVALID_PRESCALER 12

// nyttige macros //
#define ACTIVATE_OUTPUT_PORTS_m(DDRx, DDxn_liste) ACTIVATE_OUTPUT_PORTS(&DDRx, DDxn_liste)
#define SET_PORTS_m(PORTx, Pxn_list) ACTIVATE_OUTPUT_PORTS(&PORTx, Pxn_list)
#define SET_PORT(PORTx, Pxn) PORTx |= (1<<Pxn)
#define CLEAR_PORT(PORTx, Pxn) PORTx &= ~(1<<Pxn)
#define TOGGLE_PORT(PORTx, Pxn) PORTx ^= (1<<Pxn)

// servo //
#define SERVO_MIDDLE(OCR1x) OCR1x = 1500 - 1
#define SERVO_R_90d_CLOCKWISE_FROM_MIDDLE(OCR1x) OCR1x = 500 - 1 
#define SERVO_R_90d_ANTI_CLOCKWISE_FROM_MIDDLE(OCR1x) OCR1x = 2500 - 1
#define SERVO_ANGLE_MOVE_STARTS_AT_ACLOCKWISE_90d(OCR1x, angle) OCR1x = 500-1 + (2000/180)*(angle)
#define SERVO_TURN_BASED_ON_ADC_RESULT(OCR1x, result) OCR1x = (result) + 500

// servo continues // 
const uint16_t servorStop = 1437;
#define SERVO_STOP(OCR1x) OCR1x = servorStop
#define SERVO_CCW(OCR1x, speed) OCR1x = (servorStop) + (speed)
#define SERVO_CW(OCR1x, speed) OCR1x = (servorStop) - (speed)


// 1
int TIMER_perscalar_selct(uint8_t timer_clock_id, uint16_t bit_description){
    // side 127 tabell 54 for clock 2
    // side 110 tabell 48 for clock 1
    // side 82 tabell 42 for clock 0

    // exit status code 1: ugyldig timer/clock Number
    // exit status code 2: ugyldig bit description
    if(timer_clock_id > 2){
        return TIMER_INVALID_ID;
    }

    switch (bit_description)
    {
        case 0:
            if(timer_clock_id == 0){
                TCCR0 &= ~((1<<CS00) | (1<<CS01) | (1<<CS02));
            }else if(timer_clock_id == 1){
                TCCR1B &= ~((1<<CS10) | (1<<CS11) | (1<<CS12));
            }else if(timer_clock_id == 2){
                TCCR2 &= ~((1<<CS20) | (1<<CS21) | (1<<CS22));
            }
            break;
        case 1:
            if(timer_clock_id == 0){
                TCCR0 |= (1<<CS00);
            }else if(timer_clock_id == 1){
                TCCR1B |= (1<<CS10);
            }else if(timer_clock_id == 2){
                TCCR2 |= (1<<CS20);
            }
            break;
        case 8:
            if(timer_clock_id == 0){
                TCCR0 |= (1<<CS01);
            }else if(timer_clock_id == 1){
                TCCR1B |= (1<<CS11);
            }else if(timer_clock_id == 2){
                TCCR2 |= (1<<CS21);
            }
            break;
        case 64:
            if(timer_clock_id == 0){
                TCCR0 |= (1<<CS01) |(1<<CS00);
            }else if(timer_clock_id == 1){
                TCCR1B |= (1<<CS10) | (1<< CS11);
            }else if(timer_clock_id == 2){
                TCCR2 |= (1<<CS22);
            }
            break;
        case 256:
            if(timer_clock_id == 0){
                TCCR0 |= (1<<CS02);
            }else if(timer_clock_id == 1){
                TCCR1B |= (1<<CS12);
            }else if(timer_clock_id == 2){
                TCCR2 |= (1<<CS22) | (1<<CS21);
            }
            break;
        case 1024:
            if(timer_clock_id == 0){
                TCCR0 |= (1<<CS00) | (1<<CS02);
            }else if(timer_clock_id == 1){
                TCCR1B |= (1<<CS10) | (1<<CS12);
            }else if(timer_clock_id == 2){
                TCCR2 |= (1<<CS20) | (1<<CS21) | (1<<CS22);
            }
            break;
        default:
            return TIMER_INVALID_PRESCALER;
    }

    return bit_description;
}


///////////////////// debounce ////////////////////////

// 2
void DB_start_timer(uint8_t timer_clock_id, uint16_t prescaler) {
    switch (timer_clock_id)
    {
    case 0:
        TCNT0 = 0;
        break; 
    case 1:
        TCNT1 = 0;
        break;
    case 2:
        TCNT2 = 0;
        break;
    default:
        break;
    }
    TIMER_perscalar_selct(timer_clock_id, prescaler);
}
// 3
void DB_stop_timer(uint8_t timer_clock_id) {
    TIMER_perscalar_selct(timer_clock_id, 0);
}

void DB_config_timer2(){
    // Configure Timer2 in CTC mode for debounce interval (~5ms)
    TCCR2 |= (1<<WGM21);
    TIMSK |= (1<<OCIE2);
    OCR2 = (uint8_t) 20;
}

///////////////////// port activation ////////////////////////

// 4
void ACTIVATE_output_ports(volatile uint8_t *DDRx_Register, uint8_t *DDxn){ //E.g. DDRC, DDC0, DDC3, DDC5
    for(uint8_t i = 0; (DDxn[i] != 0 || i == 0) && i<8; i++){
        *DDRx_Register |= (1<<DDxn[i]);
    }
}
// 5
void SET_ports(volatile uint8_t *PORTx_Register, uint8_t *Pxn){ //E.g. DDRC, DDC0, DDC3, DDC5
    for(uint8_t i = 0; (Pxn[i] != 0 || i == 0) && i<8; i++){
        *PORTx_Register |= (1<<Pxn[i]);
    }
}


///////////////////// external interrupts ////////////////////////

// 6
void INT0_config_onlow() {
    sei();  
    // configuration for the interrupt  
    GICR |= (1 << INT0);        // Enable INT0

    MCUCR &= ~(1 << ISC01);     // ISC01 = 0
    MCUCR &= ~(1 << ISC00);   // ISC00 = 0 → trigger on low level
    
    DDRD &= ~(1 << PD2);        // PD2 as input
    PORTD |= (1 << PD2);        // Enable internal pull-up
} 

// 7
void INT1_config_onlow() { 
    sei(); 
    // configuration for the interrupt  
    GICR |= (1 << INT1); // external interrupt request 0 enabled (INT0, not INT1)  

    MCUCR &= ~(1 << ISC10);   // ISC10 = 1
    MCUCR &= ~(1 << ISC11); // ISC11 = 1
  
    DDRD &= ~(1 << PD3); // Set PD3 as input  
    PORTD |= (1 << PD3);  // Enable pull-up resistor on PD3
}

// void INT2_config_onlow() { 
//     sei(); 
//     // configuration for the interrupt  
//     GICR |= (1 << INT2); // external interrupt request 0 enabled (INT0, not INT1)  
//     MCUCSR &= ~(1<<ISC2);    

//     DDRB &= ~(1 << PB2); // Set PB2 as input  
//     PORTB |= (1 << PB2);  // Enable pull-up resistor on PB2
// }


///////////////////// for servo motor ////////////////////////

// 9
uint32_t SERVO_config_timer1_nc(uint8_t type_0_fast_1_phase_correct, uint16_t frequency, uint16_t prescaler){
    uint32_t TOP = 0;
    if(!type_0_fast_1_phase_correct){
        TCCR1A |= (1<<WGM11);
        TCCR1B |= (1<<WGM12) | (1<<WGM13);
        TCCR1A |= (1<<COM1A1); TCCR1A &= ~(1<<COM1A0);
        DDRD |= (1<<PD5);
    
        TIMER_perscalar_selct(1,1);
        TOP = round((F_CPU/(prescaler*frequency)) - 1);
        ICR1 = TOP;
        SERVO_MIDDLE(OCR1A);
    }

    return TOP;
}

// 10
uint32_t SERVO_config_timer1_c(){
    uint32_t TOP = 0;
    
    TCCR1A |= (1<<WGM11);
    TCCR1B |= (1<<WGM12) | (1<<WGM13);
    TCCR1A |= (1<<COM1A1); TCCR1A &= ~(1<<COM1A0);
    DDRD |= (1<<PD5);

    TIMER_perscalar_selct(1,1);
    TOP = round((F_CPU/(1*50)) - 1);
    ICR1 = TOP;
    SERVO_STOP(OCR1A);


    return TOP;
}

// sleep modes
void SLEEP_enter_power_down() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();           // Enable global interrupts
    sleep_cpu();     // Enter sleep
    sleep_disable(); // Immediately disable after waking
}



void SLEEP_enter_idle() {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
  }





















// #define cmd_ddrx DDRC
// #define cmd_port PORTC
// #define E PC1
// #define RS PC0

// #define data_ddrx DDRD
// #define data_port PORTD

// #define LCD_cmd_OM cmd_ddrx |= (1<<E) | (1<<RS)
// #define LCD_data_OM data_ddrx = 0xFF

// #define LCD_EE cmd_port |= (1<<E)
// #define LCD_DE cmd_port &= ~(1<<E)



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
