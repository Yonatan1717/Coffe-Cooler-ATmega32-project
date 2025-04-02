#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>

void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_init(void);

int main(void) {
    DDRA = 0xFF;                        // LCD D0–D7 on PA0–PA7
    DDRB |= (1 << PB0) | (1 << PB2);    // PB0 = RS, PB2 = E

    lcd_init();                         // Initialize LCD
    lcd_data('H');                      // Display 'H'
    lcd_data('i');                      // Display 'i'

    while (1);
}

void lcd_command(uint8_t cmd) {
    PORTA = cmd;
    PORTB &= ~(1 << PB0);              // RS = 0 (command)
    PORTB |= (1 << PB2);               // E = 1
    _delay_us(10);
    PORTB &= ~(1 << PB2);              // E = 0
    _delay_ms(2);                      // Allow time for command
}

void lcd_data(uint8_t data) {
    PORTA = data;
    PORTB |= (1 << PB0);               // RS = 1 (data)
    PORTB |= (1 << PB2);               // E = 1
    _delay_us(10);
    PORTB &= ~(1 << PB2);              // E = 0
    _delay_ms(2);                      // Allow time for write
}

void lcd_init(void) {
    _delay_ms(20);                      // Wait >15ms after power-on

    lcd_command(0x38);                 // Function set: 8-bit, 2 lines, 5x8 dots
    lcd_command(0x0C);                 // Display ON, cursor OFF
    lcd_command(0x01);                 // Clear display
    _delay_ms(2);                      // Required delay after clear
    lcd_command(0x06);                 // Entry mode: cursor right, no shift
}
