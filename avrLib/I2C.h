#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <util/delay.h>


// TWINT with ack and without ack
#define TWI_SET_TWINT_ACK TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA) // set TWINT and TWEA
#define TWI_SET_TWINT TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) // set TWINT

// TWI start, stop and stop/start
#define TWI_START TWCR = (1<<TWINT) | (1<<TWEN) | (1<< TWSTA)| (1<<TWIE)| (1<<TWEA); TWI_SET_TWINT // Send start condition
#define TWI_STOP TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) // Transmit stop condition
#define TWI_STOP_START TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) |(1<<TWSTA)

// Read and send data
#define TWI_readData_ack(buffer) buffer = TWDR; TWI_SET_TWINT_ACK // read data
#define TWI_readData_nack(buffer) buffer = TWDR; TWI_SET_TWINT // read data nack
#define TWI_sendData_ML(data,transmitt_count) TWDR = data; --(*transmitt_count); TWI_SET_TWINT// send data
#define TWI_sendData_ack(data) TWDR = data; TWI_SET_TWINT_ACK// send data
#define TWI_sendData_nack(data) TWDR = data; TWI_SET_TWINT// send data nack

// Status code
#define STATUS_CODE (TWSR & 0xF8)

// TWI select bit rate prescaler
#define TWI_BIT_RATE_PRESCALER_1 TWSR &= ~((1<<TWPS1)|(1<<TWPS0)) 
#define TWI_BIT_RATE_PRESCALER_8 TWSR = (1<<TWPS0)
#define TWI_BIT_RATE_PRESCALER_16 TWSR = (1<<TWPS1)
#define TWI_BIT_RATE_PRESCALER_64 TWSR = (1<<TWPS1) | (1<<TWPS0)

// Set slave address
#define SET_SLAVE_ADRESS_7BIT(this_slave_addr_7bit) TWAR |= (this_slave_addr_7bit << 1)

// TWI send slave addr and mode
#define TWI_SLA_W(slave_addr_7bit) TWDR = (slave_addr_7bit << 1); TWI_SET_TWINT
#define TWI_SLA_R(slave_addr_7bit) TWDR = (slave_addr_7bit << 1) | 1; TWI_SET_TWINT

// Set pull up resistor on SDA and SCL pins
#define SET_PULL_UP_RESISTOR_ON_SDA_SCL DDRC &= ~((1<<PC1)|(1<<PC0)); PORTC |= (1<<PC1) | (1<<PC0)



void TWI_send_start(){
    TWI_START;
}

void TWI_send_sla_w_or_r(uint8_t mode, uint8_t slave_addr){
    if(mode == 'w'){
        TWI_SLA_W(slave_addr);
    }
    else if('r')
    {
        TWI_SLA_R(slave_addr);
    }
}

void TWI_send_data(uint8_t data, uint8_t last_1){
    if(last_1)
    {
        TWI_sendData_nack(data);
    }
    else if(!last_1)
    {
        TWI_sendData_ack(data);
    }
}

uint8_t TWI_recive_data(uint8_t ack_1){
    uint8_t recived_data = 0;
    if(ack_1)
    {
        TWI_readData_ack(recived_data);
    }
    else if(!ack_1)
    {
        TWI_readData_nack(recived_data);
    }

    return recived_data;
}

void TWI_send_stop(){
    TWI_STOP;
}

void TWI_return_to_not_addressed_slave(){
    TWI_SET_TWINT_ACK;
}



// main function
uint8_t reciveData_REQUESTED_AND_THEN_CLOSE_CONNECTION_PR_11_STATUS_CODE(uint8_t dest_slave_addr_7bit, uint8_t requested_value){
    uint8_t recived_data = 0;
    static uint8_t send_error_count = 0;

    switch (STATUS_CODE)
    {
        case 0x08:
            TWI_send_sla_w_or_r('w',dest_slave_addr_7bit);
            break;
            
        case 0x10:
            TWI_send_sla_w_or_r('r',dest_slave_addr_7bit);
            break;

        case 0x18:
            TWI_send_data(requested_value,1);
            break;
        
        case 0x20:
            if(send_error_count < 2) TWI_send_start();
            else if (send_error_count >= 2){
                send_error_count = 0;
                TWI_send_stop();
            }
            send_error_count++;
            break;

        case 0x28:
            send_error_count = 0;
            TWI_send_start();
            break;

        case 0x30: 
            send_error_count = 0;
            TWI_send_start();
            break;
        
        case 0x38:
            TWI_SET_TWINT;
            break;

        case 0x40:
            TWI_SET_TWINT;
            break;

        case 0x48:
            TWI_START;
            break;

        case 0x50: // won't get inn herer
            recived_data = TWI_recive_data(1); 
            TWI_SET_TWINT;
            break;

        case 0x58:
            recived_data = TWI_recive_data(1);
            TWI_send_stop();
            break;
        default:
            break;
    }

    return recived_data;
}















uint8_t reciveData_AND_THEN_CLOSE_CONNECTION(uint8_t dest_slave_addr_7bit){
    uint8_t recivedData = 0;
    switch (STATUS_CODE)
    {
        case 0x08:
            // PORTA ^= (1<<PB0); // kunn for debuging ikke nødvendign
            TWI_SLA_R(dest_slave_addr_7bit);
            TWI_SET_TWINT_ACK;
            break;
        case 0x10:
            // PORTA ^= (1<<PB1); // kunn for debuging ikke nødvendign
            TWI_SET_TWINT_ACK;
            break;
        case 0x38:
            // PORTA ^= (1<<PB2); // kunn for debuging ikke nødvendign
            TWI_SET_TWINT_ACK;
            break;
        case 0x40:
            // PORTA ^= (1<<PB3); // kunn for debuging ikke nødvendign
            TWI_SET_TWINT;
            break;
        case 0x48:
            // PORTA ^= (1<<PB4); // kunn for debuging ikke nødvendign
            TWI_START;
            break;
        case 0x50: 
            recivedData = TWDR; 
            TWI_SET_TWINT;
            break;
        case 0x58:
            recivedData = TWDR;
            // PORTA ^= (1<<PB6); // kunn for debuging ikke nødvendign
            TWI_STOP;
            break;
        default:
            break;
    }

    return recivedData;
}
