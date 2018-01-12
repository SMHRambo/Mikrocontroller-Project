#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/io.h>

/* PORTA */
#define PA7     7
#define PA6     6
#define PA5     5
#define PA4     4
#define PA3     3
#define PA2     2
#define PA1     1
#define PA0     0

/* DDRA */
#define DDA7    7
#define DDA6    6
#define DDA5    5
#define DDA4    4
#define DDA3    3
#define DDA2    2
#define DDA1    1
#define DDA0    0

/* PINA */
#define PINA7   7
#define PINA6   6
#define PINA5   5
#define PINA4   4
#define PINA3   3
#define PINA2   2
#define PINA1   1
#define PINA0   0

/* PORTB */
#define PB7     7
#define PB6     6
#define PB5     5
#define PB4     4
#define PB3     3
#define PB2     2
#define PB1     1
#define PB0     0

/* DDRB */
#define DDB7    7
#define DDB6    6
#define DDB5    5
#define DDB4    4
#define DDB3    3
#define DDB2    2
#define DDB1    1
#define DDB0    0

/* PINB */
#define PINB7   7
#define PINB6   6
#define PINB5   5
#define PINB4   4
#define PINB3   3
#define PINB2   2
#define PINB1   1
#define PINB0   0

/* PORTC */
#define PC7     7
#define PC6     6
#define PC5     5
#define PC4     4
#define PC3     3
#define PC2     2
#define PC1     1
#define PC0     0

/* DDRC */
#define DDC7    7
#define DDC6    6
#define DDC5    5
#define DDC4    4
#define DDC3    3
#define DDC2    2
#define DDC1    1
#define DDC0    0

/* PINC */
#define PINC7   7
#define PINC6   6
#define PINC5   5
#define PINC4   4
#define PINC3   3
#define PINC2   2
#define PINC1   1
#define PINC0   0

/* PORTD */
#define PD7     7
#define PD6     6
#define PD5     5
#define PD4     4
#define PD3     3
#define PD2     2
#define PD1     1
#define PD0     0

/* DDRD */
#define DDD7    7
#define DDD6    6
#define DDD5    5
#define DDD4    4
#define DDD3    3
#define DDD2    2
#define DDD1    1
#define DDD0    0

/* PIND */
#define PIND7   7
#define PIND6   6
#define PIND5   5
#define PIND4   4
#define PIND3   3
#define PIND2   2
#define PIND1   1
#define PIND0   0

uint8_t PORTA;
uint8_t PORTB;
uint8_t PORTC;
uint8_t PORTD;

uint16_t Timer;


uint16_t iChannel[31];

uint16_t pwm_Time = 0;

int main() {

    for(uint8_t i = 0; i < 30; i++)
    {
        iChannel[i] = 1500; //550(350) - 2450(2250)
    }

    iChannel[30] = 20000;

    DDRA |= (1 << PA0);
    DDRA |= (1 << PA1);
    DDRA |= (1 << PA2);
    DDRA |= (1 << PA3);
    DDRA |= (1 << PA4);
    DDRA |= (1 << PA5);
    DDRA |= (1 << PA6);
    DDRA |= (1 << PA7);
    DDRA |= (1 << PA0);
    DDRA |= (1 << PA1);
    DDRA |= (1 << PA2);
    DDRA |= (1 << PA3);
    DDRA |= (1 << PA4);
    DDRA |= (1 << PA5);
    DDRA |= (1 << PA6);
    DDRA |= (1 << PA7);
    DDRA |= (1 << PA0);
    DDRA |= (1 << PA1);
    DDRA |= (1 << PA2);
    DDRA |= (1 << PA3);
    DDRA |= (1 << PA4);
    DDRA |= (1 << PA5);
    DDRA |= (1 << PA6);
    DDRA |= (1 << PA7);
    DDRA |= (1 << PA0);
    DDRA |= (1 << PA1);
    DDRA |= (1 << PA2);
    DDRA |= (1 << PA3);
    DDRA |= (1 << PA4);
    DDRA |= (1 << PA5);
    DDRA |= (1 << PA6);
    DDRA |= (1 << PA7);

    while(true) {
        pwm_Time = Timer;

        if(pwm_Time > iChannel[30])
        {
            PORTA |= (1 << PA0);
            PORTA |= (1 << PA1);
            PORTA |= (1 << PA2);
            PORTA |= (1 << PA3);
            PORTA |= (1 << PA4);
            PORTA |= (1 << PA5);
            PORTA |= (1 << PA6);
            PORTA |= (1 << PA7);
            PORTB |= (1 << PB0);
            PORTB |= (1 << PB1);
            PORTB |= (1 << PB2);
            PORTB |= (1 << PB3);
            PORTB |= (1 << PB4);
            PORTB |= (1 << PB5);
            PORTB |= (1 << PB6);
            PORTB |= (1 << PB7);
            PORTC |= (1 << PC0);
            PORTC |= (1 << PC1);
            PORTC |= (1 << PC2);
            PORTC |= (1 << PC3);
            PORTC |= (1 << PC4);
            PORTC |= (1 << PC5);
            PORTC |= (1 << PC6);
            PORTC |= (1 << PC7);
            PORTD |= (1 << PD0);
            PORTD |= (1 << PD1);
            PORTD |= (1 << PD2);
            PORTD |= (1 << PD3);
            PORTD |= (1 << PD4);
            PORTD |= (1 << PD5);
            PORTD |= (1 << PD6);
            PORTD |= (1 << PD7);
            Timer = 0;
            pwm_Time = 0;
        }

        if(pwm_Time > iChannel[0]) PORTA &= ~(1 << PA0);
        if(pwm_Time > iChannel[1]) PORTA &= ~(1 << PA1);
        if(pwm_Time > iChannel[2]) PORTA &= ~(1 << PA2);
        if(pwm_Time > iChannel[3]) PORTA &= ~(1 << PA3);
        if(pwm_Time > iChannel[4]) PORTA &= ~(1 << PA4);
        if(pwm_Time > iChannel[5]) PORTA &= ~(1 << PA5);
        if(pwm_Time > iChannel[6]) PORTA &= ~(1 << PA6);
        if(pwm_Time > iChannel[7]) PORTA &= ~(1 << PA7);
        if(pwm_Time > iChannel[8]) PORTB &= ~(1 << PB0);
        if(pwm_Time > iChannel[9]) PORTB &= ~(1 << PB1);
        if(pwm_Time > iChannel[10]) PORTB &= ~(1 << PB2);
        if(pwm_Time > iChannel[11]) PORTB &= ~(1 << PB3);
        if(pwm_Time > iChannel[12]) PORTB &= ~(1 << PB4);
        if(pwm_Time > iChannel[13]) PORTB &= ~(1 << PB5);
        if(pwm_Time > iChannel[14]) PORTB &= ~(1 << PB6);
        if(pwm_Time > iChannel[15]) PORTB &= ~(1 << PB7);
        if(pwm_Time > iChannel[16]) PORTC &= ~(1 << PC2);
        if(pwm_Time > iChannel[17]) PORTC &= ~(1 << PC3);
        if(pwm_Time > iChannel[18]) PORTC &= ~(1 << PC4);
        if(pwm_Time > iChannel[19]) PORTC &= ~(1 << PC5);
        if(pwm_Time > iChannel[20]) PORTC &= ~(1 << PC6);
        if(pwm_Time > iChannel[21]) PORTC &= ~(1 << PC7);
        if(pwm_Time > iChannel[22]) PORTD &= ~(1 << PD0);
        if(pwm_Time > iChannel[23]) PORTD &= ~(1 << PD1);
        if(pwm_Time > iChannel[24]) PORTD &= ~(1 << PD2);
        if(pwm_Time > iChannel[25]) PORTD &= ~(1 << PD3);
        if(pwm_Time > iChannel[26]) PORTD &= ~(1 << PD4);
        if(pwm_Time > iChannel[27]) PORTD &= ~(1 << PD5);
        if(pwm_Time > iChannel[28]) PORTD &= ~(1 << PD6);
        if(pwm_Time > iChannel[29]) PORTD &= ~(1 << PD7);
    }

    return 0;
}

void init_twi_slave(uint8_t adr)
{
    TWAR= adr; //Adresse setzen
	TWCR &= ~(1<<TWSTA)|(1<<TWSTO);
	TWCR|= (1<<TWEA) | (1<<TWEN)|(1<<TWIE); 	
	buffer_adr=0xFF;  
	sei();
}

//ACK nach empfangenen Daten senden/ ACK nach gesendeten Daten erwarten
#define TWCR_ACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);  

//NACK nach empfangenen Daten senden/ NACK nach gesendeten Daten erwarten     
#define TWCR_NACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);

//switch to the non adressed slave mode...
#define TWCR_RESET TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC);  

ISR (TWI_vect) {
    uint8_t data=0;

    switch (TW_STATUS) {

        case TW_SR_SLA_ACK:
            buffer_adr = 0xFF;
            TWCR_ACK;
            break;
        
        case TW_SR_DATA_ACK:
            data = TWDR;
            if (buffer_adr == 0xFF) {
                if(data < 63) {
                    buffer_adr = data;
                } else {
                    buffer_adr = 0;
                }				
                TWCR_ACK;
            } else {
                if(buffer_adr < 63) {
                    iChannel[buffer_adr++] = data;
                }
                TWCR_ACK;	
            }
            break;

        case TW_ST_SLA_ACK:
        case TW_ST_DATA_ACK: //0xB8 Slave Transmitter, Daten wurden angefordert
            if (buffer_adr == 0xFF) {
                buffer_adr = 0;
            }	
                
            if(buffer_adr < 63) {
                TWDR = iChannel[buffer_adr];
                buffer_adr++;
            } else {
                TWDR = 0;
            }
            TWCR_ACK;
            break;

        case TW_SR_STOP:
            TWCR_ACK;
            break;

        case TW_ST_DATA_NACK:
        case TW_SR_DATA_NACK:
        case TW_ST_LAST_DATA:
        default: 	
            TWCR_RESET;
            break;
    }
}