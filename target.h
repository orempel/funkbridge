#ifndef __TARGET_H__
#define __TARGET_H__

/* *********************************************************************** */
/*
 * using ATmega328p @16MHz:
 * Fuse E: 0x02 (512 words bootloader)
 * Fuse H: 0xDC (2.7V BOD)
 * Fuse L: 0xFF (external crystal)
 */
#define F_CPU               16000000
#define BAUDRATE            38400
#define RFM12_ADDRESS       0x01
#define TIMEOUT             1000

/* 1ms @16MHz */
#define TIMER_RELOAD        (0xFF - 250)

#define LED_INIT()          DDRD |= (1<<PORTD4)
#define LED_ON()            PORTD |= (1<<PORTD4)
#define LED_OFF()           PORTD &= ~(1<<PORTD4)
#define LED_TOGGLE()        PORTD ^= (1<<PORTD4)

#define UART_RXBUF_SIZE     128
#define UART_TXBUF_SIZE     128

#define RFM12_INT_INIT()    EICRA |= (1<<ISC01)
#define RFM12_INT_ON()      EIMSK |= (1<<INT0)
#define RFM12_INT_OFF()     EIMSK &= ~(1<<INT0)
#define RFM12_INT_CLEAR()   EIFR = INTF0
#define RFM12_INT_VECT      INT0_vect

#define RFM12_CS_INIT()     DDRD |= (1<<PORTD7)
#define RFM12_CS_ACTIVE()   PORTD &= ~(1<<PORTD7)
#define RFM12_CS_INACTIVE() PORTD |= (1<<PORTD7)

#define RFM12_SPI_INIT()    {   /* SS, SCK and MOSI are outputs */ \
                                DDRB |= (1<<PORTB2) | (1<<PORTB3) | (1<<PORTB5); \
                                /* SPI Master, F_CPU /16 */ \
                                SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0); \
                            }

/* *********************************************************************** */

#endif /* __TARGET_H__ */
