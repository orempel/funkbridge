/***************************************************************************
 *   Copyright (C) 11/2014 by Olaf Rempel                                  *
 *   razzor@kopf-tisch.de                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; version 2 of the License,               *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>

#include <string.h>

#include "target.h"
#include "rfm12.h"
#include "uart.h"

/* *********************************************************************** */

#define BRIDGE_CMD_IDLE                 0x00
#define BRIDGE_CMD_TRANSMIT             'T'
#define BRIDGE_CMD_RECEIVE              'R'
#define BRIDGE_CMD_VERSION              'V'

#define BRIDGE_CAUSE_SUCCESS            0x00
#define BRIDGE_CAUSE_TIMEOUT            0x01
#define BRIDGE_CAUSE_NOT_SUPPORTED      0xF0
#define BRIDGE_CAUSE_INVALID_PARAMETER  0xF1
#define BRIDGE_CAUSE_UNSPECIFIED_ERROR  0xFF

/* *********************************************************************** */

static uint8_t led_timer;
volatile static uint8_t clock_tick;

ISR(TIMER0_OVF_vect)
{
    TCNT0 = TIMER_RELOAD;

    clock_tick = 1;
} /* TIMER0_OVF_vect */


/*
 * For newer devices the watchdog timer remains active even after a
 * system reset. So disable it as soon as possible.
 * automagically called on startup
 */
void disable_wdt_timer(void) __attribute__((naked, section(".init3")));
void disable_wdt_timer(void)
{
    MCUSR = 0;
    WDTCSR = (1<<WDCE) | (1<<WDE);
    WDTCSR = (0<<WDE);
} /* disable_wdt_timer */


int main(void) __attribute__ ((noreturn));
int main(void)
{
    /* init LEDs */
    LED_INIT();

    /* timer0, FCPU/64, overflow interrupt */
    TCCR0B = (1<<CS01) | (1<<CS00);
    TIMSK0 = (1<<TOIE0);

    uart_init();
    rfm12_init(RFM12_ADDRESS);

    sei();

    uint8_t command = BRIDGE_CMD_IDLE;
    uint8_t timeout = 0;
    uint8_t datalen = 0;
    uint8_t bcnt = 0;


    while (1)
    {
        if (clock_tick)
        {
            clock_tick = 0;

            if (led_timer > 0)
            {
                LED_ON();
                led_timer--;
            }
            else
            {
                LED_OFF();
            }

            /* do periodic work (wait for 5 ticks silence before start TX) */
            rfm12_tick(5);

            if (timeout != 0)
            {
                timeout--;
            }
        }

        uint8_t rx_count = uart_rx_count();

        if ((command == BRIDGE_CMD_IDLE) && (rx_count != 0))
        {
            command = uart_getc();
            datalen = 0;
            timeout = 0;
            bcnt = 1;

            switch (command)
            {
                case 'R':
                    timeout = 200;
                case 'T':
                case 'V':
                    break;

                default:
                    uart_putc(command);
                    uart_putc(BRIDGE_CAUSE_NOT_SUPPORTED);
                    uart_putc(0x00);

                    command = BRIDGE_CMD_IDLE;
                    break;
            }
        }
        else if ((bcnt == 1) && (rx_count != 0))
        {
            datalen = uart_getc();
            bcnt++;
        }
        else
        {
            if ((command == BRIDGE_CMD_TRANSMIT) & (bcnt == 2) && (rx_count == datalen))
            {
                struct rfm12_packet * pkt = rfm12_get_txpkt();
                if (pkt == (void *)0)
                {
                    uart_putc(command);
                    uart_putc(BRIDGE_CAUSE_UNSPECIFIED_ERROR);
                    uart_putc(0x00);
                }
                else
                {
                    pkt->dest_address       = uart_getc();
                    pkt->source_address     = uart_getc();
                    pkt->data_length        = uart_getc();
                    pkt->header_checksum    = uart_getc();
                    datalen -= 4;

                    uint8_t i = 0;
                    while (datalen--)
                    {
                        pkt->data[i++] = uart_getc();
                    }

                    rfm12_start_tx();
                    led_timer = 5;

                    uart_putc(command);
                    uart_putc(BRIDGE_CAUSE_SUCCESS);
                    uart_putc(0x00);
                }

                command = BRIDGE_CMD_IDLE;
            }
            else if ((command == BRIDGE_CMD_RECEIVE) && (bcnt == 2))
            {
                if ((datalen != 0) && (rx_count != 0))
                {
                    datalen--;
                    uart_getc();
                }
                else if (timeout == 0)
                {
                    uart_putc(command);
                    uart_putc(BRIDGE_CAUSE_TIMEOUT);
                    uart_putc(0);

                    command = BRIDGE_CMD_IDLE;
                }
                else
                {
                    struct rfm12_packet *pkt = rfm12_get_rxpkt();
                    if (pkt)
                    {
                        uint8_t i;

                        led_timer = 5;

                        uart_putc(command);
                        uart_putc(BRIDGE_CAUSE_SUCCESS);
                        uart_putc(pkt->data_length + 4);

                        uart_putc(pkt->dest_address);
                        uart_putc(pkt->source_address);
                        uart_putc(pkt->data_length);
                        uart_putc(pkt->header_checksum);

                        for (i = 0; i < pkt->data_length; i++)
                        {
                            uart_putc(pkt->data[i]);
                        }

                        command = BRIDGE_CMD_IDLE;
                    }
                }
            }
            else if ((command == BRIDGE_CMD_VERSION) && (bcnt == 2))
            {
                if ((datalen != 0) && (rx_count != 0))
                {
                    datalen--;
                    uart_getc();
                }
                else
                {
                    uart_putc(command);
                    uart_putc(BRIDGE_CAUSE_SUCCESS);
                    uart_putc(16);
                    uart_putstr("funkbridge v0.99");

                    command = BRIDGE_CMD_IDLE;
                }
            }
        }
    }
} /* main */
