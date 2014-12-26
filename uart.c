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

#include "target.h"

#define UART_CALC_BAUDRATE(baudRate) ((uint32_t)(F_CPU) / ((uint32_t)(baudRate)*16) -1)

/* *********************************************************************** */

static uint8_t uart_rxbuf[UART_RXBUF_SIZE];
static uint8_t uart_txbuf[UART_TXBUF_SIZE];
static volatile uint8_t uart_rx_idx[2];
static volatile uint8_t uart_tx_idx[2];

#define UART_IDX_IN     0
#define UART_IDX_OUT    1

/* *********************************************************************** */

ISR(USART_RX_vect)
{
    uint8_t idx = uart_rx_idx[UART_IDX_IN];
    uart_rxbuf[idx++] = UDR0;
    uart_rx_idx[UART_IDX_IN] = idx % sizeof(uart_rxbuf);
} /* USART_RX_vect */


ISR(USART_UDRE_vect)
{
    /* tx buffer empty? */
    if (uart_tx_idx[UART_IDX_IN] != uart_tx_idx[UART_IDX_OUT])
    {
        /* send next byte */
        uint8_t idx = uart_tx_idx[UART_IDX_OUT];
        UDR0 = uart_txbuf[idx++];
        uart_tx_idx[UART_IDX_OUT] = idx % sizeof(uart_txbuf);
    }
    else
    {
        /* disable tx-interrupt */
        UCSR0B &= ~(1<<UDRIE0);
    }
} /* USART_UDRE_vect */


void uart_init(void)
{
    /* set baudrate */
    UBRR0H = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
    UBRR0L = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

    /* USART: rx/tx enable, 8n1 */
    UCSR0B = (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0);
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
} /* uart_init */


void uart_putc(uint8_t data)
{
    uint8_t idx = uart_tx_idx[UART_IDX_IN];
    uart_txbuf[idx++] = data;
    uart_tx_idx[UART_IDX_IN] = idx % sizeof(uart_txbuf);

    /* enable tx-interrupt */
    UCSR0B |= (1<<UDRIE0);
} /* uart_putc */


uint8_t uart_getc(void)
{
    while (uart_rx_idx[UART_IDX_IN] == uart_rx_idx[UART_IDX_OUT]);

    /* send next byte */
    uint8_t idx = uart_rx_idx[UART_IDX_OUT];
    uint8_t result = uart_rxbuf[idx++];
    uart_rx_idx[UART_IDX_OUT] = idx % sizeof(uart_rxbuf);

    return result;
} /* uart_getc */


uint8_t uart_rx_count(void)
{
    return (uart_rx_idx[UART_IDX_IN] - uart_rx_idx[UART_IDX_OUT]) % sizeof(uart_rxbuf);
} /* uart_check */


void uart_putstr(const char *str)
{
    while (*str)
    {
        uart_putc((uint8_t)(*str++));
    }
} /* uart_putstr */


static uint8_t bin2hex(uint8_t data)
{
    return data + ((data < 0x0A) ? '0' : ('A' - 0x0A));
} /* bin2hex */


static uint8_t hex2bin(uint8_t data)
{
    if (data >= '0' && data <= '9')
    {
        return (data - '0');
    }
    else
    {
        data &= ~(0x20);
        if (data >= 'A' && data <= 'F')
        {
            return (data - 'A' + 0x0A);
        }
    }

    return 0x00;
} /* hex2bin */


void uart_put_hex(uint8_t data)
{
    uart_putc(bin2hex(data >> 4));
    uart_putc(bin2hex(data & 0x0F));
} /* uart_hex */


uint8_t uart_get_hex(void)
{
    uint8_t result;

    result = hex2bin(uart_getc()) << 4;
    result |= hex2bin(uart_getc());

    return result;
} /* uart_hex */
