#ifndef __UART_H__
#define __UART_H__

/* *********************************************************************** */

void    uart_init       (void);

void    uart_putc       (uint8_t data);
uint8_t uart_getc       (void);
uint8_t uart_rx_count   (void);
void    uart_putstr     (const char *str);

void    uart_put_hex    (uint8_t data);
uint8_t uart_get_hex    (void);

/* *********************************************************************** */

#endif /* __UART_H__ */
