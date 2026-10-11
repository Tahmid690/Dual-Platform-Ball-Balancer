#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(void);
void uart_tx(char c);
void uart_print(const char *s);
void uart_println(const char *s);
uint8_t uart_available(void);
char uart_rx(void);

#endif // UART_H
