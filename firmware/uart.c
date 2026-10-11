#ifndef F_CPU
#define F_CPU 8000000UL
#endif
#define BAUD 9600UL

// Double speed mode (U2X=1) for lower error margin & RC drift tolerance
#define BRC  ((F_CPU / (8UL * BAUD)) - 1)

#include <avr/io.h>
#include "uart.h"

void uart_init(void) {
    // Set double speed mode for UART
    UCSRA = (1 << U2X);

    // Set baud rate registers (URSEL = 0 for UBRRH)
    UBRRH = (uint8_t)(BRC >> 8);
    UBRRL = (uint8_t)BRC;

    // Enable Transmitter & Receiver
    UCSRB = (1 << TXEN) | (1 << RXEN);

    // Set frame format: 8 data bits, 1 stop bit (URSEL = 1 for UCSRC)
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void uart_tx(char c) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = c;
}

void uart_print(const char *s) {
    while (*s) uart_tx(*s++);
}

void uart_println(const char *s) {
    uart_print(s);
    uart_print("\r\n");
}

uint8_t uart_available(void) {
    return (UCSRA & (1 << RXC)) ? 1 : 0;
}

char uart_rx(void) {
    while (!(UCSRA & (1 << RXC)));
    return UDR;
}
