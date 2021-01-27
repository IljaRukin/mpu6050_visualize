
#include <avr/io.h>
#include "uart.h"
//#define BAUD 38400UL
#define BAUD 115200UL
#define MYUBRR F_CPU/8/(BAUD-1)

char send_buffer[10];

void uartINIT(void)
{
  UBRR0H = (uint8_t) (MYUBRR>>8);
  UBRR0L = (uint8_t) (MYUBRR);
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  //UCSR0C = (1<<USBS0)|(3<<UCSZ00); //8 bit data, 2 stop bit
  UCSR0C = (1<<USBS0)|(3 << UCSZ00); //8 bit data, 1 stop bit
  UCSR0A = (1<<U2X0); //2x speed
}

void uart_send_bit(unsigned char c){
  while ( !( UCSR0A & (1<<UDRE0)) );
  UDR0 = c;
}

void uart_send_string (char *s){
  while (*s)
  {
    uart_send_bit(*s);
    s++;
  }
}

void uart_test(void){
    uart_send_bit('x');
    uart_send_bit('\n');
    uart_send_string( "myString\n" );
    uart_send_string( utoa( (uint16_t)65535, send_buffer, 10 ) );
    uart_send_bit('\n');
    uart_send_string( dtostrf( (float)39.10, send_buffer, 2, "ascii" ) ); //send uint16_t
    uart_send_bit('\n');
    _delay_ms(100);
}
