/*
GY-521 / MPU5060
3-axis gyroscope, 3-axis accelerometer, temperature sensor, digital motion processor (DMP)
#libraries:
Wire.h (I2C Communication)
Pins:
 * GND = GND
 * VCC = 3.3V
 * SCK/SCL = A5
 * SDA = A4
 * AD0 = high (pullup already in place!)
others:
 * INT = Interrupt
 * XDA & XCL for I2C as Master
I2C Adress: AD0=high (0x68) / AD0=high (0x69)
*/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "TWI.h"
#include "TWI.c"
#include "uart.h"
#include "uart.c"

volatile byte state = false;
const int MPU_ADDR = 0x68<<1; //MPU I2C Address: AD0=high (0x68) / AD0=high (0x69)
int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;
#define temp_scale 1/340.00
#define temp_offset 36.53


//interrupt 0: read and print data
ISR( INT0_vect ){
  state = true;
  PORTB ^= (1<<PB5); //toggle LED
}

void mpu_read_and_print(void){
  TWIStart();
  TWIWrite(MPU_ADDR);
  TWIWrite(0x3B);
  
  TWIStart();
  TWIWrite(MPU_ADDR|1);
  
  //read data & combine two bytes to one data bytes
  accelerometer_x = TWIReadACK() | TWIReadACK()<<8; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = TWIReadACK() | TWIReadACK()<<8; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = TWIReadACK() | TWIReadACK()<<8; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = TWIReadACK() | TWIReadACK()<<8; // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = TWIReadACK() | TWIReadACK()<<8; // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = TWIReadACK() | TWIReadACK()<<8; // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = TWIReadACK() | TWIReadNACK()<<8; // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  TWIStop();
  
  // print out data
  // itoa,utoa,ltoa,ultoa,dtostre,dtostrf
  uart_send_string( utoa( accelerometer_x, send_buffer, 10 ) ); //send uint16_t
  uart_send_bit(' ');
  uart_send_string( utoa( accelerometer_y, send_buffer, 10 ) ); //send uint16_t
  uart_send_bit(' ');
  uart_send_string( utoa( accelerometer_z, send_buffer, 10 ) ); //send uint16_t
  uart_send_bit(' ');
  uart_send_string( utoa( gyro_x, send_buffer, 10 ) ); //send uint16_t
  uart_send_bit(' ');
  uart_send_string( utoa( gyro_y, send_buffer, 10 ) ); //send uint16_t
  uart_send_bit(' ');
  uart_send_string( utoa( gyro_z, send_buffer, 10 ) ); //send uint16_t
  uart_send_bit(' ');
  uart_send_string( dtostrf( temperature*temp_scale + temp_offset, send_buffer, 2, "ascii" ) ); //send uint16_t
  uart_send_bit('\n');
}

void startup(void) {
  //Serial
  uartINIT();
  
  //interrupt INT0
  DDRD &= ~(1<<DDD2); //enable PD2 as input
  PORTD |= (1<<PD2); // set pull-up
  EIMSK |= 1<<INT0; //enable INT0
  EICRA |= (1<<ISC01)|(0<<ISC00); // interrupt on falling edge
  sei();
  
  //enable LED PB5/D13
  DDRB |= DDB5; //enable PB5 as output
  PORTB &= ~(1<<PB5); //turn LED off
  //PORTB |= (1<<PB5); //turn LED on
  //PORTB ^= (1<<PB5); //toggle LED
  
  //I2C
  TWIInit();
  
  //I2C start sensor
  TWIStart();
  TWIWrite(MPU_ADDR);
  TWIWrite(0x6B);
  TWIStop();
}

int main(void) {
  startup();
  while(1){
    //_delay_ms(1);
    if (state == true){
        uart_send_string( "triggered\n" );
        mpu_read_and_print();
        state = false;
    }
  }
}
