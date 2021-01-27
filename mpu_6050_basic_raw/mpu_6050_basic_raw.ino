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

const byte interruptPin = 2;
volatile byte state = LOW;
#include "Wire.h" //I2C library

const int MPU_ADDR = 0x68; //MPU I2C Address: AD0=high (0x68) / AD0=high (0x69)
int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;

void stateHIGH(void){
  state = HIGH;
}

void mpu_read_and_print(void){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting register 0x3B of MPU-6050
  Wire.endTransmission(false); // restart & keep connection alive
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  //read data & combine two bytes to one data 
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // print out data
  Serial.print(accelerometer_x); Serial.print(' ');
  Serial.print(accelerometer_y); Serial.print(' ');
  Serial.print(accelerometer_z); Serial.print(' ');
  Serial.print(gyro_x); Serial.print(' ');
  Serial.print(gyro_y); Serial.print(' ');
  Serial.print(gyro_z); Serial.print(' ');
  Serial.print(temperature/340.00+36.53); Serial.print(' ');
  Serial.println();
}

void setup() {
  //UART
  Serial.begin(115200);
  Serial.println("Start Serial Connection");
  
  //interrupt
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), stateHIGH, HIGH);
  
  //I2C
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // wake up MPU-6050
  Wire.endTransmission(true);
}

void loop() {
  if (state == HIGH){
    mpu_read_and_print();
    state = LOW;
  }
}
