//mpu6050 i2cdevlib modified to return raw sensor data only (time, gyro gx/gy/gz, acceleration ax/ay/az)

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define INT_PIN 0 //D2
#define LED_PIN 13 //D13 (PB5)

#define PI 3.14159265359
#define RAD2DEG 180/PI
#define DEG2RAD PI/180

bool blinkState = false;

//#################### mpu6050 status ####################

MPU6050 mpu;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//#################### mpu6050 data ####################

// sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

// data
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

// orientation/motion
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];

// offset
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

// scaling
float    GYRO_FACTOR;
float    ACCEL_FACTOR;

//#################### interrupt on new data ####################

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//#################### reset ####################

void reset(void){
  Serial.println("reset in 2s");
  //WDTCSR = (1<<WDCE);
  //WDTCSR = (1<<WDE)|(1<<WDP0)|(1<<WDP1)|(1<<WDP2)|(0<<WDP3);
  //while(1);
}

//#################### calibration ####################

// Simple calibration - just average first few readings to subtract
// from the later data
void calibrate_sensors() {
  int       num_readings = 100;

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }
  
  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;
}

//#################### setup ####################

void setup() {
    
    Wire.begin();
    
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println("start up");

    //setup mpu6050
    mpu.initialize();
    if(mpu.testConnection()!=true){reset();}
    if(mpu.dmpInitialize()!=0){reset();}
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();

    // enable Arduino interrupt detection
    attachInterrupt(INT_PIN, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    //set scale range of the gyro
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000); //setFullScaleGyroRange to +-1000 degrees/sec
    //verify
    uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
    GYRO_FACTOR = 131.072/(1<<READ_FS_SEL); //2^16/(250*2^READ_FS_SEL)/2;
    
    //set scale range of the accelerometer
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8); //setFullScaleAccelRange to +-8g
    //verify
    uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
    ACCEL_FACTOR = 16384.0/(1<<READ_AFS_SEL); //2^16/(2*2^READ_AFS_SEL)/2;
    
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    // get calibration values for sensors
    calibrate_sensors();
    Serial.println("setup complete");
}

//#################### main ####################

unsigned long t_prev = 0;
unsigned long t_now = 0;
unsigned long t_dif = 0;

void loop() {
    
    // wait for MPU interrupt or extra packet(s) available
    if (mpuInterrupt) {
        
        // Keep calculating the values of the complementary filter angles for comparison with DMP here
        // Read the raw accel/gyro values from the MPU-6050
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        mpuInterrupt = false;
        
        // Get time of last raw data read
        t_now = millis();
        t_dif = t_now - t_prev;
        t_prev = t_now;
          
        // Remove offsets and scale gyro data  
        float gyro_x = (gx - base_x_gyro)/GYRO_FACTOR;
        float gyro_y = (gy - base_y_gyro)/GYRO_FACTOR;
        float gyro_z = (gz - base_z_gyro)/GYRO_FACTOR;
        //float accel_x = (ax - base_x_accel)/ACCEL_FACTOR;
        //float accel_y = (ay - base_y_accel)/ACCEL_FACTOR;
        //float accel_z = (az - base_z_accel)/ACCEL_FACTOR;
        float accel_x = (ax)/ACCEL_FACTOR;
        float accel_y = (ay)/ACCEL_FACTOR;
        float accel_z = (az)/ACCEL_FACTOR;
        
        // output data
        Serial.print(t_dif, 5);
        Serial.print(" ");
        Serial.print(gyro_x, 5);
        Serial.print(" ");
        Serial.print(gyro_y, 5);
        Serial.print(" ");
        Serial.print(gyro_z, 5);
        Serial.print(" ");
        Serial.print(accel_x, 5);
        Serial.print(" ");
        Serial.print(accel_y, 5);
        Serial.print(" ");
        Serial.print(accel_z, 5);
        Serial.println("");
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
