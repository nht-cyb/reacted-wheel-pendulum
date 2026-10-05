// Read the X-axis accelerometer of an MPU6050 over I2C and smooth it with a
// 1-D Kalman filter. Raw and filtered values are printed as "raw,filtered"
// so they can be compared in the Arduino Serial Plotter.
//
// Needs the SimpleKalmanFilter library and the I2C.h helper
// (i2cWrite/i2cWrite2/i2cRead) from TKJ Electronics' KalmanFilter MPU6050
// example, placed next to this sketch.

#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include "I2C.h"
#define RESTRICT_PITCH
// SimpleKalmanFilter(measurement uncertainty, estimation uncertainty, process noise)
SimpleKalmanFilter simpleKalmanFilter(1, 1, 0.001);

uint8_t i2cData[14]; // Buffer for I2C data

int16_t accX;        // raw X acceleration (LSB, 2048 LSB/g at +-16g)
float accX_kalman;   // filtered X acceleration

void setup()
{
    Serial.begin(9600);
    Wire.begin();
#if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

    // Configure registers 0x19..0x1C (SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACCEL_CONFIG)
    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x03; // Set Accelerometer Full Scale Range to ±16g
    while (i2cWrite(0x19, i2cData, 4, false))
        ; // Write to all four registers at once
    while (i2cWrite2(0x6B, 0x01, true))
        ; // PLL with X axis gyroscope reference and disable sleep mode
}
void loop()
{
    // Read 14 bytes from 0x3B: accel XYZ, temperature, gyro XYZ (big-endian)
    while (i2cRead(0x3B, i2cData, 14)) {
        ;
    }
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accX_kalman = simpleKalmanFilter.updateEstimate((float)accX);
    Serial.print(accX);
    Serial.print(",");
    Serial.print(accX_kalman, 3);
    Serial.println();
}
