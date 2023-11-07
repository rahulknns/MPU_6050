#include <Arduino.h>
#include <Wire.h>
#include <mpu6050.hpp>

MPU6050* mpu6050;
void setup(){
    Wire.begin();
    Serial.begin(9600);  
    mpu6050 = new MPU6050(0x68, &Wire);
    mpu6050->setAccelerometerRange(ACCELERO_METER_RANGE_2);
    mpu6050->setGyroscopeRange(GYROSCOPE_RANGE_250);
    mpu6050->setSampleRateDivider(0);
    mpu6050->setLowpassCutOffFrequency(0);
    mpu6050->disableSleepMode();
}

void loop(){
    double ax, ay, az, gx, gy, gz;
    mpu6050->getSensorsReadings(ax, ay, az, gx, gy, gz);
    Serial.print("ax: ");
    Serial.print(ax);
    Serial.print(" ay: ");
    Serial.print(ay);
    Serial.print(" az: ");
    Serial.print(az);
    Serial.print(" gx: ");
    Serial.print(gx);
    Serial.print(" gy: ");
    Serial.print(gy);
    Serial.print(" gz: ");
    Serial.println(gz);
    delay(100);
}