#ifndef MPU6050_HPP_
#define MPU6050_HPP_
#include <Arduino.h>
#include <Wire.h>
#include "I2C_driver/src/I2C_device.hpp"
#include "mpu6050_registers.hpp"
#define DEFAULT_ADDRESS 0x68 
#define ACCELERO_METER_SENSITIVITY_2 16384.0
#define ACCELERO_METER_SENSITIVITY_4 8192.0
#define ACCELERO_METER_SENSITIVITY_8 4096.0
#define ACCELERO_METER_SENSITIVITY_16 2048.0
#define GYROSCOPE_SENSITIVITY_250 131.0
#define GYROSCOPE_SENSITIVITY_500 65.5
#define GYROSCOPE_SENSITIVITY_1000 32.8
#define GYROSCOPE_SENSITIVITY_2000 16.4
#define ACCELERATION_DUE_TO_GRAVITY 9.81
#define ACCELERO_METER_RANGE_2 0
#define ACCELERO_METER_RANGE_4 1
#define ACCELERO_METER_RANGE_8 2
#define ACCELERO_METER_RANGE_16 3
#define GYROSCOPE_RANGE_250 0
#define GYROSCOPE_RANGE_500 1
#define GYROSCOPE_RANGE_1000 2
#define GYROSCOPE_RANGE_2000 3
#define LOWPASS_CUTOFF_FREQUENCY_250 0
#define LOWPASS_CUTOFF_FREQUENCY_184 1
#define LOWPASS_CUTOFF_FREQUENCY_92 2
#define LOWPASS_CUTOFF_FREQUENCY_41 3
#define LOWPASS_CUTOFF_FREQUENCY_20 4
#define LOWPASS_CUTOFF_FREQUENCY_10 5
#define LOWPASS_CUTOFF_FREQUENCY_5 6
class MPU6050 : public I2CDevice
{
private:
    float _accelerometer_sensitivity, _gyroscope_sensitivity, _acceleration_due_to_gravity;
public:
    MPU6050(byte address = DEFAULT_ADDRESS, TwoWire* preferred_wire = &Wire,float g = ACCELERATION_DUE_TO_GRAVITY);
    ~MPU6050();
    setAccelerometerRange(byte range);
    setGyroscopeRange(byte range);
    setSampleRateDivider(byte divider);
    setLowpassCutOffFrequency(byte frequency);
    enableSleepMode();
    disableSleepMode();
    getAccelerometerReadings(float& ax, float& ay, float& az);
    getGyroscopeReadings(float& gx, float& gy, float& gz);
    getSensorsReadings(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
};
#endif /* MPU6050_HPP_ */