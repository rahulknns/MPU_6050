#ifndef MPU6050_HPP_
#define MPU6050_HPP_
#include <Arduino.h>
#include <Wire.h>
#include "I2C_driver/src/I2C_device.hpp"
#include "mpu6050_registers.hpp"
#define DEFAULT_ADDRESS 0x68 
#define ACCELERO_METER_SENSITIVITY_2 0.00003051757812*2
#define ACCELERO_METER_SENSITIVITY_4 0.00006103515625*2
#define ACCELERO_METER_SENSITIVITY_8 0.0001220703125*2
#define ACCELERO_METER_SENSITIVITY_16 0.000244140625*2
#define GYROSCOPE_SENSITIVITY_250 0.003814697266*2
#define GYROSCOPE_SENSITIVITY_500 0.007629394531*2
#define GYROSCOPE_SENSITIVITY_1000 0.01525878906*2
#define GYROSCOPE_SENSITIVITY_2000 0.03051757812*2
#define ACCELERATION_DUE_TO_GRAVITY 9.80665
#define ACCELERO_METER_RANGE_2 0<<3
#define ACCELERO_METER_RANGE_4 1<<3
#define ACCELERO_METER_RANGE_8 2<<3
#define ACCELERO_METER_RANGE_16 3<<3
#define GYROSCOPE_RANGE_250 0<<3
#define GYROSCOPE_RANGE_500 1<<3
#define GYROSCOPE_RANGE_1000 2<<3
#define GYROSCOPE_RANGE_2000 3<<3
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
    double _accelerometer_sensitivity, _gyroscope_sensitivity, _acceleration_due_to_gravity;
    double _ax_offset = 0.0, _ay_offset = 0.0, _az_offset = 0.0;
    double _gx_offset = 0.0, _gy_offset = 0.0, _gz_offset = 0.0;
public:
    MPU6050(byte address = DEFAULT_ADDRESS, TwoWire* preferred_wire = &Wire ,double g = ACCELERATION_DUE_TO_GRAVITY);
    ~MPU6050();
    void setAccelerometerRange(byte range);
    void setGyroscopeRange(byte range);
    void setSampleRateDivider(byte divider);
    void setLowpassCutOffFrequency(byte frequency);
    void enableSleepMode();
    void disableSleepMode();
    void calibrateSensor(int n = 100);
    void getAccelerometerReadings(double& ax, double& ay, double& az, bool calibrated = true);
    void getGyroscopeReadings(double& gx, double& gy, double& gz, bool calibrated = true);
    void getSensorsReadings(double& ax, double& ay, double& az, double& gx, double& gy, double& gz, bool calibrated = true);
};
#endif /* MPU6050_HPP_ */
