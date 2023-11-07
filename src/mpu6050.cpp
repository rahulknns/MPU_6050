#include "mpu6050.hpp"
#include "mpu6050_registers.hpp"

/**
 * @brief Construct a new MPU6050::MPU6050 object
 * @param address - I2C address of the MPU6050 device
 * @param preferred_wire - pointer to the preferred wire object
 * @param g - acceleration due to gravity
*/
MPU6050::MPU6050(byte address , TwoWire* preferred_wire  ,double g ) : I2CDevice(address, preferred_wire)
{
    _accelerometer_sensitivity = ACCELERO_METER_SENSITIVITY_2;
    _gyroscope_sensitivity = GYROSCOPE_SENSITIVITY_250;
    _acceleration_due_to_gravity = g;
    disableSleepMode();

}

/**
 * @brief Destroy the MPU6050::MPU6050 object
*/
MPU6050::~MPU6050()
{

}

/**
 * @brief Set the Accelerometer Range object
 * @param range - accelerometer range, Should be one of the following: ACCELERO_METER_RANGE_2, ACCELERO_METER_RANGE_4, ACCELERO_METER_RANGE_8, ACCELERO_METER_RANGE_16
*/
void MPU6050::setAccelerometerRange(byte range)
{
    writeBitsToReg(ACCEL_CONFIG_REG, ACCEL_RANGE_BITMASK, range); 
    switch (range)
    {
    case ACCELERO_METER_RANGE_2:
        _accelerometer_sensitivity = ACCELERO_METER_SENSITIVITY_2;
        break;
    case ACCELERO_METER_RANGE_4:
        _accelerometer_sensitivity = ACCELERO_METER_SENSITIVITY_4;
        break;
    case ACCELERO_METER_RANGE_8:
        _accelerometer_sensitivity = ACCELERO_METER_SENSITIVITY_8;
        break;
    case ACCELERO_METER_RANGE_16:
        _accelerometer_sensitivity = ACCELERO_METER_SENSITIVITY_16;
        break;
    default:
        break;
    }
}

/**
 * @brief Set the Gyroscope Range object
 * @param range - gyroscope range, Should be one of the following: GYROSCOPE_RANGE_250, GYROSCOPE_RANGE_500, GYROSCOPE_RANGE_1000, GYROSCOPE_RANGE_2000
*/
void MPU6050::setGyroscopeRange(byte range)
{
    writeBitsToReg(GYRO_CONFIG_REG, GYRO_RANGE_BITMASK, range); 
    switch (range)
    {
    case GYROSCOPE_RANGE_250:
        _gyroscope_sensitivity = GYROSCOPE_SENSITIVITY_250;
        break;
    case GYROSCOPE_RANGE_500:
        _gyroscope_sensitivity = GYROSCOPE_SENSITIVITY_500;
        break; 
    case GYROSCOPE_RANGE_1000:
        _gyroscope_sensitivity = GYROSCOPE_SENSITIVITY_1000;
        break;
    case GYROSCOPE_RANGE_2000:
        _gyroscope_sensitivity = GYROSCOPE_SENSITIVITY_2000;
        break;
    default:
        break;
    }
}

/**
 * @brief Set the Sample Rate Divider value
 * @param divider - sample rate divider
 * @note Sample rate = 1kHz / (1 + divider) for accelerometer and gyroscope, if the DLPF is disabled gyro sample rate is 8kHz/ (1 + divider)
*/
void MPU6050::setSampleRateDivider(byte divider)
{
    writeByteToReg(SMPLRT_DIV_REG, divider);
}

/**
 * @brief Set the Lowpass Cut Off Frequency value
 * @param frequency - lowpass cut off frequency, Should be one of the following: LOWPASS_CUTOFF_FREQUENCY_250, LOWPASS_CUTOFF_FREQUENCY_184, LOWPASS_CUTOFF_FREQUENCY_92, LOWPASS_CUTOFF_FREQUENCY_41, LOWPASS_CUTOFF_FREQUENCY_20, LOWPASS_CUTOFF_FREQUENCY_10, LOWPASS_CUTOFF_FREQUENCY_5
*/
void MPU6050::setLowpassCutOffFrequency(byte frequency)
{
    writeBitsToReg(CONFIG_REG, LOWPASS_CUTOFF_BITMASK, frequency);
}

/**
 * @brief Enable the sleep mode
 * @note The MPU6050 device will enter sleep mode when the SLEEP bit is set to 1 in the PWR_MGMT_1 register. In sleep mode, only the serial interface and internal registers remain active, allowing for a very low standby current. Clearing the SLEEP bit in PWR_MGMT_1 wakes the device.
*/
void MPU6050::enableSleepMode()
{
    writeBitsToReg(PWR_MGMT_1_REG, SLEEP_BITMASK, 1<<6);
}

/**
 * @brief Disable the sleep mode
 * @note The MPU6050 device will enter sleep mode when the SLEEP bit is set to 1 in the PWR_MGMT_1 register. In sleep mode, only the serial interface and internal registers remain active, allowing for a very low standby current. Clearing the SLEEP bit in PWR_MGMT_1 wakes the device.
*/
void MPU6050::disableSleepMode()
{
    writeBitsToReg(PWR_MGMT_1_REG, SLEEP_BITMASK, 0<<6);
}


/**
 * @brief Calibrate the sensor
 * @param n - number of samples to take
 * @note The calibration is done by taking n samples and averaging them
 * @note During calibration the sensor should be kept still
*/
void MPU6050::calibrateSensor(int n){
    double ax, ay, az, gx, gy, gz;
    for (int  sample_no = 0; sample_no < n; sample_no++)
    {
        getSensorsReadings(ax, ay, az, gx, gy, gz,false);
        _ax_offset += ax;
        _ay_offset += ay;
        _az_offset += az;
        _gx_offset += gx;
        _gy_offset += gy;
        _gz_offset += gz;
        delay(50);
    }
    _ax_offset /= n;
    _ay_offset /= n;
    _az_offset /= n;
    _gx_offset /= n;
    _gy_offset /= n;
    _gz_offset /= n;
    
}

/**
 * @brief Get the Accelerometer Readings and update the ax, ay, az variables
 * @param ax - acceleration in x direction
 * @param ay - acceleration in y direction
 * @param az - acceleration in z direction
 * @note The acceleration values are in m/s^2
*/
void MPU6050::getAccelerometerReadings(double& ax, double& ay, double& az,bool calibrated)
{
    short int raw_a[3];
    readShortIntsFromReg(ACCEL_XOUT_H_REG, 3,  raw_a);
    ax = (double) _acceleration_due_to_gravity * raw_a[0] * _accelerometer_sensitivity;
    ay = (double) _acceleration_due_to_gravity * raw_a[1] * _accelerometer_sensitivity;
    az = (double) _acceleration_due_to_gravity * raw_a[2] * _accelerometer_sensitivity;
    ax -= calibrated ? _ax_offset : 0;
    ay -= calibrated ? _ay_offset : 0;
    az -= calibrated ? _az_offset : 0;
}

/**
 * @brief Get the Gyroscope Readings and update the gx, gy, gz variables
 * @param gx - angular velocity in x direction
 * @param gy - angular velocity in y direction
 * @param gz - angular velocity in z direction
 * @note The angular velocity values are in rad/s
*/
void MPU6050::getGyroscopeReadings(double& gx, double& gy, double& gz,bool calibrated)
{
    short int raw_g[3];
    readShortIntsFromReg(GYRO_XOUT_H_REG, 3,  raw_g);
    gx = (double) raw_g[0] * _gyroscope_sensitivity;
    gy = (double) raw_g[1] * _gyroscope_sensitivity;
    gz = (double) raw_g[2] * _gyroscope_sensitivity;
    gx -= calibrated ? _gx_offset : 0;
    gy -= calibrated ? _gy_offset : 0;
    gz -= calibrated ? _gz_offset : 0;
}

/**
 * @brief Get the Accelerometer and Gyroscope Readings and update the ax, ay, az, gx, gy, gz variables
 * @param ax - acceleration in x direction
 * @param ay - acceleration in y direction
 * @param az - acceleration in z direction
 * @param gx - angular velocity in x direction
 * @param gy - angular velocity in y direction
 * @param gz - angular velocity in z direction
 * @note The acceleration values are in m/s^2 and the angular velocity values are in rad/s
*/
void MPU6050::getSensorsReadings(double& ax, double& ay, double& az, double& gx, double& gy, double& gz,bool calibrated)
{
    getAccelerometerReadings(ax, ay, az, calibrated);
    getGyroscopeReadings(gx, gy, gz, calibrated);

}
