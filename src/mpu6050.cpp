#include "mpu6050.hpp"
#include "mpu6050_registers.hpp"

/**
 * @brief Construct a new MPU6050::MPU6050 object
 * @param address - I2C address of the MPU6050 device
 * @param preferred_wire - pointer to the preferred wire object
 * @param g - acceleration due to gravity
*/
MPU6050::MPU6050(byte address = DEFAULT_ADDRESS, TwoWire* preferred_wire = &Wire,float g = ACCELERATION_DUE_TO_GRAVITY) : I2CDevice(address, preferred_wire)
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
    writeBitsToReg(PWR_MGMT_1_REG, SLEEP_BITMASK, 1);
}

/**
 * @brief Disable the sleep mode
 * @note The MPU6050 device will enter sleep mode when the SLEEP bit is set to 1 in the PWR_MGMT_1 register. In sleep mode, only the serial interface and internal registers remain active, allowing for a very low standby current. Clearing the SLEEP bit in PWR_MGMT_1 wakes the device.
*/
void MPU6050::disableSleepMode()
{
    writeBitsToReg(PWR_MGMT_1_REG, SLEEP_BITMASK, 0);
}

/**
 * @brief Get the Accelerometer Readings and update the ax, ay, az variables
 * @param ax - acceleration in x direction
 * @param ay - acceleration in y direction
 * @param az - acceleration in z direction
 * @note The acceleration values are in m/s^2
*/
void MPU6050::getAccelerometerReadings(float& ax, float& ay, float& az)
{
    short int raw_a[3];
    readShortIntsFromReg(ACCEL_XOUT_H_REG, 3,  raw_a);
    ax = (float) _acceleration_due_to_gravity * raw_a[0] / _accelerometer_sensitivity;
    ay = (float) _acceleration_due_to_gravity * raw_a[1] / _accelerometer_sensitivity;
    az = (float) _acceleration_due_to_gravity * raw_a[2] / _accelerometer_sensitivity;
}

/**
 * @brief Get the Gyroscope Readings and update the gx, gy, gz variables
 * @param gx - angular velocity in x direction
 * @param gy - angular velocity in y direction
 * @param gz - angular velocity in z direction
 * @note The angular velocity values are in rad/s
*/
void MPU6050::getGyroscopeReadings(float& gx, float& gy, float& gz)
{
    short int raw_g[3];
    readShortIntsFromReg(GYRO_XOUT_H_REG, 3,  raw_g);
    gx = (float) raw_g[0] / _gyroscope_sensitivity;
    gy = (float) raw_g[1] / _gyroscope_sensitivity;
    gz = (float) raw_g[2] / _gyroscope_sensitivity;
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
void MPU6050::getSensorsReadings(float& ax, float& ay, float& az, float& gx, float& gy, float& gz)
{
    getAccelerometerReadings(ax, ay, az);
    getGyroscopeReadings(gx, gy, gz);

}