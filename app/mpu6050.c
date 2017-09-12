#include "app_common.h"
#include "mpu6050.h"
#include "i2c_bus.h"

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline void
mpu6050_write_reg(MPU6050_t* mpu6050, uint8_t reg, uint8_t data)
{
  uint8_t buffer[2];

  buffer[0] = reg;
  buffer[1] = data;

  i2c_bus_write_sync(MPU6050_I2C_BUS, mpu6050->Address, buffer, 2);
}

static inline void
mpu6050_write_reg16(MPU6050_t* mpu6050, uint8_t reg, uint16_t data)
{
  uint8_t buffer[3];

  buffer[0] = reg;
  buffer[1] = (data >> 8 ) & 0xff;
  buffer[2] = data & 0xff;

  i2c_bus_write_sync(MPU6050_I2C_BUS, mpu6050->Address, buffer, 3);
}

static inline uint8_t
mpu6050_read_reg(MPU6050_t* mpu6050, uint8_t reg)
{
  uint8_t ret;

  i2c_bus_write_read(MPU6050_I2C_BUS, mpu6050->Address, &reg, 1, &ret, 1);

  return ret;
}

static inline void
mpu6050_read_data(MPU6050_t* mpu6050, uint8_t reg, uint8_t* data, uint8_t len)
{
  i2c_bus_write_read(MPU6050_I2C_BUS, mpu6050->Address, &reg, 1, data, len);
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
bool
mpu6050_init(MPU6050_t* mpu6050, MPU6050_Accelerometer_t accel_sensitivity, MPU6050_Gyroscope_t gyro_sensitivity)
{
	uint8_t	temp;

	mpu6050->Address = MPU6050_I2C_ADDR;

  /* Wakeup MPU6050 */
  mpu6050_write_reg(mpu6050, MPU6050_PWR_MGMT_1, 0x00);

  /* Config accelerometer */
  temp = mpu6050_read_reg(mpu6050, MPU6050_ACCEL_CONFIG);
  temp = (temp & 0xE7) | (uint8_t)accel_sensitivity << 3;
  mpu6050_write_reg(mpu6050, MPU6050_ACCEL_CONFIG, temp);

  /* Config gyroscope */
  temp = mpu6050_read_reg(mpu6050, MPU6050_GYRO_CONFIG);
  temp = (temp & 0xE7) | (uint8_t)gyro_sensitivity << 3;
  mpu6050_write_reg(mpu6050, MPU6050_GYRO_CONFIG, temp);


  /* Set sensitivities for multiplying gyro and accelerometer data */
  switch (accel_sensitivity)
  {
  case MPU6050_Accelerometer_2G:
    mpu6050->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2;
    mpu6050->one_g    = MPU6050_ACCE_SENS_2;
    break;

  case MPU6050_Accelerometer_4G:
    mpu6050->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4;
    mpu6050->one_g    = MPU6050_ACCE_SENS_4;
    break;

  case MPU6050_Accelerometer_8G:
    mpu6050->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8;
    mpu6050->one_g    = MPU6050_ACCE_SENS_8;
    break;

  case MPU6050_Accelerometer_16G:
    mpu6050->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16;
    mpu6050->one_g    = MPU6050_ACCE_SENS_16;
    break;

  default:
    break;
  }

  switch (gyro_sensitivity)
  {
  case MPU6050_Gyroscope_250s:
    mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250;
    break;
  
  case MPU6050_Gyroscope_500s:
    mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;
    break;

  case MPU6050_Gyroscope_1000s:
    mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000;
    break;

  case MPU6050_Gyroscope_2000s:
    mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000;
    break;

  default:
    break;
  }

  return true;
}

bool
mpu6050_read_accel(MPU6050_t* mpu6050)
{
  uint8_t data[6];

  /* Read accelerometer data */
  mpu6050_read_data(mpu6050, MPU6050_ACCEL_XOUT_H, data, 6);

  /* Format */
  mpu6050->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
  mpu6050->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
  mpu6050->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

  return true;

}

bool
mpu6050_read_gyro(MPU6050_t* mpu6050)
{
  uint8_t data[6];

  /* Read gyroscope data */
  mpu6050_read_data(mpu6050, MPU6050_GYRO_XOUT_H, data, 6);

  /* Format */
  mpu6050->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
  mpu6050->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
  mpu6050->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

  return true;
}

bool
mpu6050_read_temperature(MPU6050_t* mpu6050)
{
  uint8_t data[2];
  int16_t temp;

  /* Read temperature */
  mpu6050_read_data(mpu6050, MPU6050_TEMP_OUT_H, data, 2);

  /* Format temperature */
  temp = (data[0] << 8 | data[1]);
  mpu6050->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

  /* Return OK */
  return true;

}

bool
mpu6050_read_all(MPU6050_t* mpu6050)
{
  uint8_t data[14];
  int16_t temp;

  /* Read full raw data, 14bytes */
  mpu6050_read_data(mpu6050, MPU6050_ACCEL_XOUT_H, data, 14);

  /* Format accelerometer data */
  mpu6050->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
  mpu6050->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
  mpu6050->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

  /* Format temperature */
  temp = (data[6] << 8 | data[7]);
  mpu6050->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

  /* Format gyroscope data */
  mpu6050->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
  mpu6050->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
  mpu6050->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

  /* Return OK */
  return true;
}

void
mpu6050_set_gyro_offset(MPU6050_t* mpu6050, int16_t x, int16_t y, int16_t z)
{
  mpu6050_write_reg16(mpu6050, MPU6050_REG_GYRO_XOFFS_H, x);
  mpu6050_write_reg16(mpu6050, MPU6050_REG_GYRO_YOFFS_H, y);
  mpu6050_write_reg16(mpu6050, MPU6050_REG_GYRO_ZOFFS_H, z);
}

void
mpu6050_set_accel_offset(MPU6050_t* mpu6050, int16_t x, int16_t y, int16_t z)
{
  mpu6050_write_reg16(mpu6050, MPU6050_REG_ACCEL_XOFFS_H, x);
  mpu6050_write_reg16(mpu6050, MPU6050_REG_ACCEL_YOFFS_H, y);
  mpu6050_write_reg16(mpu6050, MPU6050_REG_ACCEL_ZOFFS_H, z);
}

void
mpu6050_set_gyro_dlpf(MPU6050_t* mpu6050, uint8_t val)
{
  mpu6050_write_reg(mpu6050, MPU6050_CONFIG, val);
}
