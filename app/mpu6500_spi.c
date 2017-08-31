#include "app_common.h"
#include "spi_bus.h"
#include "mpu6500_spi.h"

#define MPU6500_SPI_BUS     SPIBus_0

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline void
mpu6500_spi_cs_select(MPU6500_t* mpu6500)
{
}

static inline void
mpu6500_spi_cs_unselect(MPU6500_t* mpu6500)
{
}

static inline void
mpu6500_spi_write_reg(MPU6500_t* mpu6500, uint8_t reg, uint8_t data)
{
  uint8_t   buffer[2];

  buffer[0] = reg;
  buffer[1] = data;

  mpu6500_spi_cs_select(mpu6500);
  spi_bus_write_sync(MPU6500_SPI_BUS, buffer, 2);
  mpu6500_spi_cs_unselect(mpu6500);
}

static inline void
mpu6500_spi_write_reg16(MPU6500_t* mpu6500, uint8_t reg, uint16_t data)
{
  uint8_t buffer[3];

  buffer[0] = reg;
  buffer[1] = (data >> 8 ) & 0xff;
  buffer[2] = data & 0xff;

  mpu6500_spi_cs_select(mpu6500);
  spi_bus_write_sync(MPU6500_SPI_BUS, buffer, 3);
  mpu6500_spi_cs_unselect(mpu6500);
}

static inline uint8_t
mpu6500_spi_read_reg(MPU6500_t* mpu6500, uint8_t reg)
{
  uint8_t ret;

  mpu6500_spi_cs_select(mpu6500);
  spi_bus_read_sync(MPU6500_SPI_BUS, &ret, 1);
  mpu6500_spi_cs_unselect(mpu6500);

  return ret;
}

static inline void
mpu6500_spi_read_data(MPU6500_t* mpu6500, uint8_t reg, uint8_t* data, uint8_t len)
{
  mpu6500_spi_cs_select(mpu6500);
  spi_bus_write_sync(MPU6500_SPI_BUS, &reg, 1);
  spi_bus_read_sync(MPU6500_SPI_BUS, data, len);
  mpu6500_spi_cs_unselect(mpu6500);
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
bool
mpu6500_init(MPU6500_t* mpu6500, MPU6500_Accelerometer_t accel_sensitivity,
    MPU6500_Gyroscope_t gyro_sensitivity)
{
  uint8_t   temp;

  /* Wakeup MPU6500 */
  mpu6500_spi_write_reg(mpu6500, MPU6500_PWR_MGMT_1, 0x00);

  /* Disable Primary I2C interface */
  mpu6500_spi_write_reg(mpu6500, MPU6500_USER_CTRL, MPU6500_BIT_I2C_IF_DIS);

  /* Delay 100ms */

  temp = mpu6500_spi_read_reg(mpu6500, MPU6500_ACCEL_CONFIG);
  temp = (temp & 0xE7) | (uint8_t)accel_sensitivity << 3;
  mpu6500_spi_write_reg(mpu6500, MPU6500_ACCEL_CONFIG, temp);

  /* Config gyroscope */
  temp = mpu6500_spi_read_reg(mpu6500, MPU6500_GYRO_CONFIG);
  temp = (temp & 0xE7) | (uint8_t)gyro_sensitivity << 3;
  mpu6500_spi_write_reg(mpu6500, MPU6500_GYRO_CONFIG, temp);

  /* Set sensitivities for multiplying gyro and accelerometer data */
  switch (accel_sensitivity)
  {
  case MPU6500_Accelerometer_2G:
    mpu6500->Acce_Mult = (float)1 / MPU6500_ACCE_SENS_2;
    break;

  case MPU6500_Accelerometer_4G:
    mpu6500->Acce_Mult = (float)1 / MPU6500_ACCE_SENS_4;
    break;

  case MPU6500_Accelerometer_8G:
    mpu6500->Acce_Mult = (float)1 / MPU6500_ACCE_SENS_8;
    break;

  case MPU6500_Accelerometer_16G:
    mpu6500->Acce_Mult = (float)1 / MPU6500_ACCE_SENS_16;
    break;

  default:
    break;
  }

  switch (gyro_sensitivity)
  {
  case MPU6500_Gyroscope_250s:
    mpu6500->Gyro_Mult = (float)1 / MPU6500_GYRO_SENS_250;
    break;
  
  case MPU6500_Gyroscope_500s:
    mpu6500->Gyro_Mult = (float)1 / MPU6500_GYRO_SENS_500;
    break;

  case MPU6500_Gyroscope_1000s:
    mpu6500->Gyro_Mult = (float)1 / MPU6500_GYRO_SENS_1000;
    break;

  case MPU6500_Gyroscope_2000s:
    mpu6500->Gyro_Mult = (float)1 / MPU6500_GYRO_SENS_2000;
    break;

  default:
    break;
  }

  return true;
}

bool
mpu6500_read_accel(MPU6500_t* mpu6500)
{
  uint8_t data[6];

  /* Read accelerometer data */
  mpu6500_spi_read_data(mpu6500, MPU6500_ACCEL_XOUT_H, data, 6);

  /* Format */
  mpu6500->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
  mpu6500->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
  mpu6500->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

  return true;
}

bool
mpu6500_read_gyro(MPU6500_t* mpu6500)
{
  uint8_t data[6];

  /* Read gyroscope data */
  mpu6500_spi_read_data(mpu6500, MPU6500_GYRO_XOUT_H, data, 6);

  /* Format */
  mpu6500->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
  mpu6500->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
  mpu6500->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

  return true;
}

bool
mpu6500_read_temperature(MPU6500_t* mpu6500)
{
  uint8_t data[2];
  int16_t temp;

  /* Read temperature */
  mpu6500_spi_read_data(mpu6500, MPU6500_TEMP_OUT_H, data, 2);

  /* Format temperature */
  temp = (data[0] << 8 | data[1]);
  mpu6500->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

  /* Return OK */
  return true;

}

bool
mpu6500_read_all(MPU6500_t* mpu6500)
{
  uint8_t data[14];
  int16_t temp;

  /* Read full raw data, 14bytes */
  mpu6500_spi_read_data(mpu6500, MPU6500_ACCEL_XOUT_H, data, 14);

  /* Format accelerometer data */
  mpu6500->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
  mpu6500->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
  mpu6500->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

  /* Format temperature */
  temp = (data[6] << 8 | data[7]);
  mpu6500->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

  /* Format gyroscope data */
  mpu6500->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
  mpu6500->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
  mpu6500->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

  /* Return OK */
  return true;
}

void
mpu6500_set_gyro_offset(MPU6500_t* mpu6500, int16_t x, int16_t y, int16_t z)
{
  mpu6500_spi_write_reg16(mpu6500, MPU6500_REG_GYRO_XOFFS_H, x);
  mpu6500_spi_write_reg16(mpu6500, MPU6500_REG_GYRO_YOFFS_H, y);
  mpu6500_spi_write_reg16(mpu6500, MPU6500_REG_GYRO_ZOFFS_H, z);
}

void
mpu6500_set_accel_offset(MPU6500_t* mpu6500, int16_t x, int16_t y, int16_t z)
{
  mpu6500_spi_write_reg16(mpu6500, MPU6500_REG_ACCEL_XOFFS_H, x);
  mpu6500_spi_write_reg16(mpu6500, MPU6500_REG_ACCEL_YOFFS_H, y);
  mpu6500_spi_write_reg16(mpu6500, MPU6500_REG_ACCEL_ZOFFS_H, z);
}

void
mpu6500_set_gyro_dlpf(MPU6500_t* mpu6500, uint8_t val)
{
  mpu6500_spi_write_reg(mpu6500, MPU6500_CONFIG, val);
}
