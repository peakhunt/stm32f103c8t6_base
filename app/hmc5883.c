#include <string.h>
#include "app_common.h"
#include "i2c_bus.h"
#include "hmc5883.h"

////////////////////////////////////////////////////////////////////////////////
//
// internal defines
//
////////////////////////////////////////////////////////////////////////////////
#define SENSORS_GAUSS_TO_MICROTESLA       (100)  

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline void
hmc5883_write_reg(hmc5883Mag* mag, uint8_t reg, uint8_t data)
{
  uint8_t   buffer[2];

  buffer[0] = reg;
  buffer[1] = data;

  i2c_bus_write_sync(HMC5883_I2C_BUS, mag->address, buffer, 2);
}

static inline void
hmc5883_read_reg(hmc5883Mag* mag, uint8_t reg, uint8_t* data, uint8_t len)
{
  i2c_bus_write_sync(HMC5883_I2C_BUS, mag->address, &reg, 1);
  if(i2c_bus_read_sync(HMC5883_I2C_BUS, mag->address, data, len) == false)
  {
    memset(data, 0, len);
  }
}


////////////////////////////////////////////////////////////////////////////////
//
// public utilities
//
////////////////////////////////////////////////////////////////////////////////
void
hmc5883_init(hmc5883Mag* mag, uint8_t address, hmc5883MagGain gain)
{
  mag->address    = address;

  mag->rx   = 0.0f;
  mag->ry   = 0.0f;
  mag->rz   = 0.0f;

  mag->gx   = 0.0f;
  mag->gy   = 0.0f;
  mag->gz   = 0.0f;

  //
  // enable magnetometer
  //
  hmc5883_write_reg(mag, HMC5883_REGISTER_MAG_MR_REG_M, 0x00);
  hmc5883_set_mag_gain(mag, gain);
}

void
hmc5883_set_mag_gain(hmc5883Mag* mag, hmc5883MagGain gain)
{
  //
  // cra setup
  // 7    : 0       , no temperature sensor
  // 6/5  : 00,     , 1 sample per measure
  // 4/2  : 111     , 220 Hz output rate
  // 1/0  : 00      , normal
  hmc5883_write_reg(mag, HMC5883_REGISTER_MAG_CRA_REG_M, 0x1c);

  //
  // crb setup
  // gain 
  hmc5883_write_reg(mag, HMC5883_REGISTER_MAG_CRB_REG_M, gain);

  // mode register
  // continuous measure mode
  hmc5883_write_reg(mag, HMC5883_REGISTER_MAG_MR_REG_M, 0);

  mag->mag_gain = gain;

  switch(gain)
  {
  case HMC5883_MAGGAIN_1_3:
    mag->Gauss_LSB_XY = 1100;
    mag->Gauss_LSB_Z  = 980;
    break;
  case HMC5883_MAGGAIN_1_9:
    mag->Gauss_LSB_XY = 855;
    mag->Gauss_LSB_Z  = 760;
    break;
  case HMC5883_MAGGAIN_2_5:
    mag->Gauss_LSB_XY = 670;
    mag->Gauss_LSB_Z  = 600;
    break;
  case HMC5883_MAGGAIN_4_0:
    mag->Gauss_LSB_XY = 450;
    mag->Gauss_LSB_Z  = 400;
    break;
  case HMC5883_MAGGAIN_4_7:
    mag->Gauss_LSB_XY = 400;
    mag->Gauss_LSB_Z  = 255;
    break;
  case HMC5883_MAGGAIN_5_6:
    mag->Gauss_LSB_XY = 330;
    mag->Gauss_LSB_Z  = 295;
    break;
  case HMC5883_MAGGAIN_8_1:
    mag->Gauss_LSB_XY = 230;
    mag->Gauss_LSB_Z  = 205;
    break;
  } 
}

void
hmc5883_read(hmc5883Mag* mag)
{
  uint8_t   data[6];

  hmc5883_read_reg(mag, HMC5883_REGISTER_MAG_OUT_X_H_M, data, 6);

  mag->rx   = (int16_t)(data[1] | ((int16_t)(data[0] << 8)));
  mag->rz   = (int16_t)(data[3] | ((int16_t)(data[2] << 8)));
  mag->ry   = (int16_t)(data[5] | ((int16_t)(data[4] << 8)));

  mag->gx   = (mag->rx / mag->Gauss_LSB_XY) * SENSORS_GAUSS_TO_MICROTESLA;
  mag->gy   = (mag->ry / mag->Gauss_LSB_XY) * SENSORS_GAUSS_TO_MICROTESLA;
  mag->gz   = (mag->rz / mag->Gauss_LSB_Z ) * SENSORS_GAUSS_TO_MICROTESLA;
}
