#include <string.h>
#include "app_common.h"
#include "i2c_bus.h"
#include "qmc5883.h"

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
qmc5883_write_reg(qmc5883Mag* mag, uint8_t reg, uint8_t data)
{
  uint8_t   buffer[2];

  buffer[0] = reg;
  buffer[1] = data;

  i2c_bus_write_sync(QMC5883_I2C_BUS, mag->address, buffer, 2);
}

static inline void
qmc5883_read_reg(qmc5883Mag* mag, uint8_t reg, uint8_t* data, uint8_t len)
{
  i2c_bus_write_sync(QMC5883_I2C_BUS, mag->address, &reg, 1);
  if(i2c_bus_read_sync(QMC5883_I2C_BUS, mag->address, data, len) == false)
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
qmc5883_init(qmc5883Mag* mag, uint8_t address)
{
  uint8_t   config = 0;

  mag->address    = address;

  mag->rx   = 0.0f;
  mag->ry   = 0.0f;
  mag->rz   = 0.0f;

  mag->gx   = 0.0f;
  mag->gy   = 0.0f;
  mag->gz   = 0.0f;

  //
  // configure magnetometer as
  // Mode : Confinuous (01)
  // output data rate : 100 Hz (10)
  // full scale       : 2G (00);
  // over sample ratio: 512
  config = 0x01;          // mode
  config |= (0x03 << 2);  // output data rate
  config |= (0x00 << 4);  // full scale
  config |= (0x00 << 6);  // oversample ratio
  qmc5883_write_reg(mag, QMC5883_REGISTER_MAG_CTRL_REG1,  config);
}

void
qmc5883_read(qmc5883Mag* mag)
{
  uint8_t   data[6];

  qmc5883_read_reg(mag, QMC5883_REGISTER_MAG_OUT_X_L_M, data, 6);

  mag->rx   = (int16_t)(data[0] | ((int16_t)(data[1] << 8)));
  mag->ry   = (int16_t)(data[2] | ((int16_t)(data[3] << 8)));
  mag->rz   = (int16_t)(data[4] | ((int16_t)(data[5] << 8)));

  mag->gx   = mag->rx;
  mag->gy   = mag->ry;
  mag->gz   = mag->rz;
}
