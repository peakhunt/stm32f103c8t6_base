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

  config = 0x01;          // mode : continuous
  config |= (0x03 << 2);  // output data rate: 200 Hz
  config |= (0x00 << 4);  // full scale : 2G
  config |= (0x00 << 6);  // oversample ratio : 256

  qmc5883_write_reg(mag, QMC5883_REGISTER_MAG_CTRL_REG1,  config);

  //mag->to_mgauss = (1.0f/12000) * 1000;         // in 2G range, 12000 is 1 Gauss
                                                // in 8G ramge 3000 is 1 Gauss
  mag->to_mgauss = 1.0f;  // test
}

void
qmc5883_read(qmc5883Mag* mag)
{
  uint8_t   data[6];

  qmc5883_read_reg(mag, QMC5883_REGISTER_MAG_OUT_X_L_M, data, 6);

  mag->rx   = (int16_t)(data[0] | ((int16_t)(data[1] << 8)));
  mag->ry   = (int16_t)(data[2] | ((int16_t)(data[3] << 8)));
  mag->rz   = (int16_t)(data[4] | ((int16_t)(data[5] << 8)));

  mag->rx  = mag->rx * 10;
  //mag->ry  = mag->ry * 10;
  //mag->rz  = mag->rz * 10;
}
