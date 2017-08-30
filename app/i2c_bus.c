#include "i2c_bus.h"
#include "i2c.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////
//#define I2C_DEFAULT_TIMEOUT     HAL_MAX_DELAY
#define I2C_DEFAULT_TIMEOUT     1000

////////////////////////////////////////////////////////////////////////////////
//
// private variables
//
////////////////////////////////////////////////////////////////////////////////
static I2C_HandleTypeDef* _hi2cs[] =
{
  &hi2c1,
  &hi2c2
};

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline I2C_HandleTypeDef*
get_i2c_handle(I2CBusNumber bus)
{
  return _hi2cs[bus];
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
i2c_bus_init(void)
{
  // everything is already initialized by CubeMX
}

bool
i2c_bus_write_sync(I2CBusNumber bus, I2CSlaveAddress  addr, uint8_t* data, uint16_t len)
{
  I2C_HandleTypeDef*    hi2c = get_i2c_handle(bus);
  HAL_StatusTypeDef     ret;

  ret = HAL_I2C_Master_Transmit(hi2c, (addr << 1), data, len, I2C_DEFAULT_TIMEOUT);
  if(ret != HAL_OK)
  {
    return false;
  }
  return true;
}

bool
i2c_bus_write_read(I2CBusNumber bus, I2CSlaveAddress  addr,
    uint8_t* wdata, uint8_t wdata_len,
    uint8_t* rdata, uint8_t rdata_len)
{
  I2C_HandleTypeDef*    hi2c = get_i2c_handle(bus);
  HAL_StatusTypeDef     ret;

  ret = HAL_I2C_Master_Transmit(hi2c, (addr << 1), wdata, wdata_len, I2C_DEFAULT_TIMEOUT);
  if(ret != HAL_OK)
  {
    return false;
  }

  if(HAL_I2C_Master_Receive(hi2c, (addr << 1), rdata, rdata_len, I2C_DEFAULT_TIMEOUT) != HAL_OK)
  {
    return false;
  }

  return true;
}

bool
i2c_bus_read_sync(I2CBusNumber bus, I2CSlaveAddress addr, uint8_t* data, uint16_t len)
{
  I2C_HandleTypeDef*    hi2c = get_i2c_handle(bus);
  HAL_StatusTypeDef     ret;

  ret = HAL_I2C_Master_Receive(hi2c, (addr << 1), data, len, I2C_DEFAULT_TIMEOUT);
  if(ret != HAL_OK)
  {
    return false;
  }
  return true;
}
