#ifndef __I2C_BUS_DEF_H__
#define __I2C_BUS_DEF_H__

#include "stm32f1xx_hal.h"
#include "app_common.h"

typedef enum
{
  I2CBus_0  = 0,
  I2CBus_1  = 1,
} I2CBusNumber;

#define I2CBUS_NUMBER_MAX     (I2CBus_1 + 1)

typedef struct
{
  uint32_t        num_attempt;
  uint32_t        num_success;
  uint32_t        num_failure;
} I2CBusStat;

typedef uint8_t I2CSlaveAddress;

extern void i2c_bus_init(void);
extern bool i2c_bus_write_sync(I2CBusNumber bus, I2CSlaveAddress  addr, uint8_t* data, uint16_t len);
extern bool i2c_bus_write_read(I2CBusNumber bus, I2CSlaveAddress  addr,
    uint8_t* wdata, uint8_t wdata_len,
    uint8_t* rdata, uint8_t rdata_len);
extern bool i2c_bus_read_sync(I2CBusNumber bus, I2CSlaveAddress addr, uint8_t* data, uint16_t len);
extern I2CBusStat* i2c_bus_get_stat(I2CBusNumber bus);

#endif /* !__I2C_BUS_DEF_H__ */
