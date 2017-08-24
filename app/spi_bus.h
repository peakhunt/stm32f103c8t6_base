#ifndef __SPI_BUS_DEF_H__
#define __SPI_BUS_DEF_H__

#include "stm32f1xx_hal.h"
#include "app_common.h"

typedef enum
{
  SPIBus_0    = 0,
  SPIBus_1    = 1,
} SPIBusNumber;

extern void spi_bus_init(void);
extern void spi_bus_tranceive_sync(SPIBusNumber bus, uint8_t* tx, uint8_t* rx, uint16_t len);
extern void spi_bus_write_sync(SPIBusNumber bus, uint8_t* tx, uint16_t len);
extern void spi_bus_read_sync(SPIBusNumber bus, uint8_t* rx, uint16_t len);

#endif /* !__SPI_BUS_DEF_H__ */
