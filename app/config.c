#include "config.h"
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal_flash.h"

#define CONFIG_START_ADDR           (0x08000000 + FLASH_PAGE_SIZE * 63)      // at 63K
#define CONFIG_END_ADDR             (CONFIG_START_ADDR + FLASH_PAGE_SIZE)    // exactly 1K

typedef struct {
  config_t    cfg;
  uint32_t    crc;
} config_internal_t;

////////////////////////////////////////////////////////////////////////////////
//
// private globals
//
////////////////////////////////////////////////////////////////////////////////
static config_internal_t     _config = 
{
  .cfg = {
    .version        = CONFIG_VERSION,
    .magic          = CONFIG_MAGIC,
    .accl_off[0]    = 0,
    .accl_off[1]    = 0,
    .accl_off[2]    = 0,
    .accl_scale[0]  = 4096,
    .accl_scale[1]  = 4096,
    .accl_scale[2]  = 4096,
    .gyro_off[0]    = 0,
    .gyro_off[1]    = 0,
    .gyro_off[2]    = 0,
    .mag_bias[0]    = 0,
    .mag_bias[1]    = 0,
    .mag_bias[2]    = 0,
  },
  .crc            = 0,
};

////////////////////////////////////////////////////////////////////////////////
//
// checksum utilities
//
////////////////////////////////////////////////////////////////////////////////
uint16_t
crc16_ccitt(uint16_t crc, unsigned char a)
{
  crc ^= (uint16_t)a << 8;

  for (int ii = 0; ii < 8; ++ii) {
    if (crc & 0x8000) {
      crc = (crc << 1) ^ 0x1021;
    } else {
      crc = crc << 1;
    }
  }
  return crc;
}

static uint16_t
calcCRC(uint16_t crc, const void *data, uint32_t length)
{
  const uint8_t *p = (const uint8_t *)data;
  const uint8_t *pend = p + length;

  for (; p != pend; p++) {
    crc = crc16_ccitt(crc, *p);
  }
  return crc;
}

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
static bool
is_flash_config_valid(void)
{
  config_internal_t* flash_cfg = (config_internal_t*)CONFIG_START_ADDR;
  int16_t   flash_crc;

  if(flash_cfg->cfg.magic != CONFIG_MAGIC)
  {
    return false;
  }

  flash_crc = calcCRC(0, (const void*)flash_cfg, sizeof(config_t));

  if(flash_cfg->crc != flash_crc)
    return false;

  return true;
}

static void
erase_program_config_to_flash(void)
{
  FLASH_EraseInitTypeDef    eraseStruct;
  uint32_t                  pageErr;
  uint32_t                  *data_ptr;
  uint32_t                  addr;

  HAL_FLASH_Unlock();

  eraseStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  eraseStruct.PageAddress = CONFIG_START_ADDR;
  eraseStruct.NbPages     = (CONFIG_END_ADDR - CONFIG_START_ADDR) / FLASH_PAGE_SIZE;
  if (HAL_FLASHEx_Erase(&eraseStruct, &pageErr) != HAL_OK)
  {
    while(1); // infinite loop to indicate something went wrong 
  }

  data_ptr  = (uint32_t*)&_config;
  addr      = CONFIG_START_ADDR;

  while(addr < (CONFIG_START_ADDR + sizeof(config_internal_t)))
  {
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *data_ptr) != HAL_OK)
    {
      while(1); // infinite loop to indicate something went wrong 
    }
    addr += 4;
    data_ptr++;
  }
  HAL_FLASH_Lock();
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
config_init(void)
{
  config_internal_t* flash_cfg = (config_internal_t*)CONFIG_START_ADDR;

  if(is_flash_config_valid() == false) {
    // configuration in flash is not valid
    return;
  }

  //
  // just copy
  //
  memcpy(&_config, flash_cfg, sizeof(config_internal_t));
}

config_t*
config_get(void)
{
  return &(_config.cfg);
}

config_t*
config_get_flash(void)
{
  return (config_t*)CONFIG_START_ADDR;
}

void
config_save(void)
{
  _config.crc = calcCRC(0, (const void*)&(_config.cfg), sizeof(config_t));

  __disable_irq();
  erase_program_config_to_flash();
  __enable_irq();
}
