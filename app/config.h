#ifndef __CONFIG_DEF_H__
#define __CONFIG_DEF_H__

#include "app_common.h"

#define CONFIG_VERSION          1

#define CONFIG_MAGIC            0x63149654

typedef struct {
  volatile int32_t       version;
  volatile uint32_t      magic;

  volatile int16_t       accl_off[3];
  volatile int16_t       accl_scale[3];
  volatile int16_t       gyro_off[3];
  volatile int16_t       mag_bias[3];
} config_t;

extern void config_init(void);
extern config_t* config_get(void);
extern config_t* config_get_flash(void);
extern void config_save(void);

#endif /* !__CONFIG_DEF_H__ */
