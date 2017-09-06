#ifndef __APP_COMMON_DEF_H__
#define __APP_COMMON_DEF_H__

#include <stdint.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"

#define bool      uint8_t
#define true      1
#define false     0

////////////////////////////////////////////////////////////////////////////////
//
// system uptime. defined in stm32f1xx_callback.c
//
////////////////////////////////////////////////////////////////////////////////
extern volatile uint32_t     __uptime;

////////////////////////////////////////////////////////////////////////////////
//
// I2C device layout
//
////////////////////////////////////////////////////////////////////////////////
#define MPU6050_I2C_BUS                   I2CBus_0
#define BMP180_I2C_BUS                    I2CBus_1
#define QMC5883_I2C_BUS                   I2CBus_0
#define HMC5883_I2C_BUS                   I2CBus_0

#endif //!__APP_COMMON_DEF_H__
