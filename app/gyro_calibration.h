#ifndef __GYRO_CALIBRATION_H__
#define __GYRO_CALIBRATION_H__

#include "app_common.h"
#include "qmc5883.h"
#include "hmc5883.h"
#include "mpu6050.h"
#include "mainloop_timer.h"
#include "sensor_calib.h"

extern void gyro_calib_init(void);
extern void gyro_calib_perform(MPU6050_t* gyro, void (*calib_done)(void*), void* arg);
extern void gyro_calib_get_offset(int16_t* offs);

#endif /* !__GYRO_CALIBRATION_H__ */
