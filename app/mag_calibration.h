#ifndef __MAG_CALIBRATION_H__
#define __MAG_CALIBRATION_H__

#include "app_common.h"
#include "qmc5883.h"
#include "hmc5883.h"
#include "mpu6050.h"
#include "mainloop_timer.h"

extern void mag_calib_init(void);
extern void mag_calib_perform(hmc5883Mag* mag, void (*calib_done)(void*), void* arg);
extern void mag_calib_get_offset(int32_t* offs);

#endif /* !__MAG_CALIBRATION_H__ */
