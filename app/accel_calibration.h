#ifndef __ACCEL_CALIBRATION_H__
#define __ACCEL_CALIBRATION_H__

#include "app_common.h"
#include "qmc5883.h"
#include "hmc5883.h"
#include "mpu6050.h"
#include "mainloop_timer.h"

extern void accel_calib_init(void);
extern void accel_calib_perform(MPU6050_t* accel, void (*calib_done)(int axis_ndx, void*), void* arg);
extern void accel_calib_calculate(void);
extern void accel_calib_get_offset_gain(int32_t* offs, int32_t* gain);
extern void accel_get_acc_sum(int32_t sum[6][3], int32_t count[6]);

#endif /* !__ACCEL_CALIBRATION_H__ */
