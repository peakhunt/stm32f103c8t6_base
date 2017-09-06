#ifndef __IMU_CALIBRATION_DEF_H__
#define __IMU_CALIBRATION_DEF_H__

extern void imu_calibration_init(void);
extern void imu_calibration_accel_perform(void (*calib_done)(void*), void* arg);
extern void imu_calibration_mag_perform(void (*calib_done)(void*), void* arg);

#endif /* !__IMU_CALIBRATION_DEF_H__ */
