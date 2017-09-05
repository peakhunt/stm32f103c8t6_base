#ifndef __IMU_DEF_H__
#define __IMU_DEF_H__

extern void imu_init(void);
extern void imu_get_mag(float data[4]);
extern void imu_get_accel(int16_t data[3], float fdata[3]);
extern void imu_get_gyro(int16_t data[3], float fdata[3]);

#endif /* !__IMU_DEF_H__ */
