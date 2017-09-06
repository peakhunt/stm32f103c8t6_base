#ifndef __IMU_DEF_H__
#define __IMU_DEF_H__

#include "app_common.h"
#include "qmc5883.h"
#include "hmc5883.h"
#include "mpu6050.h"
#include "mainloop_timer.h"
#include "madgwick.h"
#include "mahony.h"

typedef struct
{
  SoftTimerElem     sampling_timer;
#ifdef USE_QMC5883_MAG
  qmc5883Mag        mag;
#else
  hmc5883Mag        mag;
#endif
  MPU6050_t         mpu6050;       // mpu6050 core

  Madgwick          madgwick_ahrs;
  Mahony            mahony_ahrs;

  int16_t           accl_off[3];
  int16_t           gyro_off[3];
  int16_t           mag_bias[3];
  float             mag_scale[3];
} IMU_t;

extern void imu_init(IMU_t* imu);
extern void imu_get_mag(IMU_t* imu, float data[4]);
extern void imu_get_accel(IMU_t* imu, int16_t data[3], float fdata[3]);
extern void imu_get_gyro(IMU_t* imu, int16_t data[3], float fdata[3]);
extern void imu_get_pitch_roll_yaw(IMU_t* imu, float mahony[3], float madgwick[3]);
extern void imu_start(IMU_t* imu);
extern void imu_stop(IMU_t* imu);
extern IMU_t* imu_get_instance(int ndx);

#endif /* !__IMU_DEF_H__ */
