#ifndef __IMU_DEF_H__
#define __IMU_DEF_H__

#include "app_common.h"
#include "qmc5883.h"
#include "hmc5883.h"
#include "mpu6050.h"
#include "mainloop_timer.h"
#include "madgwick.h"

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

  int16_t           accl_off[3];
  int16_t           accl_scale[3];
  int16_t           gyro_off[3];
  int16_t           mag_bias[3];

  int32_t           gyro_value[3];
  int32_t           accl_value[3];
  int32_t           mag_value[3];

  float             orientation_madgwick[3];
  float             heading_madgwick;
  float             heading_raw;
  float             mag_declination;
} IMU_t;

extern void imu_init(IMU_t* imu);

extern void imu_get_accel(IMU_t* imu, int32_t data[3]);
extern void imu_get_gyro(IMU_t* imu, int32_t data[3]);
extern void imu_get_mag(IMU_t* imu, int32_t data[3]);

extern void imu_get_orientation(IMU_t* imu, float madgwick[4]);
extern void imu_start(IMU_t* imu);
extern void imu_stop(IMU_t* imu);
extern void imu_get_offset(IMU_t* imu, int16_t gyro[3], int16_t accl[3]);
extern void imu_set_mag_calib(IMU_t* imu, int16_t x_bias, int16_t y_bias, int16_t z_bias);
extern void imu_get_mag_calib(IMU_t* imu, int16_t bias[3]);
extern IMU_t* imu_get_instance(int ndx);

extern void imu_set_gyro_calib(IMU_t* imu, int16_t gx, int16_t gy, int16_t gz);
extern void imu_set_accel_calib(IMU_t* imu, int16_t ax, int16_t ay, int16_t az,
    int16_t sx, int16_t sy, int16_t sz);

#endif /* !__IMU_DEF_H__ */
