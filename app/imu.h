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

  float             orientation_madgwick[3];
  float             orientation_mahony[3];
  float             heading_madgwick;
  float             heading_mahony;
  float             heading_raw;
  float             mag_declination;
} IMU_t;

extern void imu_init(IMU_t* imu);
extern void imu_get_mag(IMU_t* imu, float data[4]);
extern void imu_get_accel(IMU_t* imu, int16_t data[3], float fdata[3]);
extern void imu_get_gyro(IMU_t* imu, int16_t data[3], float fdata[3]);
extern void imu_get_orientation(IMU_t* imu, float mahony[4], float madgwick[4]);
extern void imu_start(IMU_t* imu);
extern void imu_stop(IMU_t* imu);
extern void imu_set_offset(IMU_t* imu,
    int16_t gx, int16_t gy, int16_t gz,
    int16_t ax, int16_t ay, int16_t az);
extern void imu_get_offset(IMU_t* imu, int16_t gyro[3], int16_t accl[3]);
extern void imu_set_mag_calib(IMU_t* imu,
    int16_t x_bias, int16_t y_bias, int16_t z_bias,
    float x_scale, float y_scale, float z_scale);
extern void imu_get_mag_calib(IMU_t* imu, int16_t bias[3], float scale[3]);
extern IMU_t* imu_get_instance(int ndx);

#endif /* !__IMU_DEF_H__ */
