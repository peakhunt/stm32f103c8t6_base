#include <math.h>

#include "imu.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////

#define IMU_SAMPLE_INTERVAL                 2         // mms. 500 Hz
#define IMU_SAMPLE_FREQUENCY                (1000.0f/IMU_SAMPLE_INTERVAL)

#define TO_DEGREE   (180 / M_PI)
#define TO_RADIAN		(M_PI / 180)

//
// X axis : Pitch
// Y axis : Roll
// Z axis : Yaw
//

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// AHRS related
//
////////////////////////////////////////////////////////////////////////////////
static inline float
to_gyro_dps(int32_t gyro_v, MPU6050_t* mpu)
{
  return (float)gyro_v * mpu->Gyro_Mult;
}

static void
ahrs_update(IMU_t* imu)
{
  int32_t     gx, gy, gz;
  int32_t     ax, ay, az;
  int32_t     mx, my, mz;
  int32_t     tmp;

  // gyro zero calibration
  gx =  (imu->mpu6050.Gyroscope_X - imu->gyro_off[0]);
  gy =  (imu->mpu6050.Gyroscope_Y - imu->gyro_off[1]);
  gz =  (imu->mpu6050.Gyroscope_Z - imu->gyro_off[2]);
  // FIXME sensor align

  // accel zero/scale calibration
  ax =  (imu->mpu6050.Accelerometer_X - imu->accl_off[0]) * imu->accl_scale[0] / 4096;
  ay =  (imu->mpu6050.Accelerometer_Y - imu->accl_off[1]) * imu->accl_scale[1] / 4096;
  az =  (imu->mpu6050.Accelerometer_Z - imu->accl_off[2]) * imu->accl_scale[2] / 4096;
  // FIXME sensor align

  // compass calibration
  mx = imu->mag.rx - imu->mag_bias[0];
  my = imu->mag.ry - imu->mag_bias[1];
  mz = imu->mag.rz - imu->mag_bias[2];

  // XXX sensor align
  tmp = mx;
  mx =  my;
  my = -tmp;
  

  imu->gyro_value[0] = gx;
  imu->gyro_value[1] = gy;
  imu->gyro_value[2] = gz;

  imu->accl_value[0] = ax;
  imu->accl_value[1] = ay;
  imu->accl_value[2] = az;

  imu->mag_value[0] = mx;
  imu->mag_value[1] = my;
  imu->mag_value[2] = mz;

  //////////////////////////////////////////////////////////////
  //
  // XXX
  // acceleration and magnetometer values can be in any unit.
  // but gyro values should be in deg/s.
  //
  //////////////////////////////////////////////////////////////
  madgwick_update(&imu->madgwick_ahrs,
      to_gyro_dps(gx, &imu->mpu6050),
      to_gyro_dps(gy, &imu->mpu6050),
      to_gyro_dps(gz, &imu->mpu6050),       // gyro
      ax, ay, az,       // accel
      mx, my, mz        // mag
  );

  madgwick_get_roll_pitch_yaw(&imu->madgwick_ahrs,  imu->orientation_madgwick);

  imu->heading_madgwick *= -1.0f;
}

////////////////////////////////////////////////////////////////////////////////
//
// sampling timer callback
//
////////////////////////////////////////////////////////////////////////////////
static void
imu_read(SoftTimerElem* te)
{
  IMU_t* imu = container_of(te, IMU_t, sampling_timer);

  mpu6050_read_all(&imu->mpu6050);

#if 1
#ifdef USE_QMC5883_MAG
  qmc5883_read(&imu->mag);
#else
  hmc5883_read(&imu->mag);
#endif
#endif

  ahrs_update(imu);

  mainloop_timer_schedule(&imu->sampling_timer, IMU_SAMPLE_INTERVAL);
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
imu_init(IMU_t* imu)
{
  // wait 100ms for the sensor to initialize properly
  HAL_Delay(1000);

#if 1
#ifdef USE_QMC5883_MAG
  qmc5883_init(&imu->mag, QMC5883_ADDRESS_MAG);
#else
  hmc5883_init(&imu->mag, HMC5883_ADDRESS_MAG, HMC5883_MAGGAIN_1_3);
#endif
#endif

  imu->accl_off[0]  =
  imu->accl_off[1]  =
  imu->accl_off[2]  = 0;

  imu->accl_scale[0]  =
  imu->accl_scale[1]  =
  imu->accl_scale[2] = 4096;

  imu->gyro_off[0]  =
  imu->gyro_off[1]  =
  imu->gyro_off[2] = 0;

  imu->mag_bias[0]  =
  imu->mag_bias[1]  =
  imu->mag_bias[2] = 0;

  imu->heading_madgwick = 0.0f;
  imu->mag_declination  = 7.68f;

  mpu6050_init(&imu->mpu6050, MPU6050_Accelerometer_8G, MPU6050_Gyroscope_500s);
  mpu6050_set_gyro_dlpf(&imu->mpu6050, 0);   // H/W gyro LPF in 256 Hz

  soft_timer_init_elem(&imu->sampling_timer);
  imu->sampling_timer.cb     = imu_read;
}

void
imu_get_accel(IMU_t* imu, int32_t data[3])
{
  data[0] = imu->accl_value[0];
  data[1] = imu->accl_value[1];
  data[2] = imu->accl_value[2];
}

void
imu_get_gyro(IMU_t* imu, int32_t data[3])
{
  data[0] = imu->gyro_value[0];
  data[1] = imu->gyro_value[1];
  data[2] = imu->gyro_value[2];
}

void
imu_get_mag(IMU_t* imu, int32_t data[3])
{
  data[0] = imu->mag_value[0];
  data[1] = imu->mag_value[1];
  data[2] = imu->mag_value[2];
}

void
imu_get_orientation(IMU_t* imu, float madgwick[4])
{
  madgwick[0] = imu->orientation_madgwick[0];
  madgwick[1] = imu->orientation_madgwick[1];
  madgwick[2] = imu->orientation_madgwick[2];
  madgwick[3] = imu->heading_madgwick;
}

void
imu_start(IMU_t* imu)
{
  madgwick_init(&imu->madgwick_ahrs, IMU_SAMPLE_FREQUENCY);

  mainloop_timer_schedule(&imu->sampling_timer, IMU_SAMPLE_INTERVAL);
}

void
imu_stop(IMU_t* imu)
{
  mainloop_timer_cancel(&imu->sampling_timer);
}

void
imu_set_mag_calib(IMU_t* imu, int16_t x_bias, int16_t y_bias, int16_t z_bias)
{
  imu->mag_bias[0] = x_bias;
  imu->mag_bias[1] = y_bias;
  imu->mag_bias[2] = z_bias;
}

void
imu_get_offset(IMU_t* imu, int16_t gyro[3], int16_t accl[3])
{
  gyro[0] = imu->gyro_off[0];
  gyro[1] = imu->gyro_off[1];
  gyro[2] = imu->gyro_off[2];

  accl[0] = imu->accl_off[0];
  accl[1] = imu->accl_off[1];
  accl[2] = imu->accl_off[2];
}

void
imu_get_mag_calib(IMU_t* imu, int16_t bias[3])
{
  bias[0] = imu->mag_bias[0];
  bias[1] = imu->mag_bias[1];
  bias[2] = imu->mag_bias[2];
}

IMU_t*
imu_get_instance(int ndx)
{
  static IMU_t      _imu0;

  return &_imu0;
}

void
imu_set_gyro_calib(IMU_t* imu, int16_t gx, int16_t gy, int16_t gz)
{
  imu->gyro_off[0] = gx;
  imu->gyro_off[1] = gy;
  imu->gyro_off[2] = gz;
}

void
imu_set_accel_calib(IMU_t* imu, int16_t ax, int16_t ay, int16_t az,
                                int16_t sx, int16_t sy, int16_t sz)
{
  imu->accl_off[0] = ax;
  imu->accl_off[1] = ay;
  imu->accl_off[2] = az;

  imu->accl_scale[0] = sx;
  imu->accl_scale[1] = sy;
  imu->accl_scale[2] = sz;
}
