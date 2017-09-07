#include <math.h>

#include "imu.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////

#define IMU_SAMPLE_INTERVAL                 4         // mms. 200 Hz
#define IMU_SAMPLE_FREQUENCY                (1000.0f/IMU_SAMPLE_INTERVAL)

#define TO_DEGREE   (180 / M_PI)


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
static void
ahrs_init(IMU_t* imu)
{
  madgwick_init(&imu->madgwick_ahrs, IMU_SAMPLE_FREQUENCY);
  mahony_init(&imu->mahony_ahrs, IMU_SAMPLE_FREQUENCY);
}

static void
ahrs_update(IMU_t* imu)
{
  float     gx, gy, gz;
  float     ax, ay, az;
  float     mx, my, mz;

  gx =  (imu->mpu6050.Gyroscope_X - imu->gyro_off[0]) * imu->mpu6050.Gyro_Mult;
  gy =  (imu->mpu6050.Gyroscope_Y - imu->gyro_off[1]) * imu->mpu6050.Gyro_Mult;
  gz =  (imu->mpu6050.Gyroscope_Z - imu->gyro_off[2]) * imu->mpu6050.Gyro_Mult;

  ax =  (imu->mpu6050.Accelerometer_X - imu->accl_off[0]) * imu->mpu6050.Acce_Mult;
  ay =  (imu->mpu6050.Accelerometer_Y - imu->accl_off[1]) * imu->mpu6050.Acce_Mult;
  az =  (imu->mpu6050.Accelerometer_Z - imu->accl_off[2]) * imu->mpu6050.Acce_Mult;

  //
  // https://edwardmallon.wordpress.com/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
  //
  // CalibratedData = ( unCalibratedData – Offset ) / Scaling Factor
  //
#if 0
  mx = (imu->mag_scale[0] * (imu->mag.rx - imu->mag_bias[0] )) * imu->mag.multi_factor;
  my = (imu->mag_scale[1] * (imu->mag.ry - imu->mag_bias[1] )) * imu->mag.multi_factor;
  mz = (imu->mag_scale[2] * (imu->mag.rz - imu->mag_bias[2] )) * imu->mag.multi_factor;
#else
  mx = ((imu->mag.rx - imu->mag_bias[0]) / imu->mag_scale[0]) * imu->mag.multi_factor;
  my = ((imu->mag.ry - imu->mag_bias[1]) / imu->mag_scale[1]) * imu->mag.multi_factor;
  mz = ((imu->mag.rz - imu->mag_bias[2]) / imu->mag_scale[2]) * imu->mag.multi_factor;
#endif

  madgwick_update(&imu->madgwick_ahrs,
      gx, gy, gz,       // gyro
      ax, ay, az,       // accel
      mx, my, mz        // mag
  );

  mahony_update(&imu->mahony_ahrs,
      gx, gy, gz,       // gyro
      ax, ay, az,       // accel
      mx, my, mz        // mag
  );
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

#ifdef USE_QMC5883_MAG
  qmc5883_read(&imu->mag);
#else
  hmc5883_read(&imu->mag);
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
  // wait 10ms for the sensor to initialize properly
  HAL_Delay(10);

#ifdef USE_QMC5883_MAG
  qmc5883_init(&imu->mag, QMC5883_ADDRESS_MAG);
#else
  hmc5883_init(&imu->mag, HMC5883_ADDRESS_MAG, HMC5883_MAGGAIN_1_3);
#endif

  mpu6050_init(&imu->mpu6050, MPU6050_Accelerometer_8G, MPU6050_Gyroscope_500s);
  mpu6050_set_gyro_dlpf(&imu->mpu6050, 0);   // H/W gyro LPF in 256 Hz

  soft_timer_init_elem(&imu->sampling_timer);
  imu->sampling_timer.cb     = imu_read;

  ahrs_init(imu);
}

void
imu_get_mag(IMU_t* imu, float data[4])
{
  float h,
        gx,
        gy,
        gz;

  gx = imu->mag.rx * imu->mag.multi_factor;
  gy = imu->mag.ry * imu->mag.multi_factor;
  gz = imu->mag.rz * imu->mag.multi_factor;

  data[0] = gx;
  data[1] = gy;
  data[2] = gz;

  h = atan2f(gy, gx);
  if(h < 0)
  {
    h += 2 * M_PI;
  }

  if(h > 2 * M_PI)
  {
    h -= 2 * M_PI;
  }

  //
  // magnetic declination here is -7.68f
  // final data[3] is in units of degree ( just like compass )
  // 
  data[3] =  h * TO_DEGREE - 7.68f;
}

void
imu_get_accel(IMU_t* imu, int16_t data[3], float fdata[3])
{
  data[0] = imu->mpu6050.Accelerometer_X;
  data[1] = imu->mpu6050.Accelerometer_Y;
  data[2] = imu->mpu6050.Accelerometer_Z;

  //
  // unit here is G.
  // that is, 1 = 1G
  // 
  fdata[0] = imu->mpu6050.Accelerometer_X * imu->mpu6050.Acce_Mult;
  fdata[1] = imu->mpu6050.Accelerometer_Y * imu->mpu6050.Acce_Mult;
  fdata[2] = imu->mpu6050.Accelerometer_Z * imu->mpu6050.Acce_Mult;
}

void
imu_get_gyro(IMU_t* imu, int16_t data[3], float fdata[3])
{
  data[0] = imu->mpu6050.Gyroscope_X;
  data[1] = imu->mpu6050.Gyroscope_Y;
  data[2] = imu->mpu6050.Gyroscope_Z;

  //
  // unit here is degree/sec
  // ŧhat is, 1 = 1 degree/sec
  // 
  fdata[0] = imu->mpu6050.Gyroscope_X * imu->mpu6050.Gyro_Mult;
  fdata[1] = imu->mpu6050.Gyroscope_Y * imu->mpu6050.Gyro_Mult;
  fdata[2] = imu->mpu6050.Gyroscope_Z * imu->mpu6050.Gyro_Mult;
}

void
imu_get_pitch_roll_yaw(IMU_t* imu, float mahony[3], float madgwick[3])
{
  mahony_get_pitch_roll_yaw(&imu->mahony_ahrs, mahony);
  madgwick_get_pitch_roll_yaw(&imu->madgwick_ahrs, madgwick);
}

void
imu_start(IMU_t* imu)
{
  mainloop_timer_schedule(&imu->sampling_timer, IMU_SAMPLE_INTERVAL);
}

void
imu_stop(IMU_t* imu)
{
  mainloop_timer_cancel(&imu->sampling_timer);
}

void
imu_set_offset(IMU_t* imu,
    int16_t gx, int16_t gy, int16_t gz,
    int16_t ax, int16_t ay, int16_t az)
{
  imu->gyro_off[0] = gx;
  imu->gyro_off[1] = gy;
  imu->gyro_off[2] = gz;

  imu->accl_off[0] = ax;
  imu->accl_off[1] = ay;
  imu->accl_off[2] = az;
}

void
imu_set_mag_calib(IMU_t* imu,
    int16_t x_bias, int16_t y_bias, int16_t z_bias,
    float x_scale, float y_scale, float z_scale)
{
  imu->mag_bias[0] = x_bias;
  imu->mag_bias[1] = y_bias;
  imu->mag_bias[2] = z_bias;

  imu->mag_scale[0] = x_scale;
  imu->mag_scale[1] = y_scale;
  imu->mag_scale[2] = z_scale;
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
imu_get_mag_calib(IMU_t* imu, int16_t bias[3], float scale[3])
{
  bias[0] = imu->mag_bias[0];
  bias[1] = imu->mag_bias[1];
  bias[2] = imu->mag_bias[2];

  scale[0] = imu->mag_scale[0];
  scale[1] = imu->mag_scale[1];
  scale[2] = imu->mag_scale[2];
}

IMU_t*
imu_get_instance(int ndx)
{
  static IMU_t      _imu0;

  return &_imu0;
}
