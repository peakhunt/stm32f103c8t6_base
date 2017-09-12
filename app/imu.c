#include <math.h>

#include "imu.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////

#define IMU_SAMPLE_INTERVAL                 5         // mms. 200 Hz
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
static void
ahrs_init(IMU_t* imu)
{
  madgwick_init(&imu->madgwick_ahrs, IMU_SAMPLE_FREQUENCY);
  mahony_init(&imu->mahony_ahrs, IMU_SAMPLE_FREQUENCY);
}

static float
ahrs_calc_heading(float mx, float my, float mz, float roll, float pitch)
{
  //
  // fro en.DM00269987.pdf
  //
  //
  // Phi    : roll
  // Theta  : pitch
  // Psi    : Yaw
  //
  // By2 = Bz * Sin(Phy) - By * Cos(Phi)
  // Bz2 = By * Sin(Phi) + Bz * Cos(Phi)
  // Bx3 = Bx * Cos(Theta) + Bz2 * Sin(Theta)
  //
  // Yaw: Psi = Atan2(By2, Bx3)
  //          = Atan2(Bz * Sin(Phy) - By * Cos(Phi),
  //                  Bx * Cos(Theta) + 
  //                  (By * Sin(Phi) + Bz * Cos(Phi)) * Sin(Theta))
  //          = Atan2(Bz * Sin(Phi) - By * Cos(Phi),
  //                  Bx * Cos(Theta) + 
  //                  By * Sin(Phi) * Sin(Theta) +
  //                  Bz * Cos(Phi) * Sin(Theta))
  //
  //          = Atan2(mz * sin(roll) - my * cos(roll),
  //                  mx * cos(pitch) +
  //                  my * sin(roll) * sin(pitch) +
  //                  mz * cos(roll) * sin(pitch))
  //
  float   heading;

  heading = (float)atan2(mz * sin(roll * TO_RADIAN) - my * cos(roll * TO_RADIAN),
                         mx * cos(pitch * TO_RADIAN) + my * sin(pitch * TO_RADIAN) * sin(roll * TO_RADIAN) + 
                         mz * sin(pitch * TO_RADIAN) * cos(roll * TO_RADIAN));

  if(heading < 0)
  {
    heading += 2 * M_PI;
  }

  return heading * TO_DEGREE;
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
  // https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
  // ħttp://diydrones.com/profiles/blogs/advanced-hard-and-soft-iron-magnetometer-calibration-for-dummies
  //
  // CalibratedData = ( unCalibratedData – Offset ) / Scaling Factor
  //
  mx = ((imu->mag.rx - imu->mag_bias[0] ) / imu->mag_scale[0]) / imu->mag.Gauss_LSB_XY; 
  my = ((imu->mag.ry - imu->mag_bias[1] ) / imu->mag_scale[1]) / imu->mag.Gauss_LSB_XY;
  mz = ((imu->mag.rz - imu->mag_bias[2] ) / imu->mag_scale[2]) / imu->mag.Gauss_LSB_Z;

  madgwick_update(&imu->madgwick_ahrs,
      gx, gy, gz,       // gyro
      ax, ay, az,       // accel
      my,-mx, mz        // mag
  );

  mahony_update(&imu->mahony_ahrs,
      gx, gy, gz,       // gyro
      ax, ay, az,       // accel
      my,-mx, mz        // mag
  );

  mahony_get_roll_pitch_yaw(&imu->mahony_ahrs,      imu->orientation_mahony);
  madgwick_get_roll_pitch_yaw(&imu->madgwick_ahrs,  imu->orientation_madgwick);

  imu->heading_madgwick = ahrs_calc_heading( my,-mx, mz, imu->orientation_madgwick[0], imu->orientation_madgwick[1]) -
    imu->mag_declination;
  imu->heading_mahony   = ahrs_calc_heading( my,-mx, mz, imu->orientation_mahony[0], imu->orientation_mahony[1]) -
    imu->mag_declination;

  imu->heading_madgwick -= 360.0f;
  imu->heading_mahony   -= 360.0f;

  imu->heading_madgwick *= -1.0f;
  imu->heading_mahony   *= -1.0f;

  if(imu->heading_madgwick >= 360.0f)
  {
    imu->heading_madgwick -= 360.0f;
  }

  if(imu->heading_mahony >= 360.0f)
  {
    imu->heading_mahony -= 360.0f;
  }

  //
  // heading compensation due to board aignment
  //
  // XXX FIXME
  
  // due to board alignment
  imu->heading_raw      = atan2f(-mx, my);
  if(imu->heading_raw < 0)
  {
    imu->heading_raw += 2 * M_PI;
  }

  if(imu->heading_raw > 2 * M_PI)
  {
    imu->heading_raw -= 2 * M_PI;
  }

  // 7.68f is magnetic declination
  imu->heading_raw = imu->heading_raw * TO_DEGREE - imu->mag_declination;
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
  // wait 100ms for the sensor to initialize properly
  HAL_Delay(300);

#ifdef USE_QMC5883_MAG
  qmc5883_init(&imu->mag, QMC5883_ADDRESS_MAG);
#else
  hmc5883_init(&imu->mag, HMC5883_ADDRESS_MAG, HMC5883_MAGGAIN_1_3);
#endif

  imu->accl_off[0]  = imu->accl_off[1]  = imu->accl_off[2] = 0;
  imu->gyro_off[0]  = imu->gyro_off[1]  = imu->gyro_off[2] = 0;
  imu->mag_bias[0]  = imu->mag_bias[1]  = imu->mag_bias[2] = 0;
  imu->mag_scale[0] = imu->mag_scale[1] = imu->mag_scale[2] = 1.0f;

  imu->heading_mahony   = 0.0f;
  imu->heading_madgwick = 0.0f;
  imu->mag_declination  = 7.68f;

  mpu6050_init(&imu->mpu6050, MPU6050_Accelerometer_8G, MPU6050_Gyroscope_500s);
  mpu6050_set_gyro_dlpf(&imu->mpu6050, 0);   // H/W gyro LPF in 256 Hz

  soft_timer_init_elem(&imu->sampling_timer);
  imu->sampling_timer.cb     = imu_read;

  ahrs_init(imu);
}

void
imu_get_mag(IMU_t* imu, float data[4])
{
  float gx,
        gy,
        gz;

  gx = ((imu->mag.rx - imu->mag_bias[0] ) / imu->mag_scale[0]) / imu->mag.Gauss_LSB_XY; 
  gy = ((imu->mag.ry - imu->mag_bias[1] ) / imu->mag_scale[1]) / imu->mag.Gauss_LSB_XY;
  gz = ((imu->mag.rz - imu->mag_bias[2] ) / imu->mag_scale[2]) / imu->mag.Gauss_LSB_Z;

  data[0] = gx;
  data[1] = gy;
  data[2] = gz;
  data[3] = imu->heading_raw;
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
imu_get_orientation(IMU_t* imu, float mahony[4], float madgwick[4])
{
  mahony[0] = imu->orientation_mahony[0];
  mahony[1] = imu->orientation_mahony[1];
  mahony[2] = imu->orientation_mahony[2];
  mahony[3] = imu->heading_mahony;

  madgwick[0] = imu->orientation_madgwick[0];
  madgwick[1] = imu->orientation_madgwick[1];
  madgwick[2] = imu->orientation_madgwick[2];
  madgwick[3] = imu->heading_madgwick;
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
