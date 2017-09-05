#include <math.h>

#include "app_common.h"
#include "qmc5883.h"
#include "hmc5883.h"
#include "mpu6050.h"
#include "imu.h"
#include "mainloop_timer.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////

#define IMU_SAMPLE_INTERVAL                 4         // mms. 200 Hz
#define IMU_SAMPLE_FREQUENCY                (1000/IMU_SAMPLE_INTERVAL)

#define TO_DEGREE   (180 / M_PI)
#define WEIGHT_G    (0.93)
#define WEIGHT_A    (1.00 - WEIGHT_G)

#define THRESHOLD_FACTOR        0.0001f

#define IMU_CALIBRATION_SAMPLE_DISCARD      40

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static SoftTimerElem      _sampling_timer;

#ifdef USE_QMC5883_MAG
static qmc5883Mag         _mag;
#else
static hmc5883Mag         _mag;
#endif
static MPU6050_t          _mpu6050;       // mpu6050 core

////////////////////////////////////////////////////////////////////////////////
//
// sampling timer callback
//
////////////////////////////////////////////////////////////////////////////////
static void
imu_read(SoftTimerElem* te)
{
  mpu6050_read_all(&_mpu6050);
#ifdef USE_QMC5883_MAG
  qmc5883_read(&_mag);
#else
  hmc5883_read(&_mag);
#endif

  mainloop_timer_schedule(&_sampling_timer, IMU_SAMPLE_INTERVAL);
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
imu_init(void)
{
  // wait 10ms for the sensor to initialize properly
  HAL_Delay(10);

#ifdef USE_QMC5883_MAG
  qmc5883_init(&_mag, QMC5883_ADDRESS_MAG);
#else
  hmc5883_init(&_mag, HMC5883_ADDRESS_MAG, HMC5883_MAGGAIN_1_3);
#endif

  mpu6050_init(&_mpu6050, MPU6050_Accelerometer_8G, MPU6050_Gyroscope_500s);
  mpu6050_set_gyro_dlpf(&_mpu6050, 0);   // H/W gyro LPF in 256 Hz

  soft_timer_init_elem(&_sampling_timer);
  _sampling_timer.cb     = imu_read;

  mainloop_timer_schedule(&_sampling_timer, IMU_SAMPLE_INTERVAL);
}

void
imu_get_mag(float data[4])
{
  float h,
        gx,
        gy,
        gz;

  gx = _mag.rx * _mag.multi_factor;
  gy = _mag.ry * _mag.multi_factor;
  gz = _mag.rz * _mag.multi_factor;

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
imu_get_accel(int16_t data[3], float fdata[3])
{
  data[0] = _mpu6050.Accelerometer_X;
  data[1] = _mpu6050.Accelerometer_Y;
  data[2] = _mpu6050.Accelerometer_Z;

  //
  // unit here is G.
  // that is, 1 = 1G
  // 
  fdata[0] = _mpu6050.Accelerometer_X * _mpu6050.Acce_Mult;
  fdata[1] = _mpu6050.Accelerometer_Y * _mpu6050.Acce_Mult;
  fdata[2] = _mpu6050.Accelerometer_Z * _mpu6050.Acce_Mult;
}

void
imu_get_gyro(int16_t data[3], float fdata[3])
{
  data[0] = _mpu6050.Gyroscope_X;
  data[1] = _mpu6050.Gyroscope_Y;
  data[2] = _mpu6050.Gyroscope_Z;

  //
  // unit here is degree/sec
  // ŧhat is, 1 = 1 degree/sec
  // 
  fdata[0] = _mpu6050.Gyroscope_X * _mpu6050.Gyro_Mult;
  fdata[1] = _mpu6050.Gyroscope_Y * _mpu6050.Gyro_Mult;
  fdata[2] = _mpu6050.Gyroscope_Z * _mpu6050.Gyro_Mult;
}
