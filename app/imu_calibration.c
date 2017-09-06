#include <math.h>

#include "app_common.h"
#include "qmc5883.h"
#include "hmc5883.h"
#include "mpu6050.h"
#include "imu.h"
#include "mainloop_timer.h"
#include "imu_calibration.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////
#define IMU_CALIB_SAMPLE_INTERVAL           4       // sample every 4ms
#define IMU_CALIB_TOTAL_PERIOD              5000    // perform calibration for 5 sec

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static SoftTimerElem        _sampling_timer;
static SoftTimerElem        _calib_timer;

static uint32_t           _sample_count;

static float              _gx_sum,
                          _gy_sum,
                          _gz_sum;

static float              _ax_sum,
                          _ay_sum,
                          _az_sum;

static int16_t            _mx_min,
                          _my_min,
                          _mz_min;

static int16_t            _mx_max,
                          _my_max,
                          _mz_max;

static int16_t            _mx_bias,
                          _my_bias,
                          _mz_bias;

static float              _mx_scale,
                          _my_scale,
                          _mz_scale;

static IMU_t*             _imu;

static void (*_calib_done_cb)(void*);
static void* _calib_cb_arg;

static void
imu_calib_accel_read(SoftTimerElem* te)
{
  mpu6050_read_all(&_imu->mpu6050);

  // gyro
  _gx_sum += _imu->mpu6050.Gyroscope_X;
  _gy_sum += _imu->mpu6050.Gyroscope_Y;
  _gz_sum += _imu->mpu6050.Gyroscope_Z;

  // accel
  _ax_sum += _imu->mpu6050.Accelerometer_X;
  _ay_sum += _imu->mpu6050.Accelerometer_Y;
  _az_sum += _imu->mpu6050.Accelerometer_Z;

  _sample_count++;

  mainloop_timer_schedule(&_sampling_timer, IMU_CALIB_SAMPLE_INTERVAL);
}

static void
imu_calib_accel_done(SoftTimerElem* te)
{
  int16_t       gx, gy, gz,
                ax, ay, az;

  mainloop_timer_cancel(&_sampling_timer);

  // gyro
  gx = (int16_t)(_gx_sum / _sample_count);
  gy = (int16_t)(_gy_sum / _sample_count);
  gz = (int16_t)(_gz_sum / _sample_count);

  // accel
  ax = (int16_t)(_ax_sum / _sample_count);
  ay = (int16_t)(_ay_sum / _sample_count);
  az = (int16_t)(_az_sum / _sample_count);

  //
  // just a Note here.
  // The final calibrated mag reading should be calculated as
  // mr = scale * ( raw - bias)
  //

  imu_set_offset(_imu, gx, gy, gz, ax, ay, az);

  imu_start(_imu);

  if(_calib_done_cb != NULL)
  {
    _calib_done_cb(_calib_cb_arg);
  }
}

static void
imu_calib_mag_read(SoftTimerElem* te)
{
#ifdef USE_QMC5883_MAG
  qmc5883_read(&_imu->mag);
#else
  hmc5883_read(&_imu->mag);
#endif

  // mag max
  _mx_max = _imu->mag.rx > _mx_max ? _imu->mag.rx : _mx_max;
  _my_max = _imu->mag.ry > _my_max ? _imu->mag.ry : _my_max;
  _mz_max = _imu->mag.rz > _mz_max ? _imu->mag.rz : _mz_max;

  // mag min
  _mx_min = _imu->mag.rx < _mx_min ? _imu->mag.rx : _mx_min;
  _my_min = _imu->mag.ry < _my_min ? _imu->mag.ry : _my_min;
  _mz_min = _imu->mag.rz < _mz_min ? _imu->mag.rz : _mz_min;
  
  _sample_count++;

  mainloop_timer_schedule(&_sampling_timer, IMU_CALIB_SAMPLE_INTERVAL);
}

static void
imu_calib_mag_done(SoftTimerElem* te)
{
  float         avg_rad;
  float         mx_range,
                my_range,
                mz_range;

  mainloop_timer_cancel(&_sampling_timer);

  // mag
  // hard iron
  _mx_bias = (_mx_max + _mx_min) / 2;
  _my_bias = (_my_max + _my_min) / 2;
  _mz_bias = (_mz_max + _mz_min) / 2;

  // mag
  // soft iron
  mx_range  = (_mx_max - _mx_min) / 2;
  my_range  = (_my_max - _my_min) / 2;
  mz_range  = (_mz_max - _mz_min) / 2;

  avg_rad = (mx_range + my_range + mz_range) / 3.0f;

  _mx_scale = avg_rad / mx_range;
  _my_scale = avg_rad / my_range;
  _mz_scale = avg_rad / mz_range;

  //
  // just a Note here.
  // The final calibrated mag reading should be calculated as
  // mr = scale * ( raw - bias)
  //
  imu_set_mag_calib(_imu, 
      _mx_bias, _my_bias, _mz_bias,
      _mx_scale, _my_scale, _mz_scale);

  imu_start(_imu);

  if(_calib_done_cb != NULL)
  {
    _calib_done_cb(_calib_cb_arg);
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
imu_calibration_init(void)
{
  _imu = imu_get_instance(0);

  soft_timer_init_elem(&_sampling_timer);
  soft_timer_init_elem(&_calib_timer);
}

void
imu_calibration_accel_perform(void (*calib_done)(void*), void* arg)
{
  imu_stop(_imu);
  
  _calib_done_cb  = calib_done;
  _calib_cb_arg   = arg;

  _sample_count = 0;

  _gx_sum =
  _gy_sum =
  _gz_sum = 0.0f;

  _ax_sum =
  _ay_sum =
  _az_sum = 0.0f;

  soft_timer_init_elem(&_sampling_timer);
  _sampling_timer.cb      = imu_calib_accel_read;

  soft_timer_init_elem(&_calib_timer);
  _calib_timer.cb         = imu_calib_accel_done;

  mainloop_timer_schedule(&_sampling_timer, IMU_CALIB_SAMPLE_INTERVAL);
  mainloop_timer_schedule(&_calib_timer, IMU_CALIB_TOTAL_PERIOD);
}

void
imu_calibration_mag_perform(void (*calib_done)(void*), void* arg)
{
  imu_stop(_imu);
  
  _calib_done_cb  = calib_done;
  _calib_cb_arg   = arg;

  _sample_count = 0;

  _mx_min = 
  _my_min = 
  _mz_min = 32767;

  _mx_max = 
  _my_max = 
  _mz_max = -32767;

  soft_timer_init_elem(&_sampling_timer);
  _sampling_timer.cb      = imu_calib_mag_read;

  soft_timer_init_elem(&_calib_timer);
  _calib_timer.cb         = imu_calib_mag_done;

  mainloop_timer_schedule(&_sampling_timer, IMU_CALIB_SAMPLE_INTERVAL);
  mainloop_timer_schedule(&_calib_timer, IMU_CALIB_TOTAL_PERIOD);
}
