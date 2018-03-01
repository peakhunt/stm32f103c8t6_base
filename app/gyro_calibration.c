#include <math.h>
#include "gyro_calibration.h"

/*
   As for gyro, all we can do is zero calibration. That is,
   when the board stands still, we should get zero values for all the axises from gyro.
   But in reality, those values will be slightly bigger/smaller than zero due to various
   errors. So all we gotta do is calculate the offset values for zero position and save them.
   Later after calibration, those offset values will be simply subtracted from real sample
   values to produce more correct value.
   Offset can be calculated simply by off = (sum of all samples) / num_samples
*/

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////
#define GYRO_CALIB_SAMPLE_INTERVAL          2       // sample every 4ms
#define GYRO_CALIB_TOTAL_PERIOD             10000   // 10 sec

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static SoftTimerElem        _sampling_timer;
static SoftTimerElem        _calib_timer;

static uint32_t             _sample_count;

static float                _gx_sum,
                            _gy_sum,
                            _gz_sum;

static int16_t              _gx_off,
                            _gy_off,
                            _gz_off;

static MPU6050_t*           _gyro;

static void (*_calib_done_cb)(void*);
static void* _calib_cb_arg;

////////////////////////////////////////////////////////////////////////////////
//
// calibration routines
//
////////////////////////////////////////////////////////////////////////////////
static void
gyro_calib_gyro_read(SoftTimerElem* te)
{
  mpu6050_read_all(_gyro);

  // gyro
  _gx_sum += _gyro->Gyroscope_X;
  _gy_sum += _gyro->Gyroscope_Y;
  _gz_sum += _gyro->Gyroscope_Z;

  _sample_count++;

  mainloop_timer_schedule(&_sampling_timer, GYRO_CALIB_SAMPLE_INTERVAL);
}

static void
gyro_calib_gyro_done(SoftTimerElem* te)
{
  mainloop_timer_cancel(&_sampling_timer);

  // gyro
  _gx_off = (int16_t)(_gx_sum / _sample_count);
  _gy_off = (int16_t)(_gy_sum / _sample_count);
  _gz_off = (int16_t)(_gz_sum / _sample_count);

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
gyro_calib_init(void)
{
  soft_timer_init_elem(&_sampling_timer);
  soft_timer_init_elem(&_calib_timer);
}

void
gyro_calib_perform(MPU6050_t* gyro, void (*calib_done)(void*), void* arg)
{
  _gyro           = gyro;

  _calib_done_cb  = calib_done;
  _calib_cb_arg   = arg;

  _sample_count = 0;

  _gx_sum =
  _gy_sum =
  _gz_sum = 0.0f;

  soft_timer_init_elem(&_sampling_timer);
  _sampling_timer.cb      = gyro_calib_gyro_read;

  soft_timer_init_elem(&_calib_timer);
  _calib_timer.cb         = gyro_calib_gyro_done;

  mainloop_timer_schedule(&_sampling_timer, GYRO_CALIB_SAMPLE_INTERVAL);
  mainloop_timer_schedule(&_calib_timer, GYRO_CALIB_TOTAL_PERIOD);
}

void
gyro_calib_get_offset(int16_t* offs)
{
  offs[0] = _gx_off;
  offs[1] = _gy_off;
  offs[2] = _gz_off;
}
