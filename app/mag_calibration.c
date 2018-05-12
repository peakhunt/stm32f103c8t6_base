#include <math.h>
#include <stdlib.h>
#include "mag_calibration.h"
#include "sensor_calib.h"
#include "shell.h"

/*
   As for magnetometer, things are even more complicated. To handle hard-iron error, we sample
   magnetometer while moving it around uniformly in all direction. After sampling, we must
   calculate the magnetometer offset, which shifts sample center off the zero point. This is
   so called simple sphere fitting without rotation and quite efficient for hard-iron error.
   Offset can be calculated simply by off = (max + min) / 2 but this source is using a bit
   sophiscated LR guass algorithm for the offset calculation.

*/

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////
#define MAG_CALIB_SAMPLE_INTERVAL           2           // sample every 10ms
#define MAG_CALIB_TOTAL_PERIOD              30000       // 30 sec

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static SoftTimerElem        _sampling_timer;
static SoftTimerElem        _calib_timer;

static int32_t              _mag_prev[3];
static int32_t              _mag_offset[3];

static sensor_calib_t       _cal_state;

static void (*_calib_done_cb)(void*);
static void* _calib_cb_arg;

static hmc5883Mag*          _mag;

////////////////////////////////////////////////////////////////////////////////
//
// calibration routines
//
////////////////////////////////////////////////////////////////////////////////
static void
mag_calib_mag_read(SoftTimerElem* te)
{
  float     diffMag = 0;
  float     avgMag = 0;
  int32_t   mag_data[3];

  hmc5883_read(_mag);

  mag_data[0] = _mag->rx;
  mag_data[1] = _mag->ry;
  mag_data[2] = _mag->rz;

  for (int axis = 0; axis < 3; axis++) {
    diffMag += (mag_data[axis] - _mag_prev[axis]) * (mag_data[axis] - _mag_prev[axis]);
    avgMag += (mag_data[axis] + _mag_prev[axis]) * (mag_data[axis] + _mag_prev[axis]) / 4.0f;
  }

  // sqrtf(diffMag / avgMag) is a rough approximation of tangent of angle between magADC and _mag_prev. tan(8 deg) = 0.14
  if ((avgMag > 0.01f) && ((diffMag / avgMag) > (0.14f * 0.14f))) {
    sensorCalibrationPushSampleForOffsetCalculation(&_cal_state, mag_data);

    for (int axis = 0; axis < 3; axis++) {
      _mag_prev[axis] = mag_data[axis];
    }
  }

  mainloop_timer_schedule(&_sampling_timer, MAG_CALIB_SAMPLE_INTERVAL);
}

static void
mag_calib_mag_done(SoftTimerElem* te)
{
  float magZerof[3];

  mainloop_timer_cancel(&_sampling_timer);

  sensorCalibrationSolveForOffset(&_cal_state, magZerof);

  for (int axis = 0; axis < 3; axis++) {
    _mag_offset[axis] = lrintf(magZerof[axis]);
  }

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
mag_calib_init(void)
{
  soft_timer_init_elem(&_sampling_timer);
  soft_timer_init_elem(&_calib_timer);

  _mag_prev[0]    =
  _mag_prev[1]    =
  _mag_prev[2]    = 0;

  sensorCalibrationResetState(&_cal_state);
}

void
mag_calib_perform(hmc5883Mag* mag, void (*calib_done)(void*), void* arg)
{
  _mag              = mag;

  _calib_done_cb    = calib_done;
  _calib_cb_arg     = arg;

  soft_timer_init_elem(&_sampling_timer);
  _sampling_timer.cb      = mag_calib_mag_read;

  soft_timer_init_elem(&_calib_timer);
  _calib_timer.cb         = mag_calib_mag_done;

  mainloop_timer_schedule(&_sampling_timer, MAG_CALIB_SAMPLE_INTERVAL);
  mainloop_timer_schedule(&_calib_timer, MAG_CALIB_TOTAL_PERIOD);
}

void
mag_calib_get_offset(int32_t* offs)
{
  offs[0] = _mag_offset[0];
  offs[1] = _mag_offset[1];
  offs[2] = _mag_offset[2];
}
