#include <math.h>
#include <stdlib.h>
#include "accel_calibration.h"
#include "sensor_calib.h"
#include "shell.h"

/*
   As for accelerometer, things are not that simple. The purpose of accelerometer calibration
   is to find offset and scale for the sensor so that actual accelerometer value is
   calculated using the following fomula.
      actual = (raw - offset) * scale
   To calculate offset/scale, we use a typical 6 axis sampling/calibration mechanism.
*/

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////
#define ACCEL_CALIB_SAMPLE_INTERVAL           2           // sample every 10ms
#define ACCEL_CALIB_TOTAL_PERIOD              10000       // 10 sec
#define ACCEL_1G                              (512 * 8)

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static SoftTimerElem        _sampling_timer;

static int32_t              _sample_count[6];

static int32_t              _acc_sum[6][3];

static int32_t              _offset[3];
static int32_t              _gain[3];
static int32_t              _current_axis_ndx;

static MPU6050_t*           _accel;

static void (*_calib_done_cb)(int axis_ndx, void*);
static void* _calib_cb_arg;

static sensor_calib_t       _cal_state;

////////////////////////////////////////////////////////////////////////////////
//
// calibration routines
//
////////////////////////////////////////////////////////////////////////////////
static int
getPrimaryAxisIndex(int32_t sample[3])
{
  const int x = 0,
            y = 1,
            z = 2;

  // tolerate up to atan(1 / 1.5) = 33 deg tilt (in worst case 66 deg separation between points)
  if ((abs(sample[z]) / 1.5f) > abs(sample[x]) && (abs(sample[z]) / 1.5f) > abs(sample[y])) {
    //z-axis
    return (sample[z] > 0) ? 0 : 1;
  }
  else if ((abs(sample[x]) / 1.5f) > abs(sample[y]) && (abs(sample[x]) / 1.5f) > abs(sample[z])) {
    //x-axis
    return (sample[x] > 0) ? 2 : 3;
  }
  else if ((abs(sample[y]) / 1.5f) > abs(sample[x]) && (abs(sample[y]) / 1.5f) > abs(sample[z])) {
    //y-axis
    return (sample[y] > 0) ? 4 : 5;
  }
  else
    return -1;
}

static void
accel_calib_accel_read(SoftTimerElem* te)
{
  int32_t   accel_value[3];
  int       axis_ndx;

  mpu6050_read_all(_accel);

  accel_value[0] = _accel->Accelerometer_X;
  accel_value[1] = _accel->Accelerometer_Y;
  accel_value[2] = _accel->Accelerometer_Z;

  axis_ndx = getPrimaryAxisIndex(accel_value);

  if (axis_ndx < 0 || (_current_axis_ndx != -1 && axis_ndx != _current_axis_ndx))
  {
    return;
  }

  if(_current_axis_ndx == -1)
  {
    _acc_sum[axis_ndx][0] = 0;
    _acc_sum[axis_ndx][1] = 0;
    _acc_sum[axis_ndx][2] = 0;

    _sample_count[axis_ndx] = 0;
  }

  _current_axis_ndx = axis_ndx;

  sensorCalibrationPushSampleForOffsetCalculation(&_cal_state, accel_value);

  _acc_sum[axis_ndx][0] += accel_value[0];
  _acc_sum[axis_ndx][1] += accel_value[1];
  _acc_sum[axis_ndx][2] += accel_value[2];

  _sample_count[axis_ndx]++;

  // FIXME
  if(_sample_count[axis_ndx] >= 400)
  {
    if(_calib_done_cb != NULL)
    {
      _calib_done_cb(_current_axis_ndx, _calib_cb_arg);
    }
    return;
  }

  mainloop_timer_schedule(&_sampling_timer, ACCEL_CALIB_SAMPLE_INTERVAL);
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
accel_calib_init(void)
{
  soft_timer_init_elem(&_sampling_timer);

  for(int i = 0; i < 6; i++)
  {
    _acc_sum[i][0] = 0;
    _acc_sum[i][1] = 0;
    _acc_sum[i][2] = 0;

    _sample_count[i] = 0;
  }

  sensorCalibrationResetState(&_cal_state);
}

void
accel_calib_perform(MPU6050_t* accel, void (*calib_done)(int, void*), void* arg)
{
  _current_axis_ndx = -1;

  _calib_done_cb  = calib_done;
  _calib_cb_arg   = arg;

  _accel    = accel;

  soft_timer_init_elem(&_sampling_timer);
  _sampling_timer.cb      = accel_calib_accel_read;

  mainloop_timer_schedule(&_sampling_timer, ACCEL_CALIB_SAMPLE_INTERVAL);
}

void
accel_calib_calculate(void)
{
  float   tmp[3];
  int32_t sample[3];

  /* calculate offset */
  sensorCalibrationSolveForOffset(&_cal_state, tmp);

  // shell_out("DEBUG offset %f, %f, %f\r\n",  tmp[0], tmp[1], tmp[2]);

  for(int i = 0; i < 3; i++)
  {
    _offset[i] = lrintf(tmp[i]);
  }

  /* Not we can offset our accumulated averages samples and calculate scale factors and calculate gains */
  sensorCalibrationResetState(&_cal_state);

  for (int axis = 0; axis < 6; axis++) {
    sample[0] = _acc_sum[axis][0] / _sample_count[axis] - _offset[0];
    sample[1] = _acc_sum[axis][1] / _sample_count[axis] - _offset[1];
    sample[2] = _acc_sum[axis][2] / _sample_count[axis] - _offset[2];
    
    // shell_out("DEBUG axis: %d, sample %ld, %ld, %ld\r\n", axis, sample[0], sample[1], sample[2]);

    sensorCalibrationPushSampleForScaleCalculation(&_cal_state, axis / 2, sample, ACCEL_1G);
  }

  sensorCalibrationSolveForScale(&_cal_state, tmp);

  for (int axis = 0; axis < 3; axis++)
  {
    _gain[axis] = lrintf(tmp[axis] * 4096);

    // shell_out("DEBUG %d, %f, %ld\r\n", axis, tmp[axis], _gain[axis]);
  }
}

void
accel_calib_get_offset_gain(int32_t* offs, int32_t* gain)
{
  for(int i = 0; i < 3; i++)
  {
    offs[i] = _offset[i];
    gain[i] = _gain[i];
  }
}

void
accel_get_acc_sum(int32_t sum[6][3], int32_t count[6])
{
  for(int i = 0; i < 6; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      sum[i][j] = _acc_sum[i][j];
    }
    count[i] = _sample_count[i];
  }
}
