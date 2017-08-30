#include <math.h>

#include "app_common.h"
#include "qmc5883.h"
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
static qmc5883Mag         _mag;

////////////////////////////////////////////////////////////////////////////////
//
// sampling timer callback
//
////////////////////////////////////////////////////////////////////////////////
static void
imu_read(SoftTimerElem* te)
{
  qmc5883_read(&_mag);
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
  qmc5883_init(&_mag, QMC5883_ADDRESS_MAG);

  soft_timer_init_elem(&_sampling_timer);
  _sampling_timer.cb     = imu_read;

  mainloop_timer_schedule(&_sampling_timer, IMU_SAMPLE_INTERVAL);
}

void
imu_get_mag(float data[4])
{
  float h;

  data[0] = _mag.gx;
  data[1] = _mag.gy;
  data[2] = _mag.gz;

  h = atan2f(_mag.gy, _mag.gx);
  if(h < 0)
  {
    h += 2 * M_PI;
  }

  if(h > 2 * M_PI)
  {
    h -= 2 * M_PI;
  }

  // magnetic declination here is -7.68f
  data[3] =  h * TO_DEGREE - 7.68f;
}
