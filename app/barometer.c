#include "app_common.h"
#include "barometer.h"
#include "mainloop_timer.h"
#include "bmp180.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////
#define BAROMETER_MONITOR_INTERVAL      100     // 100ms
#define BAROMETER_PRESSURE_OVERSAMPLE   3

extern int8_t BMP180_I2C_routine(struct bmp180_t* bmp180);

////////////////////////////////////////////////////////////////////////////////
//
// private prototypes
//
////////////////////////////////////////////////////////////////////////////////
static void barometer_read_pressure_callback(uint32_t pressure);
static void barometer_read_temperature_callback(uint32_t temperature);

static void barometer_read_start_again(SoftTimerElem* te);

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static SoftTimerElem      _baro_timer;
static int32_t            _temperature      = 0;
static int32_t            _pressure         = 0;
static int32_t            _pressure_sum     = 0;
static uint8_t            _baro_oversample  = 0;

static struct bmp180_t    _bmp180;

////////////////////////////////////////////////////////////////////////////////
//
// barometer read logic
//
////////////////////////////////////////////////////////////////////////////////
static void
barometer_read_pressure_callback(uint32_t pressure)
{
  //
  // handle pressure
  //
  _pressure_sum += bmp180_get_pressure(pressure);
  _baro_oversample++;

  if(_baro_oversample < BAROMETER_PRESSURE_OVERSAMPLE)
  {
    bmp180_get_uncomp_pressure_async(barometer_read_pressure_callback);
  }
  else
  {
    //
    // done. schedule next measure
    //
    _pressure = _pressure_sum / _baro_oversample;

    mainloop_timer_schedule(&_baro_timer, BAROMETER_MONITOR_INTERVAL);
  }
}

static void
barometer_read_temperature_callback(uint32_t temperature)
{
  //
  // handle temperature
  //
  _temperature = bmp180_get_temperature(temperature);

  //
  // initiate pressure reading
  //
  bmp180_get_uncomp_pressure_async(barometer_read_pressure_callback);
}

static void
barometer_read_start_again(SoftTimerElem* te)
{
  _baro_oversample    = 0;
  _pressure_sum       = 0;

  bmp180_get_uncomp_temperature_async(barometer_read_temperature_callback);
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
barometer_init(void)
{
  BMP180_I2C_routine(&_bmp180);
  bmp180_init(&_bmp180);

  soft_timer_init_elem(&_baro_timer);

  _baro_timer.cb    = barometer_read_start_again;
  mainloop_timer_schedule(&_baro_timer, BAROMETER_MONITOR_INTERVAL);
}

int32_t
barometer_get_temperature(void)
{
  return _temperature;
}

int32_t
barometer_get_pressure(void)
{
  return _pressure;
}
