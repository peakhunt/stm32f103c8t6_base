#include <stdio.h>

#include "stm32f1xx_hal.h"
#include "app_common.h"
#include "app.h"
#include "event_dispatcher.h"
#include "mainloop_timer.h"
#include "blinky.h"
#include "shell.h"
#include "micros.h"
#include "pwm_out.h"
#include "pwm_in.h"
#include "i2c_bus.h"
#include "ws2812b.h"
#include "imu.h"
#include "barometer.h"

void
app_init_f(void)
{
  event_dispatcher_init();
  mainloop_timer_init();
  blinky_init();
}

void
app_init_r(void)
{
  micros_init();
  //pwm_out_init();
  //pwm_in_init();

  i2c_bus_init();
  __disable_irq();
  shell_init();
  __enable_irq();

#ifdef __ENABLE_WS2812B
  ws2812b_init();
#endif

#ifdef __ENABLE_IMU
  imu_init(imu_get_instance(0));
  imu_start(imu_get_instance(0));
  //barometer_init();
#endif
}

void
app_mainloop(void)
{
  while(1)
  {
    event_dispatcher_dispatch();
  }
}
