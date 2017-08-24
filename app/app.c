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
#include "i2c_bus.h"
#include "ws2812b.h"

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
  pwm_out_init();

  i2c_bus_init();
  __disable_irq();
  shell_init();
  __enable_irq();

  ws2812b_init();
}

void
app_mainloop(void)
{
  while(1)
  {
    event_dispatcher_dispatch();
  }
}
