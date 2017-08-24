#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f1xx_hal.h"
#include "tim.h"

#include "app_common.h"
#include "ws2812b.h"
#include "mainloop_timer.h"

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static TIM_HandleTypeDef*     _htim         = &htim2;
static uint32_t               _pwm_channel  = TIM_CHANNEL_1;

//
// one led takes up 3 bytes.
// For DMA control over PWM, 1 byte takes 8 bytes
//
// last 1 byte contains 0, which has an effect of pulling down pwm
// signal. As long as refresh late is bigger than 1ms, everything should
// be ok with this simple scheme
// 
static uint8_t  _dma_buffer[WS2812B_MAX_LED * 3 * 8 + 1];
static uint8_t  _color_buffer[WS2812B_MAX_LED * 3];

static SoftTimerElem    _ws2812b_refresh_timer;

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
static void
ws2812b_fill_dma_buffer(void)
{
  uint8_t   ndx,
            bit_ndx;
  uint32_t  dma_ndx = 0;

  for(ndx = 0; ndx < WS2812B_MAX_LED * 3; ndx++)
  {
    for(bit_ndx = 0; bit_ndx < 8; bit_ndx++)
    {
      if((_color_buffer[ndx] << bit_ndx) & 0x80)
      {
        // high
        _dma_buffer[dma_ndx] = WS2812B_HIGH_DUTY_CYCLE;
      }
      else
      {
        // low
        _dma_buffer[dma_ndx] = WS2812B_LOW_DUTY_CYCLE;
      }
      dma_ndx += 1;
    }
  }
}

static void
ws2812b_start_transmission(void)
{
  HAL_TIM_PWM_Start_DMA(_htim,
      _pwm_channel,
      (uint32_t*)_dma_buffer,
      (uint16_t)sizeof(_dma_buffer));
}

static void
ws2812b_refresh_callback(SoftTimerElem* te)
{
  ws2812b_fill_dma_buffer();
  ws2812b_start_transmission();
}

void
ws2812b_dma_complete_callback(void)
{
  HAL_TIM_PWM_Stop_DMA(_htim, _pwm_channel);
  mainloop_timer_schedule(&_ws2812b_refresh_timer, WS2812B_UPDATE_INTERVAL);
}

////////////////////////////////////////////////////////////////////////////////
//
// debug test
//
////////////////////////////////////////////////////////////////////////////////
static SoftTimerElem    _debug_timer;

/*
static void
do_debug_test(void)
{
  static uint8_t    cnt = 0;

  switch(cnt)
  {
  case 0:
    ws2812b_update_color(0, 0x0000ff00);
    ws2812b_update_color(1, 0x00ff0000);
    ws2812b_update_color(2, 0x000000ff);
    cnt = 1;
    break;

  case 1:
    ws2812b_update_color(1, 0x0000ff00);
    ws2812b_update_color(2, 0x00ff0000);
    ws2812b_update_color(0, 0x000000ff);
    cnt = 2;
    break;

  case 2:
    ws2812b_update_color(2, 0x0000ff00);
    ws2812b_update_color(0, 0x00ff0000);
    ws2812b_update_color(1, 0x000000ff);
    cnt = 0;
    break;
  }

  mainloop_timer_schedule(&_debug_timer, 400);
}
*/
static void
do_debug_test(void)
{
  static int32_t    color = 0,
                    dir = 0;

  if(dir == 0)
  {
    color += 5;
  }
  else
  {
    color -= 5;
  }

  if(color > 255)
  {
    color = 255;
  }

  if(color < 0)
  {
    color = 0;
  }

  ws2812b_update_color(0, 0x0000ff00 & (color << 8));
  ws2812b_update_color(2, 0x00ff0000 & (color << 16));
  ws2812b_update_color(1, 0x000000ff & (color << 0));

  if(color == 255)
  {
    dir = 1;
  }

  if(color == 0)
  {
    dir = 0;
  }

  mainloop_timer_schedule(&_debug_timer, 20);
}

static void
debug_test_timer_callback(SoftTimerElem* te)
{
  do_debug_test();
}


////////////////////////////////////////////////////////////////////////////////
//
// public interface
//
////////////////////////////////////////////////////////////////////////////////
void
ws2812b_init(void)
{
  soft_timer_init_elem(&_ws2812b_refresh_timer);
  _ws2812b_refresh_timer.cb   = ws2812b_refresh_callback;

  memset(_color_buffer, 0, sizeof(_color_buffer));

  /* debug test */
  soft_timer_init_elem(&_debug_timer);
  _debug_timer.cb = debug_test_timer_callback;
  do_debug_test();

  ws2812b_fill_dma_buffer();
  ws2812b_start_transmission();
}

void
ws2812b_update_color(uint8_t led_ndx, uint32_t color)
{
  _color_buffer[led_ndx * 3 + 0] = (uint8_t)(color >> 16);
  _color_buffer[led_ndx * 3 + 1] = (uint8_t)(color >>  8);
  _color_buffer[led_ndx * 3 + 2] = (uint8_t)(color);
}
