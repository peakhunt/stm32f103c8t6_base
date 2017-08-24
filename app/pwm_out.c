#include "stm32f1xx_hal.h"
#include "tim.h"

#include "pwm_out.h"

////////////////////////////////////////////////////////////////////////////////
//
// private prototypes
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static TIM_HandleTypeDef*         _htim = &htim3;

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
pwm_out_init(void)
{
  //
  // TIM3 is already configured to generate PWM with 2.5ms period.
  // It's time to set duty cycle and start each channel.
  //
  __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, PWM_MIN_DUTY_CYCLE);
  __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, PWM_MIN_DUTY_CYCLE);
  __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_3, PWM_MIN_DUTY_CYCLE);
  __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_4, PWM_MIN_DUTY_CYCLE);

  HAL_TIM_Base_Start(_htim);

  HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_4);
}

void
pwm_out_start(void)
{
  HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_4);
}

void
pwm_out_stop(void)
{
  HAL_TIM_PWM_Stop(_htim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(_htim, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(_htim, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(_htim, TIM_CHANNEL_4);
}

void
pwm_set_duty_cycle(PWMOutChannelNumber chnl,  uint16_t duty_cycle)
{
  uint8_t ch;

  switch(chnl)
  {
  case PWMOutChannelNumber_0:
    ch = TIM_CHANNEL_1;
    break;

  case PWMOutChannelNumber_1:
    ch = TIM_CHANNEL_2;
    break;

  case PWMOutChannelNumber_2:
    ch = TIM_CHANNEL_3;
    break;

  case PWMOutChannelNumber_3:
    ch = TIM_CHANNEL_4;
    break;

  default:
    return;
  }

  __HAL_TIM_SET_COMPARE(_htim, ch, duty_cycle);
}
