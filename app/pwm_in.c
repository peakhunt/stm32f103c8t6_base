#include "stm32f1xx_hal.h"
#include "tim.h"

#include "pwm_in.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitiosn & prototypes
//
////////////////////////////////////////////////////////////////////////////////
typedef enum
{
  PWMInChannelState_WF_Rising,
  PWMInChannelState_WF_Falling,
} PWMInChannelState_t;

typedef struct
{
  PWMInChannelState_t   state;
  uint16_t              start_time;
  volatile uint16_t     duty_cycle;
} PWMInChannel;

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static TIM_HandleTypeDef*         _htim = &htim4;
static PWMInChannel               _in_channels[PWMInChannelNumber_MAX];

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
void
HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
  PWMInChannel* target = NULL;
  uint32_t      chnl = 0;
  static TIM_IC_InitTypeDef   rising_config =
  {
    .ICPolarity   = TIM_INPUTCHANNELPOLARITY_RISING,
    .ICSelection  = TIM_ICSELECTION_DIRECTTI,
    .ICPrescaler  = TIM_ICPSC_DIV1,
  };
  static TIM_IC_InitTypeDef   falling_config =
  {
    .ICPolarity   = TIM_INPUTCHANNELPOLARITY_FALLING,
    .ICSelection  = TIM_ICSELECTION_DIRECTTI,
    .ICPrescaler  = TIM_ICPSC_DIV1,
  };

  switch(htim->Channel)
  {
  case HAL_TIM_ACTIVE_CHANNEL_1:
    target = &_in_channels[PWMInChannelNumber_0];
    chnl  = TIM_CHANNEL_1;
    break;

  case HAL_TIM_ACTIVE_CHANNEL_2:
    target = &_in_channels[PWMInChannelNumber_1];
    chnl  = TIM_CHANNEL_2;
    break;

  default:
    return;
  }

  if(target->state == PWMInChannelState_WF_Rising)
  {
    target->start_time  = (uint16_t)__HAL_TIM_GET_COMPARE(_htim, chnl);
    target->state       = PWMInChannelState_WF_Falling;

    HAL_TIM_IC_ConfigChannel(htim, &falling_config, chnl);
  }
  else
  {
    uint16_t  now = (uint16_t)__HAL_TIM_GET_COMPARE(_htim, chnl);

    target->state       = PWMInChannelState_WF_Rising;
    target->duty_cycle  = now - target->start_time;

    HAL_TIM_IC_ConfigChannel(htim, &rising_config, chnl);
  }
  HAL_TIM_IC_Start_IT(_htim, chnl);
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
pwm_in_init(void)
{
  int i = 0;

  for(i = 0; i < PWMInChannelNumber_MAX; i++)
  {
    _in_channels[i].state       = PWMInChannelState_WF_Rising;
    _in_channels[i].start_time  = 0;
    _in_channels[i].duty_cycle  = 0;
  }

  HAL_TIM_Base_Start(_htim);

  pwm_in_start();
}

void
pwm_in_start(void)
{
  HAL_TIM_IC_Start_IT(_htim, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(_htim, TIM_CHANNEL_2);
}

void
pwm_in_stop(void)
{
  HAL_TIM_IC_Stop_IT(_htim, TIM_CHANNEL_1);
  HAL_TIM_IC_Stop_IT(_htim, TIM_CHANNEL_2);
}

uint16_t
pwm_in_get(PWMInChannelNumber chnl)
{
  return _in_channels[chnl].duty_cycle;
}

