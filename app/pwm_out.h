#ifndef __PWM_OUT_DEF_H__
#define __PWM_OUT_DEF_H__


////////////////////////////////////////////////////////////////////////////////
//
// the goal of this module is
//
// a) generate 4 PWM signals using 400 Hz Frequency ( 2500 us period )
// b) PWM duty cycle should be configurable in 1 us basis
// c) minimum PWM duty cycle is 1000 us
// d) maximum PWM duty cycle is 2000 us
//
////////////////////////////////////////////////////////////////////////////////

#define PWM_FREQUENCY_400HZ         400
#define PWM_PERIOD_400HZ_IN_US      2500
#define PWM_MIN_DUTY_CYCLE          1000
#define PWM_MAX_DUTY_CYCLE          2000

typedef enum
{
  PWMOutChannelNumber_0 = 0,
  PWMOutChannelNumber_1,
  PWMOutChannelNumber_2,
  PWMOutChannelNumber_3,
  PWMOutChannelNumber_MAX,
} PWMOutChannelNumber;

typedef enum
{
  PWMFrequency_400HZ,
} PWMFrequency;

extern void pwm_out_init(void);
extern void pwm_out_start(void);
extern void pwm_out_stop(void);
extern void pwm_set_duty_cycle(PWMOutChannelNumber chnl,  uint16_t duty_cycle);

#endif //!__PWM_OUT_DEF_H__
