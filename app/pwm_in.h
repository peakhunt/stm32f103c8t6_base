#ifndef __PWM_IN_DEF_H__
#define __PWM_IN_DEF_H__

typedef enum
{
  PWMInChannelNumber_0  = 0,
  PWMInChannelNumber_1, 
  PWMInChannelNumber_MAX,
} PWMInChannelNumber;

extern void pwm_in_init(void);
extern void pwm_in_start(void);
extern void pwm_in_stop(void);
extern uint16_t pwm_in_get(PWMInChannelNumber chnl);

#endif // !__PWM_IN_DEF_H__
