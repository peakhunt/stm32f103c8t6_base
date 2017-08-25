#ifndef __WS2812B_DEF_H__
#define __WS2812B_DEF_H__

//
// TIM2 is configured to
// a) run at 24 Mhz Clock
// b) thus, 1 tick takes 0.0416 us
// c) PWM period is set to 30 - 1, which is 1.248 us
// 
#define WS2812B_MAX_LED         3

//
// from WS2812B datasheet
// T0 High : 0.25us
// T1 High : 0.6us
//
// here, 1 timer tick is 0.111us
// so roughly we can the following values
//
#define WS2812B_LOW_DUTY_CYCLE        6     // 0.2496 us
#define WS2812B_HIGH_DUTY_CYCLE       15    // 0.624 us

#define WS2812B_UPDATE_INTERVAL       50    // every 50ms. max 20 HZ

extern void ws2812b_init(void);
extern void ws2812b_update_color(uint8_t led_ndx, uint32_t color);
extern void ws2812b_dma_complete_callback(void);

#endif /* !__WS2812B_DEF_H__ */
