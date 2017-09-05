#ifndef __MADGWICK_DEF_H__
#define __MADGWICK_DEF_H__

extern void madgwick_init(float sample_freq);
extern void madgwick_updateIMU(float gx, float gy, float gz,
                               float ax, float ay, float az);
extern void madgwick_update(float gx, float gy, float gz,
                            float ax, float ay, float az,
								            float mx, float my, float mz);
extern void madgwick_get_pitch_roll_yaw(float data[3]);
extern void madgwick_get_quaternion(float data[4]);

#endif //!__MADGWICK_DEF_H__