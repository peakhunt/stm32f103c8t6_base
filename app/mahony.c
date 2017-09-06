#include <math.h>
#include "app_common.h"
#include "mahony.h"
#include "math_helper.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////
#define twoKpDef  (2.0f * 0.5f)     // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f)     // 2 * integral gain

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
mahony_init(Mahony* mahony, float sampleFrequency)
{
  mahony->twoKp = twoKpDef; // 2 * proportional gain (Kp)
  mahony->twoKi = twoKiDef; // 2 * integral gain (Ki)

  mahony->q0 = 1.0f;
  mahony->q1 = 0.0f;
  mahony->q2 = 0.0f;
  mahony->q3 = 0.0f;

  mahony->integralFBx = 0.0f;
  mahony->integralFBy = 0.0f;
  mahony->integralFBz = 0.0f;

  mahony->invSampleFreq = 1.0f / sampleFrequency;
}

void
mahony_update(Mahony* mahony,
							float gx, float gy, float gz,
							float ax, float ay, float az,
							float mx, float my, float mz)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if(float_zero(mx) && float_zero(my) && float_zero(mz))
	{
		mahony_updateIMU(mahony, gx, gy, gz, ax, ay, az);
		return;
	}

  // convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!(float_zero(ax) && float_zero(ay) && float_zero(az)))
	{
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = mahony->q0 * mahony->q0;
		q0q1 = mahony->q0 * mahony->q1;
		q0q2 = mahony->q0 * mahony->q2;
		q0q3 = mahony->q0 * mahony->q3;
		q1q1 = mahony->q1 * mahony->q1;
		q1q2 = mahony->q1 * mahony->q2;
		q1q3 = mahony->q1 * mahony->q3;
		q2q2 = mahony->q2 * mahony->q2;
		q2q3 = mahony->q2 * mahony->q3;
		q3q3 = mahony->q3 * mahony->q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(mahony->twoKi > 0.0f) {
			mahony->integralFBx += mahony->twoKi * halfex * mahony->invSampleFreq;  // integral error scaled by Ki
			mahony->integralFBy += mahony->twoKi * halfey * mahony->invSampleFreq;
			mahony->integralFBz += mahony->twoKi * halfez * mahony->invSampleFreq;
			gx += mahony->integralFBx;  // apply integral feedback
			gy += mahony->integralFBy;
			gz += mahony->integralFBz;
		}
		else {
			mahony->integralFBx = 0.0f; // prevent integral windup
			mahony->integralFBy = 0.0f;
			mahony->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += mahony->twoKp * halfex;
		gy += mahony->twoKp * halfey;
		gz += mahony->twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * mahony->invSampleFreq);   // pre-multiply common factors
	gy *= (0.5f * mahony->invSampleFreq);
	gz *= (0.5f * mahony->invSampleFreq);
	qa = mahony->q0;
	qb = mahony->q1;
	qc = mahony->q2;
	mahony->q0 += (-qb * gx - qc * gy - mahony->q3 * gz);
	mahony->q1 += ( qa * gx + qc * gz - mahony->q3 * gy);
	mahony->q2 += ( qa * gy - qb * gz + mahony->q3 * gx);
	mahony->q3 += ( qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(mahony->q0 * mahony->q0 + mahony->q1 * mahony->q1 +
											mahony->q2 * mahony->q2 + mahony->q3 * mahony->q3);
	mahony->q0 *= recipNorm;
	mahony->q1 *= recipNorm;
	mahony->q2 *= recipNorm;
	mahony->q3 *= recipNorm;

  mahony_compute_angle(mahony);
}

void
mahony_updateIMU(Mahony* mahony, float gx, float gy, float gz,
    float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Compute feedback only if accelerometer measurement valid
  // (avoids NaN in accelerometer normalisation)
  if(!(float_zero(ax) && float_zero(ay) && float_zero(az))) 
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    halfvx = mahony->q1 * mahony->q3 - mahony->q0 * mahony->q2;
    halfvy = mahony->q0 * mahony->q1 + mahony->q2 * mahony->q3;
    halfvz = mahony->q0 * mahony->q0 - 0.5f + mahony->q3 * mahony->q3;

    // Error is sum of cross product between estimated
    // and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(mahony->twoKi > 0.0f) {
      // integral error scaled by Ki
      mahony->integralFBx += mahony->twoKi * halfex * mahony->invSampleFreq;
      mahony->integralFBy += mahony->twoKi * halfey * mahony->invSampleFreq;
      mahony->integralFBz += mahony->twoKi * halfez * mahony->invSampleFreq;
      gx += mahony->integralFBx;  // apply integral feedback
      gy += mahony->integralFBy;
      gz += mahony->integralFBz;
    } else {
      mahony->integralFBx = 0.0f; // prevent integral windup
      mahony->integralFBy = 0.0f;
      mahony->integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += mahony->twoKp * halfex;
    gy += mahony->twoKp * halfey;
    gz += mahony->twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * mahony->invSampleFreq);   // pre-multiply common factors
  gy *= (0.5f * mahony->invSampleFreq);
  gz *= (0.5f * mahony->invSampleFreq);
  qa = mahony->q0;
  qb = mahony->q1;
  qc = mahony->q2;
  mahony->q0 += (-qb * gx - qc * gy - mahony->q3 * gz);
  mahony->q1 += (qa * gx + qc * gz - mahony->q3 * gy);
  mahony->q2 += (qa * gy - qb * gz + mahony->q3 * gx);
  mahony->q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(mahony->q0 * mahony->q0 +
                      mahony->q1 * mahony->q1 +
                      mahony->q2 * mahony->q2 +
                      mahony->q3 * mahony->q3);
  mahony->q0 *= recipNorm;
  mahony->q1 *= recipNorm;
  mahony->q2 *= recipNorm;
  mahony->q3 *= recipNorm;

  mahony_compute_angle(mahony);
}


void
mahony_compute_angle(Mahony* mahony)
{
#if 0
  mahony->roll  = atan2f(mahony->q0 * mahony->q1 + mahony->q2 * mahony->q3,
                        0.5f - mahony->q1 *mahony->q1 - mahony->q2 * mahony->q2);
  mahony->pitch = asinf(-2.0f * (mahony->q1 * mahony->q3 - mahony->q0 * mahony->q2));
  mahony->yaw   = atan2f(mahony->q1 * mahony->q2 + mahony->q0 * mahony->q3,
                        0.5f - mahony->q2 * mahony->q2 - mahony->q3 * mahony->q3);
#else
  float   a, b, c, d;

  a = mahony->q0;
  b = mahony->q1;
  c = mahony->q2;
  d = mahony->q3;

  mahony->roll  =  atan2(2.0f * (a * b + c * d), (a * a - b * b - c * c + d * d));
  mahony->pitch = -asin (2.0f * (b * d - a * c));
  mahony->yaw   =  atan2(2.0f * (a * d + b * c), (a * a + b * b - c * c - d * d));
#endif
}

void
mahony_get_pitch_roll_yaw(Mahony* mahony, float data[3])
{
  data[0] = mahony->pitch * 57.29578f;
  data[1] = mahony->roll * 57.29578f;
  data[2] = mahony->yaw * 57.29578f + 180.0f;
}

void
mahony_get_pitch_roll_yaw_radian(Mahony* mahony, float data[3])
{
  data[0] = mahony->pitch;
  data[1] = mahony->roll;
  data[2] = mahony->yaw;
}

void
mahony_get_quaternion(Mahony* mahony, float data[4])
{
  data[0] = mahony->q0;
  data[1] = mahony->q1;
  data[2] = mahony->q2;
  data[3] = mahony->q3;
}
