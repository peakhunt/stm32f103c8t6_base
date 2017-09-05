#ifndef __MATH_HELPER_DEF_H__
#define __MATH_HELPER_DEF_H__

#define M_PIf       3.14159265358979323846f
#define M_LN2f      0.69314718055994530942f
#define M_Ef        2.71828182845904523536f
    
#define RAD         (M_PIf / 180.0f)

#define F_EPSILON       0.000000001f

typedef struct
{
  double      x;
  double      y;
  double      z;
} VectorDouble;

typedef struct
{
  int16_t     x;
  int16_t     y;
  int16_t     z;
} VectorInt16;

typedef struct
{
  float       x;
  float       y;
  float       z;
} VectorFloat;

static inline float
invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;

  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;

}

static inline bool
float_zero(float x)
{
  if(fabs(x) < F_EPSILON)
  {
    return true;
  }
  return false;
}

#endif //!__MATH_HELPER_DEF_H__
