#ifndef __MPU_6050_DEF_H__
#define __MPU_6050_DEF_H__

#define MPU6050_I2C_ADDR        0x68
//#define MPU6050_I_AM            0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO             0x01
#define MPU6050_SMPLRT_DIV            0x19
#define MPU6050_REG_ACCEL_XOFFS_H     (0x06)
#define MPU6050_REG_ACCEL_XOFFS_L     (0x07)
#define MPU6050_REG_ACCEL_YOFFS_H     (0x08)
#define MPU6050_REG_ACCEL_YOFFS_L     (0x09)
#define MPU6050_REG_ACCEL_ZOFFS_H     (0x0A)
#define MPU6050_REG_ACCEL_ZOFFS_L     (0x0B)
#define MPU6050_REG_GYRO_XOFFS_H      (0x13)
#define MPU6050_REG_GYRO_XOFFS_L      (0x14)
#define MPU6050_REG_GYRO_YOFFS_H      (0x15)
#define MPU6050_REG_GYRO_YOFFS_L      (0x16)
#define MPU6050_REG_GYRO_ZOFFS_H      (0x17)
#define MPU6050_REG_GYRO_ZOFFS_L      (0x18)
#define MPU6050_CONFIG                0x1A
#define MPU6050_GYRO_CONFIG           0x1B
#define MPU6050_ACCEL_CONFIG          0x1C
#define MPU6050_MOTION_THRESH         0x1F
#define MPU6050_INT_PIN_CFG           0x37
#define MPU6050_INT_ENABLE            0x38
#define MPU6050_INT_STATUS            0x3A
#define MPU6050_ACCEL_XOUT_H          0x3B
#define MPU6050_ACCEL_XOUT_L          0x3C
#define MPU6050_ACCEL_YOUT_H          0x3D
#define MPU6050_ACCEL_YOUT_L          0x3E
#define MPU6050_ACCEL_ZOUT_H          0x3F
#define MPU6050_ACCEL_ZOUT_L          0x40
#define MPU6050_TEMP_OUT_H            0x41
#define MPU6050_TEMP_OUT_L            0x42
#define MPU6050_GYRO_XOUT_H           0x43
#define MPU6050_GYRO_XOUT_L           0x44
#define MPU6050_GYRO_YOUT_H           0x45
#define MPU6050_GYRO_YOUT_L           0x46
#define MPU6050_GYRO_ZOUT_H           0x47
#define MPU6050_GYRO_ZOUT_L           0x48
#define MPU6050_MOT_DETECT_STATUS     0x61
#define MPU6050_SIGNAL_PATH_RESET     0x68
#define MPU6050_MOT_DETECT_CTRL       0x69
#define MPU6050_USER_CTRL             0x6A
#define MPU6050_PWR_MGMT_1            0x6B
#define MPU6050_PWR_MGMT_2            0x6C
#define MPU6050_FIFO_COUNTH           0x72
#define MPU6050_FIFO_COUNTL           0x73
#define MPU6050_FIFO_R_W              0x74
#define MPU6050_WHO_AM_I              0x75

/* Gyro sensitivities in °/s */
#define MPU6050_GYRO_SENS_250       ((float) 131)
#define MPU6050_GYRO_SENS_500       ((float) 65.5)
#define MPU6050_GYRO_SENS_1000      ((float) 32.8)
#define MPU6050_GYRO_SENS_2000      ((float) 16.4)

/* Acce sensitivities in g */
#define MPU6050_ACCE_SENS_2         ((float) 16384)
#define MPU6050_ACCE_SENS_4         ((float) 8192)
#define MPU6050_ACCE_SENS_8         ((float) 4096)
#define MPU6050_ACCE_SENS_16        ((float) 2048)


typedef enum
{
  MPU6050_Accelerometer_2G  = 0x00, /*!< Range is +- 2G */
  MPU6050_Accelerometer_4G  = 0x01, /*!< Range is +- 4G */
  MPU6050_Accelerometer_8G  = 0x02, /*!< Range is +- 8G */
  MPU6050_Accelerometer_16G = 0x03  /*!< Range is +- 16G */
} MPU6050_Accelerometer_t;

typedef enum 
{
  MPU6050_Gyroscope_250s  = 0x00,   /*!< Range is +- 250 degrees/s */
  MPU6050_Gyroscope_500s  = 0x01,   /*!< Range is +- 500 degrees/s */ 
  MPU6050_Gyroscope_1000s = 0x02,   /*!< Range is +- 1000 degrees/s */
  MPU6050_Gyroscope_2000s = 0x03    /*!< Range is +- 2000 degrees/s */
} MPU6050_Gyroscope_t;

typedef struct 
{
  /* Private */
  uint8_t     Address;          /*!< I2C address of device. Only for private use */
  float       Gyro_Mult;        /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
  float       Acce_Mult;        /*!< Accelerometer corrector from raw data to "g". Only for private use */
  int16_t     one_g;            /* raw ADC value equivalent to 1G */

  /* Public */
  int16_t     Accelerometer_X;  /*!< Accelerometer value X axis */
  int16_t     Accelerometer_Y;  /*!< Accelerometer value Y axis */
  int16_t     Accelerometer_Z;  /*!< Accelerometer value Z axis */
  int16_t     Gyroscope_X;      /*!< Gyroscope value X axis */
  int16_t     Gyroscope_Y;      /*!< Gyroscope value Y axis */
  int16_t     Gyroscope_Z;      /*!< Gyroscope value Z axis */
  float       Temperature;      /*!< Temperature in degrees */
} MPU6050_t;

extern bool mpu6050_init(MPU6050_t* mpu6050,
  MPU6050_Accelerometer_t accel_sensitivity, MPU6050_Gyroscope_t gyro_sensitivity);
extern bool mpu6050_read_accel(MPU6050_t* mpu6050);
extern bool mpu6050_read_gyro(MPU6050_t* mpu6050);
extern bool mpu6050_read_temperature(MPU6050_t* mpu6050);
extern bool mpu6050_read_all(MPU6050_t* mpu6050);

extern void mpu6050_set_gyro_offset(MPU6050_t* mpu6050, int16_t x, int16_t y, int16_t z);
extern void mpu6050_set_accel_offset(MPU6050_t* mpu6050, int16_t x, int16_t y, int16_t z);

extern void mpu6050_set_gyro_dlpf(MPU6050_t* mpu6050, uint8_t val);

#endif //!__MPU_6050_DEF_H__
