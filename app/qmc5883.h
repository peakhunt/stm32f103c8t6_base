#ifndef __QMC5883_DEF_H__
#define __QMC5883_DEF_H__

#define QMC5883_ADDRESS_MAG            (0x0d)

typedef enum
{
  QMC5883_REGISTER_MAG_OUT_X_L_M             = 0x00,
  QMC5883_REGISTER_MAG_OUT_X_H_M             = 0x01,
  QMC5883_REGISTER_MAG_OUT_Y_L_M             = 0x02,
  QMC5883_REGISTER_MAG_OUT_Y_H_M             = 0x03,
  QMC5883_REGISTER_MAG_OUT_Z_L_M             = 0x04,
  QMC5883_REGISTER_MAG_OUT_Z_H_M             = 0x05,
  QMC5883_REGISTER_MAG_SR_REG                = 0x06,
  QMC5883_REGISTER_MAG_TEMP_OUT_L_M          = 0x07,
  QMC5883_REGISTER_MAG_TEMP_OUT_H_M          = 0x08,
  QMC5883_REGISTER_MAG_CTRL_REG1             = 0x09,
  QMC5883_REGISTER_MAG_CTRL_REG2             = 0x0a,
  QMC5883_REGISTER_MAG_SET_RESET_REG         = 0x0b,
} qmc5883MagRegisters_t;

typedef struct qmc5883Mag_s
{
  int16_t rx;     // raw x
  int16_t ry;     // raw y
  int16_t rz;     // raw y

  float to_mgauss;      // number to convert raw value to gauss

  uint8_t             address;
} qmc5883Mag;

extern void qmc5883_init(qmc5883Mag* mag, uint8_t address);
extern void qmc5883_read(qmc5883Mag* mag);

#endif //!__QMC5883_DEF_H__
