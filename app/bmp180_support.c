#include "app_common.h"
#include "i2c_bus.h"
#include "bmp180.h"

#define C_BMP180_ONE_U8X        1

#ifdef BMP180_API

s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMP180_I2C_routine(struct bmp180_t* bmp180);
#endif

void BMP180_delay_msek(u32 msek);

#if 0
s32 bmp180_data_readout_template(void)
{
	s32 com_rslt = E_BMP_COMM_RES;
	u16 v_uncomp_temp_u16 = BMP180_INIT_VALUE;
	u32 v_uncomp_press_u32 = BMP180_INIT_VALUE;

	#ifdef BMP180_API
	BMP180_I2C_routine();
	#endif

	com_rslt = bmp180_init(&bmp180);
	com_rslt += bmp180_get_calib_param();

	v_uncomp_temp_u16 = bmp180_get_uncomp_temperature();
	v_uncomp_press_u32 = bmp180_get_uncomp_pressure();

	com_rslt += bmp180_get_temperature(v_uncomp_temp_u16);
	com_rslt += bmp180_get_pressure(v_uncomp_press_u32);

  return com_rslt;
}
#endif

#ifdef BMP180_API
s8 BMP180_I2C_routine(struct bmp180_t* bmp180)
{
	bmp180->bus_write   = BMP180_I2C_bus_write;
	bmp180->bus_read    = BMP180_I2C_bus_read;
	bmp180->dev_addr    = BMP180_I2C_ADDR;
	bmp180->delay_msec  = BMP180_delay_msek;

	return BMP180_INIT_VALUE;
}

#define	I2C_BUFFER_LEN 32
#define I2C0 5

s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP180_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BMP180_INIT_VALUE;

	array[BMP180_INIT_VALUE] = reg_addr;

	for (stringpos = BMP180_INIT_VALUE; stringpos < cnt; stringpos++)
  {
		array[stringpos + C_BMP180_ONE_U8X] = *(reg_data + stringpos);
	}

  // hkim implementation
  i2c_bus_write_sync(BMP180_I2C_BUS, dev_addr, array, cnt + C_BMP180_ONE_U8X);

	return (s8)iError;
}

s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP180_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BMP180_INIT_VALUE};
	u8 stringpos = BMP180_INIT_VALUE;
	array[BMP180_INIT_VALUE] = reg_addr;

  // hkim implementation
  i2c_bus_write_sync(BMP180_I2C_BUS, dev_addr, array, C_BMP180_ONE_U8X);
  i2c_bus_read_sync(BMP180_I2C_BUS, dev_addr, array, cnt);

	for (stringpos = BMP180_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}
	return (s8)iError;
}

void BMP180_delay_msek(u32 msek)
{
}

#endif
