#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f1xx_hal.h"

#include "app_common.h"
#include "shell.h"
#include "shell_if_usart.h"
#include "shell_if_usb.h"
#include "micros.h"
#include "pwm_out.h"
#include "pwm_in.h"
#include "imu.h"
#include "barometer.h"
#include "mpu6050.h"
#include "i2c_bus.h"
#include "gyro_calibration.h"
#include "accel_calibration.h"
#include "mag_calibration.h"
#include "config.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////

#define SHELL_MAX_COLUMNS_PER_LINE      128
#define SHELL_COMMAND_MAX_ARGS          4

#define VERSION       "STM32F1 Shell V0.2a"

typedef void (*shell_command_handler)(ShellIntf* intf, int argc, const char** argv);

typedef struct
{
  const char*           command;
  const char*           description;
  shell_command_handler handler;
} ShellCommand;

////////////////////////////////////////////////////////////////////////////////
//
// private prototypes
//
////////////////////////////////////////////////////////////////////////////////
static void shell_command_help(ShellIntf* intf, int argc, const char** argv);
static void shell_command_version(ShellIntf* intf, int argc, const char** argv);
static void shell_command_uptime(ShellIntf* intf, int argc, const char** argv);
static void shell_command_micros(ShellIntf* intf, int argc, const char** argv);
static void shell_command_micros_test(ShellIntf* intf, int argc, const char** argv);
#if 0
static void shell_command_pwm_out(ShellIntf* intf, int argc, const char** argv);
static void shell_command_pwm_in(ShellIntf* intf, int argc, const char** argv);
#endif
static void shell_command_i2c_stat(ShellIntf* intf, int argc, const char** argv);
static void shell_command_save(ShellIntf* intf, int argc, const char** argv);
static void shell_command_show_config(ShellIntf* intf, int argc, const char** argv);
static void shell_command_show_fconfig(ShellIntf* intf, int argc, const char** argv);
static void shell_command_show_crc(ShellIntf* intf, int argc, const char** argv);

#ifdef __ENABLE_IMU
//static void shell_command_bmp180(ShellIntf* intf, int argc, const char** argv);
static void shell_command_imu_sensor(ShellIntf* intf, int argc, const char** argv);
static void shell_command_imu_raw(ShellIntf* intf, int argc, const char** argv);
static void shell_command_imu(ShellIntf* intf, int argc, const char** argv);
static void shell_command_gyro_calib(ShellIntf* intf, int argc, const char** argv);
static void shell_command_accel_calib_init(ShellIntf* intf, int argc, const char** argv);
static void shell_command_accel_calib(ShellIntf* intf, int argc, const char** argv);
static void shell_command_accel_calib_finish(ShellIntf* intf, int argc, const char** argv);
static void shell_command_mag_calib(ShellIntf* intf, int argc, const char** argv);
static void shell_command_mag_decl(ShellIntf* intf, int argc, const char** argv);
#endif

////////////////////////////////////////////////////////////////////////////////
//
// private variables
//
////////////////////////////////////////////////////////////////////////////////
const uint8_t                 _welcome[] = "\r\n**** Welcome ****\r\n";
const uint8_t                 _prompt[]  = "\r\nSTM32F1> ";

static char                   _print_buffer[SHELL_MAX_COLUMNS_PER_LINE + 1];

static LIST_HEAD(_shell_intf_list);

static ShellCommand     _commands[] = 
{
  {
    "help",
    "show this command",
    shell_command_help,
  },
  {
    "version",
    "show version",
    shell_command_version,
  },
  {
    "uptime",
    "show system uptime",
    shell_command_uptime,
  },
  {
    "micros",
    "show micros",
    shell_command_micros,
  },
  {
    "microstest",
    "perform micro accuracy test",
    shell_command_micros_test,
  },
#if 0
  {
    "pwm_out",
    "control pwm duty cycle",
    shell_command_pwm_out,
  },
  {
    "pwm_in",
    "get pwm input duty cycle",
    shell_command_pwm_in,
  },
#endif
#ifdef __ENABLE_IMU
#if 0
  {
    "bmp180",
    "display bmp180 data",
    shell_command_bmp180,
  },
#endif
  {
    "imu_sensor",
    "display imu sensor data",
    shell_command_imu_sensor,
  },
  {
    "imu_raw",
    "display imu raw sensor data",
    shell_command_imu_raw,
  },
  {
    "imu",
    "display imu info",
    shell_command_imu,
  },
  {
    "gyro_cal",
    "perform gyro calibration",
    shell_command_gyro_calib,
  },
  {
    "accel_cal_init",
    "perform accelerometer calibration init",
    shell_command_accel_calib_init,
  },
  {
    "accel_cal",
    "perform accelerometer calibration",
    shell_command_accel_calib,
  },
  {
    "accel_cal_finish",
    "finish accelerometer calibration",
    shell_command_accel_calib_finish,
  },
  {
    "mag_cal",
    "perform magnetometer calibration",
    shell_command_mag_calib,
  },
  {
    "mag_decl",
    "set magnetic declination",
    shell_command_mag_decl,
  },
#endif
  {
    "i2cstat",
    "display i2c stat",
    shell_command_i2c_stat,
  },
  {
    "save",
    "save calibration data",
    shell_command_save,
  },
  {
    "show_config",
    "show saved configuration",
    shell_command_show_config,
  },
  {
    "show_fconfig",
    "show config in flash",
    shell_command_show_fconfig,
  },
  {
    "show_crc",
    "show config crc",
    shell_command_show_crc,
  }
};

////////////////////////////////////////////////////////////////////////////////
//
// shell utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline void
shell_prompt(ShellIntf* intf)
{
  intf->put_tx_data(intf, (uint8_t*)_prompt, sizeof(_prompt) -1);
}

////////////////////////////////////////////////////////////////////////////////
//
// shell command handlers
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_command_help(ShellIntf* intf, int argc, const char** argv)
{
  size_t i;

  shell_printf(intf, "\r\n");

  for(i = 0; i < sizeof(_commands)/sizeof(ShellCommand); i++)
  {
    shell_printf(intf, "%-10s: ", _commands[i].command);
    shell_printf(intf, "%s\r\n", _commands[i].description);
  }
}

static void
shell_command_version(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
  shell_printf(intf, "%s\r\n", VERSION);
}

static void
shell_command_uptime(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
  shell_printf(intf, "System Uptime: %lu\r\n", __uptime);
}

static void
shell_command_micros(ShellIntf* intf, int argc, const char** argv)
{
  uint16_t    m;

  m = micros();

  shell_printf(intf, "\r\n");
  shell_printf(intf, "Micros: %u\r\n", m);
}

static void
shell_command_micros_test(ShellIntf* intf, int argc, const char** argv)
{
  uint16_t    m1, m2, d;

  m1 = micros();
  HAL_Delay(5);
  m2 = micros();
  d = m2 - m1;

  shell_printf(intf, "\r\n");
  shell_printf(intf, "M1: %u\r\n", m1);
  shell_printf(intf, "M2: %u\r\n", m2);
  shell_printf(intf, "D : %u\r\n", d);
}

#if 0
static void
shell_command_pwm_out(ShellIntf* intf, int argc, const char** argv)
{
  uint8_t     chnl;
  uint16_t    duty;

  //
  // pwm_out chnl duty
  //
  if( argc != 3)
  {
    shell_printf(intf, "Syntax error %s chnl duty\r\n", argv[0]);
    return;
  }

  chnl = atoi(argv[1]);
  duty = atoi(argv[2]);

  if(chnl >= PWMOutChannelNumber_MAX)
  {
    shell_printf(intf, "Invalid channel %d\r\n", chnl);
    return;
  }

#if 0 // servo test
  if(duty < PWM_MIN_DUTY_CYCLE || duty > PWM_MAX_DUTY_CYCLE)
  {
    shell_printf(intf, "Invalid duty %d\r\n", duty);
    return;
  }
#endif

  pwm_set_duty_cycle(chnl, duty);

  shell_printf(intf, "set duty cycle to %d for channel %d\r\n", duty, chnl);
}

static void
shell_command_pwm_in(ShellIntf* intf, int argc, const char** argv)
{
  uint8_t     chnl;
  uint16_t    duty;

  //
  // pwm_in chnl
  //
  if( argc != 2)
  {
    shell_printf(intf, "Syntax error %s chnl\r\n", argv[0]);
    return;
  }

  chnl = atoi(argv[1]);

  if(chnl >= PWMInChannelNumber_MAX)
  {
    shell_printf(intf, "Invalid channel %d\r\n", chnl);
    return;
  }

  duty = pwm_in_get(chnl);

  shell_printf(intf, "duty cycle for channel %d is %d\r\n", chnl, duty);
}
#endif


static void
shell_command_i2c_stat(ShellIntf* intf, int argc, const char** argv)
{
  I2CBusStat* stat;

  stat = i2c_bus_get_stat(I2CBus_0);
  shell_printf(intf, "Bus1  num read       : %lu\r\n", stat->num_read);
  shell_printf(intf, "Bus1  num read fail  : %lu\r\n", stat->num_read_fail);
  shell_printf(intf, "Bus1  num write      : %lu\r\n", stat->num_write);
  shell_printf(intf, "Bus1  num write fail : %lu\r\n", stat->num_write_fail);

  stat = i2c_bus_get_stat(I2CBus_1);
  shell_printf(intf, "Bus2  num read       : %lu\r\n", stat->num_read);
  shell_printf(intf, "Bus2  num read fail  : %lu\r\n", stat->num_read_fail);
  shell_printf(intf, "Bus2  num write      : %lu\r\n", stat->num_write);
  shell_printf(intf, "Bus2  num write fail : %lu\r\n", stat->num_write_fail);
}

#ifdef __ENABLE_IMU
#if 0
static void
shell_command_bmp180(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "Raw Temp   : %ld C(0.1)\r\n", barometer_get_temperature());
  shell_printf(intf, "Raw Pres   : %ld Pa\r\n", barometer_get_pressure());
  shell_printf(intf, "Temperature: %.2f C\r\n", barometer_get_temperature()/10.0f);
  shell_printf(intf, "Pressure   : %.2f mBar\r\n", barometer_get_pressure()/100.0f);
}
#endif

static void
shell_command_imu_sensor(ShellIntf* intf, int argc, const char** argv)
{
  int32_t   data[3];

  imu_get_accel(imu_get_instance(0), data);
  shell_printf(intf, "Accel X: %ld, Y: %ld, Z: %ld\r\n", data[0], data[1], data[2]);

  imu_get_gyro(imu_get_instance(0), data);
  shell_printf(intf, "Gyro X: %ld, Y: %ld, Z: %ld\r\n", data[0], data[1], data[2]);

  imu_get_mag(imu_get_instance(0), data);
  shell_printf(intf, "Mag  X: %ld, Y: %ld, Z: %ld\r\n", data[0], data[1], data[2]);
}

static void
shell_command_imu_raw(ShellIntf* intf, int argc, const char** argv)
{
  IMU_t* imu = imu_get_instance(0);

  shell_printf(intf, "Accel X: %d, Y: %d, Z: %d\r\n",
      imu->mpu6050.Accelerometer_X,
      imu->mpu6050.Accelerometer_Y,
      imu->mpu6050.Accelerometer_Z);

  shell_printf(intf, "Gyro X: %d, Y: %d, Z: %d\r\n", 
    imu->mpu6050.Gyroscope_X,
    imu->mpu6050.Gyroscope_Y,
    imu->mpu6050.Gyroscope_Z);

  shell_printf(intf, "Mag  X: %d, Y: %d, Z: %d\r\n", 
      (int)imu->mag.rx,
      (int)imu->mag.ry,
      (int)imu->mag.rz);
}

static void
shell_command_imu(ShellIntf* intf, int argc, const char** argv)
{
  float   orient[3];

  imu_get_orientation(imu_get_instance(0), orient);

  shell_printf(intf, "Pitch: %.2f, Roll: %.2f, Yaw: %.2f\r\n",
      orient[1], orient[0], orient[2]);
}

static void
shell_command_gyro_calib_done(void* arg)
{
  int16_t     offs[3];
  ShellIntf*  intf = (ShellIntf*)arg;

  gyro_calib_get_offset(offs);

  imu_set_gyro_calib(imu_get_instance(0),  offs[0], offs[1], offs[2]);

  imu_start(imu_get_instance(0)); 

  shell_printf(intf, "Gyro Calibration Complete\r\n");
  shell_printf(intf, "X Offset: %d\r\n", offs[0]);
  shell_printf(intf, "Y Offset: %d\r\n", offs[1]);
  shell_printf(intf, "Z Offset: %d\r\n", offs[2]);

  config_get()->gyro_off[0] = offs[0];
  config_get()->gyro_off[1] = offs[1];
  config_get()->gyro_off[2] = offs[2];

  imu_set_gyro_calib(imu_get_instance(0), offs[0], offs[1], offs[2]);

  shell_prompt(intf);
}

static void
shell_command_gyro_calib(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "Starting Gyro Calibration\r\n");

  gyro_calib_init();

  imu_stop(imu_get_instance(0));

  gyro_calib_perform(&(imu_get_instance(0)->mpu6050), shell_command_gyro_calib_done, intf);
}

static void
shell_command_accel_calib_init(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "Init Gyro Calibration\r\n");

  accel_calib_init();

  imu_stop(imu_get_instance(0));
}

static void
shell_command_accel_calib_done(int axis_ndx, void* arg)
{
  ShellIntf*  intf = (ShellIntf*)arg;

  shell_printf(intf, "Accel Calibration for %d Axis Complete\r\n", axis_ndx);
  shell_prompt(intf);
}

static void
shell_command_accel_calib(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "Starting Accelerometer Calibration for Axis\r\n");

  accel_calib_perform(&(imu_get_instance(0)->mpu6050), shell_command_accel_calib_done, intf);
}

static void
shell_command_accel_calib_finish(ShellIntf* intf, int argc, const char** argv)
{
  int32_t     offs[3],
              gain[3];

  accel_calib_calculate();

  imu_start(imu_get_instance(0)); 

  accel_calib_get_offset_gain(offs, gain);

  shell_printf(intf, "X Offset: %ld\r\n", offs[0]);
  shell_printf(intf, "Y Offset: %ld\r\n", offs[1]);
  shell_printf(intf, "Z Offset: %ld\r\n", offs[2]);

  shell_printf(intf, "X Gain: %ld\r\n", gain[0]);
  shell_printf(intf, "Y Gain: %ld\r\n", gain[1]);
  shell_printf(intf, "Z Gain: %ld\r\n", gain[2]);

  config_get()->accl_off[0] = offs[0];
  config_get()->accl_off[1] = offs[1];
  config_get()->accl_off[2] = offs[2];
  config_get()->accl_scale[0] = gain[0];
  config_get()->accl_scale[1] = gain[1];
  config_get()->accl_scale[2] = gain[2];

  imu_set_accel_calib(imu_get_instance(0),
                      offs[0], offs[1], offs[2],
                      gain[0], gain[1], gain[2]);
#if 0
  {

    int32_t     sum[6][3],
                count[6];
    accel_get_acc_sum(sum, count);
    for(int i = 0; i < 6; i++)
    {
      for(int j = 0; j < 3; j++)
      {
        shell_printf(intf, "SUM %d:%d : %ld\r\n", i, j, sum[i][j]);
      }
    }
    for(int i = 0; i < 6; i++)
    {
      shell_printf(intf, "count %d : %ld\r\n", i, count[i]);
    }
  }
#endif
}

static void
mag_calib_done(void* arg)
{
  ShellIntf* intf = (ShellIntf*)arg;
  int32_t           bias[3];

  shell_printf(intf, "\r\nDone Magnetometer Calibration\r\n");

  mag_calib_get_offset(bias);

  imu_set_mag_calib(imu_get_instance(0), (int16_t)bias[0], (int16_t)bias[1], (int16_t)bias[2]);
  imu_start(imu_get_instance(0)); 

  shell_printf(intf, "Mag Bias X : %ld\r\n", bias[0]);
  shell_printf(intf, "Mag Bias Y : %ld\r\n", bias[1]);
  shell_printf(intf, "Mag Bias Z : %ld\r\n", bias[2]);

  config_get()->mag_bias[0] = bias[0];
  config_get()->mag_bias[1] = bias[1];
  config_get()->mag_bias[2] = bias[2];

  shell_prompt(intf);
}

static void
shell_command_mag_calib(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "Starting Magnetometer Calibration\r\n");

  imu_stop(imu_get_instance(0));

  mag_calib_init();
  mag_calib_perform(&(imu_get_instance(0)->mag), mag_calib_done, intf);
}

static void
shell_command_mag_decl(ShellIntf* intf, int argc, const char** argv)
{
  int32_t   v;

  if(argc != 2) 
  {
    shell_printf(intf, "invalid command: %s <mag decl value>\r\n", argv[0]);
    return;
  }

  v = (int32_t)(atof(argv[1]) * 100);

  config_get()->mag_declination = v;
  imu_reload_mag_declination(imu_get_instance(0));
}

#endif

static void
shell_command_save(ShellIntf* intf, int argc, const char** argv)
{
  config_save();
  shell_printf(intf, "Done saving configuration\r\n");
}

static void
print_out_config(ShellIntf* intf, config_t* cfg)
{
  shell_printf(intf, "version       %ld\r\n", cfg->version);
  shell_printf(intf, "magic         %lx\r\n", cfg->magic);
  shell_printf(intf, "accl_off[0]   %d\r\n", cfg->accl_off[0]);
  shell_printf(intf, "accl_off[1]   %d\r\n", cfg->accl_off[1]);
  shell_printf(intf, "accl_off[2]   %d\r\n", cfg->accl_off[2]);
  shell_printf(intf, "accl_scale[0] %d\r\n", cfg->accl_scale[0]);
  shell_printf(intf, "accl_scale[1] %d\r\n", cfg->accl_scale[1]);
  shell_printf(intf, "accl_scale[2] %d\r\n", cfg->accl_scale[2]);
  shell_printf(intf, "gyro_off[0]   %d\r\n", cfg->gyro_off[0]);
  shell_printf(intf, "gyro_off[1]   %d\r\n", cfg->gyro_off[1]);
  shell_printf(intf, "gyro_off[2]   %d\r\n", cfg->gyro_off[2]);
  shell_printf(intf, "mag_bias[0]   %d\r\n", cfg->mag_bias[0]);
  shell_printf(intf, "mag_bias[1]   %d\r\n", cfg->mag_bias[1]);
  shell_printf(intf, "mag_bias[2]   %d\r\n", cfg->mag_bias[2]);
  shell_printf(intf, "mag_decl      %.2f\r\n", cfg->mag_declination / 100.0f);
}

static void
shell_command_show_config(ShellIntf* intf, int argc, const char** argv)
{
  config_t*   cfg = config_get();

  print_out_config(intf, cfg);
}

static void
shell_command_show_fconfig(ShellIntf* intf, int argc, const char** argv)
{
  config_t*   cfg = config_get_flash();

  print_out_config(intf, cfg);
}

static void
shell_command_show_crc(ShellIntf* intf, int argc, const char** argv)
{
  uint16_t      mem, flash;

  config_get_crc(&mem, &flash);

  shell_printf(intf, "mem     %d\r\n", mem);
  shell_printf(intf, "flash   %d\r\n", flash);
}

////////////////////////////////////////////////////////////////////////////////
//
// shell core
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_execute_command(ShellIntf* intf, char* cmd)
{
  static const char*    argv[SHELL_COMMAND_MAX_ARGS];
  int                   argc = 0;
  size_t                i;
  char                  *s, *t;

  while((s = strtok_r(argc  == 0 ? cmd : NULL, " \t", &t)) != NULL)
  {
    if(argc >= SHELL_COMMAND_MAX_ARGS)
    {
      shell_printf(intf, "\r\nError: too many arguments\r\n");
      return;
    }
    argv[argc++] = s;
  }

  if(argc == 0)
  {
    return;
  }

  for(i = 0; i < sizeof(_commands)/sizeof(ShellCommand); i++)
  {
    if(strcmp(_commands[i].command, argv[0]) == 0)
    {
      shell_printf(intf, "\r\nExecuting %s\r\n", argv[0]);
      _commands[i].handler(intf, argc, argv);
      return;
    }
  }
  shell_printf(intf, "%s", "\r\nUnknown Command: ");
  shell_printf(intf, "%s", argv[0]);
  shell_printf(intf, "%s", "\r\n");
}


void
shell_printf(ShellIntf* intf, const char* fmt, ...)
{
  va_list   args;
  int       len;

  va_start(args, fmt);
  len = vsnprintf(_print_buffer, SHELL_MAX_COLUMNS_PER_LINE, fmt, args);
  va_end(args);

  intf->put_tx_data(intf, (uint8_t*)_print_buffer, len);
}


////////////////////////////////////////////////////////////////////////////////
//
// public interface
//
////////////////////////////////////////////////////////////////////////////////
void
shell_init(void)
{
  shell_if_usart_init();
  shell_if_usb_init();
}

void
shell_start(void)
{
  ShellIntf* intf;

  list_for_each_entry(intf, &_shell_intf_list, lh)
  {
    intf->put_tx_data(intf, (uint8_t*)_welcome, sizeof(_welcome) -1);
    shell_prompt(intf);
  }
}


void
shell_if_register(ShellIntf* intf)
{
  list_add_tail(&intf->lh, &_shell_intf_list);
}

void
shell_handle_rx(ShellIntf* intf)
{
  uint8_t   b;

  while(1)
  {
    if(intf->get_rx_data(intf, &b) == false)
    {
      return;
    }

    if(b != '\r' && intf->cmd_buffer_ndx < SHELL_MAX_COMMAND_LEN)
    {
      if(b == '\b' || b == 0x7f)
      {
        if(intf->cmd_buffer_ndx > 0)
        {
          shell_printf(intf, "%c%c%c", b, 0x20, b);
          intf->cmd_buffer_ndx--;
        }
      }
      else
      {
        shell_printf(intf, "%c", b);
        intf->cmd_buffer[intf->cmd_buffer_ndx++] = b;
      }
    }
    else if(b == '\r')
    {
      intf->cmd_buffer[intf->cmd_buffer_ndx++] = '\0';

      shell_execute_command(intf, (char*)intf->cmd_buffer);

      intf->cmd_buffer_ndx = 0;
      shell_prompt(intf);
    }
  }
}

struct list_head*
shell_get_intf_list(void)
{
  return &_shell_intf_list;
}
