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
#include "imu_calibration.h"
#include "barometer.h"
#include "mpu6050.h"
#include "i2c_bus.h"

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
static void shell_printf(ShellIntf* intf, const char* fmt, ...) __attribute__((format(gnu_printf, 2, 3)));

static void shell_command_help(ShellIntf* intf, int argc, const char** argv);
static void shell_command_version(ShellIntf* intf, int argc, const char** argv);
static void shell_command_uptime(ShellIntf* intf, int argc, const char** argv);
static void shell_command_micros(ShellIntf* intf, int argc, const char** argv);
static void shell_command_micros_test(ShellIntf* intf, int argc, const char** argv);
static void shell_command_pwm_out(ShellIntf* intf, int argc, const char** argv);
static void shell_command_pwm_in(ShellIntf* intf, int argc, const char** argv);
static void shell_command_i2c_stat(ShellIntf* intf, int argc, const char** argv);

#ifdef __ENABLE_IMU
static void shell_command_mag_data(ShellIntf* intf, int argc, const char** argv);
static void shell_command_bmp180(ShellIntf* intf, int argc, const char** argv);
static void shell_command_mpu6050(ShellIntf* intf, int argc, const char** argv);
static void shell_command_imu(ShellIntf* intf, int argc, const char** argv);
static void shell_command_accel_calib(ShellIntf* intf, int argc, const char** argv);
static void shell_command_mag_calib(ShellIntf* intf, int argc, const char** argv);
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
#ifdef __ENABLE_IMU
  {
    "mag_data",
    "read mag compass data",
    shell_command_mag_data,
  },
  {
    "bmp180",
    "display bmp180 data",
    shell_command_bmp180,
  },
  {
    "mpu6050",
    "display mpu6050 data",
    shell_command_mpu6050,
  },
  {
    "imu",
    "display imu info",
    shell_command_imu,
  },
  {
    "accel_cal",
    "perform accelerometer calibration",
    shell_command_accel_calib,
  },
  {
    "mag_cal",
    "perform magnetometer calibration",
    shell_command_mag_calib,
  },
#endif
  {
    "i2cstat",
    "display i2c stat",
    shell_command_i2c_stat,
  },
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
static void
shell_command_mag_data(ShellIntf* intf, int argc, const char** argv)
{
  float data[4];

  imu_get_mag(imu_get_instance(0), data);

  shell_printf(intf, "X: %.2f\r\n", data[0]);
  shell_printf(intf, "Y: %.2f\r\n", data[1]);
  shell_printf(intf, "Z: %.2f\r\n", data[2]);
  shell_printf(intf, "O: %.2f\r\n", data[3]);
}

static void
shell_command_bmp180(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "Raw Temp   : %ld C(0.1)\r\n", barometer_get_temperature());
  shell_printf(intf, "Raw Pres   : %ld Pa\r\n", barometer_get_pressure());
  shell_printf(intf, "Temperature: %.2f C\r\n", barometer_get_temperature()/10.0f);
  shell_printf(intf, "Pressure   : %.2f mBar\r\n", barometer_get_pressure()/100.0f);
}

static void
shell_command_mpu6050(ShellIntf* intf, int argc, const char** argv)
{
  int16_t   data[3];
  float     fdata[3];

  imu_get_accel(imu_get_instance(0), data, fdata);
  shell_printf(intf, "Accel X: %.2f, Y: %.2f, Z: %.2fd\r\n",
      fdata[0], fdata[1], fdata[2]);

  imu_get_gyro(imu_get_instance(0), data, fdata);
  shell_printf(intf, "Gyro  X: %.2f, Y: %.2f, Z: %.2f\r\n",
      fdata[0], fdata[1], fdata[2]);
}

static void
shell_command_imu(ShellIntf* intf, int argc, const char** argv)
{
  float   mahony[4],
          madgwick[4];

  imu_get_orientation(imu_get_instance(0), mahony, madgwick);

  shell_printf(intf, "Mahony   Pitch: %.2f, Roll: %.2f, Yaw: %.2f, Heading: %.2f\r\n",
      mahony[1], mahony[0], mahony[2], mahony[3]);

  shell_printf(intf, "Madgwick Pitch: %.2f, Roll: %.2f, Yaw: %.2f, Heading: %.2f\r\n",
      madgwick[1], madgwick[0], madgwick[2], madgwick[3]);

  shell_printf(intf, "Heading Raw: %.2f\r\n", imu_get_instance(0)->heading_raw);
}

static void
accel_calib_done(void* arg)
{
  ShellIntf* intf = (ShellIntf*)arg;
  int16_t   gyro[3],
            accl[3];

  shell_printf(intf, "\r\nDone Accelerometer Calibration\r\n");

  imu_get_offset(imu_get_instance(0), gyro, accl);

  shell_printf(intf, "Gyro Offset X : %d\r\n", gyro[0]);
  shell_printf(intf, "Gyro Offset Y : %d\r\n", gyro[1]);
  shell_printf(intf, "Gyro Offset Z : %d\r\n", gyro[2]);

  shell_printf(intf, "Accl Offset X : %d\r\n", accl[0]);
  shell_printf(intf, "Accl Offset Y : %d\r\n", accl[1]);
  shell_printf(intf, "Accl Offset Z : %d\r\n", accl[2]);

  shell_prompt(intf);
}

static void
shell_command_accel_calib(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "Starting Accelerometer Calibration\r\n");
  imu_calibration_init();
  imu_calibration_accel_perform(accel_calib_done, intf);
}

static void
mag_calib_done(void* arg)
{
  ShellIntf* intf = (ShellIntf*)arg;
  int16_t     bias[3];
  float       scale[3];

  shell_printf(intf, "\r\nDone Magnetometer Calibration\r\n");

  imu_get_mag_calib(imu_get_instance(0), bias, scale);

  shell_printf(intf, "Mag Bias X : %d\r\n", bias[0]);
  shell_printf(intf, "Mag Bias Y : %d\r\n", bias[1]);
  shell_printf(intf, "Mag Bias Z : %d\r\n", bias[2]);

  shell_printf(intf, "Mag Scale X : %.2f\r\n", scale[0]);
  shell_printf(intf, "Mag Scale Y : %.2f\r\n", scale[1]);
  shell_printf(intf, "Mag Scale Z : %.2f\r\n", scale[2]);
  shell_prompt(intf);
}

static void
shell_command_mag_calib(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "Starting Magnetometer Calibration\r\n");
  imu_calibration_init();
  imu_calibration_mag_perform(mag_calib_done, intf);
}
#endif

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


static void
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
