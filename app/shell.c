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
#include "imu.h"
#include "barometer.h"

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
static void shell_command_micros(ShellIntf* intf, int argc, const char** argv);
static void shell_command_micros_test(ShellIntf* intf, int argc, const char** argv);
static void shell_command_pwm_out(ShellIntf* intf, int argc, const char** argv);
static void shell_command_mag_data(ShellIntf* intf, int argc, const char** argv);
static void shell_command_bmp180(ShellIntf* intf, int argc, const char** argv);

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
    "mag_data",
    "read mag compass data",
    shell_command_mag_data,
  },
  {
    "bmp180",
    "display bmp180 data",
    shell_command_bmp180,
  },
};

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

  if(duty < PWM_MIN_DUTY_CYCLE || duty > PWM_MAX_DUTY_CYCLE)
  {
    shell_printf(intf, "Invalid duty %d\r\n", duty);
    return;
  }

  pwm_set_duty_cycle(chnl, duty);

  shell_printf(intf, "set duty cycle to %d for channel %d\r\n", duty, chnl);
}

static void
shell_command_mag_data(ShellIntf* intf, int argc, const char** argv)
{
  float data[4];

  imu_get_mag(data);

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


static inline void
shell_prompt(ShellIntf* intf)
{
  intf->put_tx_data(intf, (uint8_t*)_prompt, sizeof(_prompt) -1);
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
