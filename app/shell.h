#ifndef __SHELL_DEF_H__
#define __SHELL_DEF_H__

#define CLI_RX_BUFFER_LENGTH            64
#define SHELL_MAX_COMMAND_LEN           64

#include "list.h"

typedef struct __shell_intef ShellIntf;

struct __shell_intef
{
  uint8_t   cmd_buffer_ndx;
  int8_t    cmd_buffer[SHELL_MAX_COMMAND_LEN + 1];

  bool      (*get_rx_data)(ShellIntf* intf, uint8_t* data);
  void      (*put_tx_data)(ShellIntf* intf, uint8_t* data, uint16_t len);

  struct list_head    lh;
};

extern void shell_init(void);
extern void shell_start(void);
extern void shell_handle_rx(ShellIntf* intf);
extern void shell_if_register(ShellIntf* intf);

#endif //!__SHELL_DEF_H__
