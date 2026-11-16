/*
 * uart_echo_test.c
 *
 * 串口自测：收到一行就回一行，用来确认
 * 树莓派 --USB--> CH340 --USART1--> STM32 --USART1--> CH340 --> 树莓派
 * 这条路是通的。
 *
 * 目前支持：
 *   $GRAB,0,320,240,0.90  ->  $ECHO,$GRAB,0,320,240,0.90
 *   $NONE                 ->  $ECHO,$NONE
 *   ECHO abc              ->  OK ECHO,abc
 *
 * 其它内容返回 0，交给原来的协议处理，不影响正常功能。
 */

#include "uart_echo_test.h"
#include "uart1_proto.h"

#include <stdio.h>
#include <string.h>

/* 回复缓冲区，够放一行就行 */
static char echo_reply[160];

int UartEchoTest_ProcessLine(const char *line)
{
  /* 空行不处理 */
  if (line == 0 || line[0] == '\0')
  {
    return 0;
  }

  /* 1) 树莓派发来的目标坐标：原样回显 */
  if (strncmp(line, "$GRAB", 5) == 0 || strncmp(line, "$NONE", 5) == 0)
  {
    snprintf(echo_reply, sizeof(echo_reply), "$ECHO,%s\r\n", line);
    Uart1Proto_Send(echo_reply);
    return 1;
  }

  /* 2) ECHO 后面跟一段文字：回 OK ECHO,文字 */
  if (strncmp(line, "ECHO", 4) == 0)
  {
    snprintf(echo_reply, sizeof(echo_reply), "OK ECHO,%s\r\n", line + 4);
    Uart1Proto_Send(echo_reply);
    return 1;
  }

  /* 3) 其它内容不管 */
  return 0;
}
