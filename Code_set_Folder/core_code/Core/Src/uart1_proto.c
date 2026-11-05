/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart1_proto.c
  * @brief          : USART1 上位机命令解析模块实现
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "uart1_proto.h"
#include "pump.h"
#include "corexy.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART1_PROTO_LINE_MAX  96U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;

static char proto_line[UART1_PROTO_LINE_MAX];
static uint16_t proto_line_len = 0U;
static uint8_t proto_line_overflow = 0U;
static char proto_reply[64];
static osMutexId_t proto_tx_mutex = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void Uart1Proto_ProcessLine(char *line);
static int Uart1Proto_StrIeq(const char *a, const char *b);
static int Uart1Proto_ParsePercent(const char *text, uint8_t *out_percent);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 初始化命令解析模块
 *
 * @details 清空内部行缓冲和溢出标志；USART1 的 DMA 接收由 uart1_comm 负责。
 *          同时创建发送互斥锁，避免 Uart1RxTask 与 CoreXYTask 同时阻塞发送时互相踩踏。
 *
 * @param 无
 * @retval 无
 */
void Uart1Proto_Init(void)
{
  proto_line_len = 0U;
  proto_line_overflow = 0U;
  proto_line[0] = '\0';

  if (proto_tx_mutex == NULL)
  {
    proto_tx_mutex = osMutexNew(NULL);
  }
}

/**
 * @brief 通过 USART1 发送一条字符串回复
 *
 * @details
 * - 设计意图：给命令解析提供简单可靠的回复通道
 * - 实现要点：strlen 得到长度后用阻塞式 HAL_UART_Transmit 发送，超时 100ms；
 *   使用互斥锁串行化 Uart1RxTask 与 CoreXYTask 的发送
 * - 注意事项：返回时数据已发送完成，因此调用者可以使用临时/静态缓冲区
 *
 * @param str 以 '\0' 结尾的字符串
 * @retval 无
 */
void Uart1Proto_Send(const char *str)
{
  uint16_t len;

  if (str == NULL)
  {
    return;
  }

  len = (uint16_t)strlen(str);
  if (len == 0U)
  {
    return;
  }

  if ((proto_tx_mutex != NULL) &&
      (osMutexAcquire(proto_tx_mutex, osWaitForever) != osOK))
  {
    return;
  }

  HAL_UART_Transmit(&huart1, (uint8_t *)str, len, 100U);

  if (proto_tx_mutex != NULL)
  {
    (void)osMutexRelease(proto_tx_mutex);
  }
}

/**
 * @brief 向解析模块喂入一段串口接收数据
 *
 * @details
 * - 设计意图：解决空闲中断造成的拆包/粘包问题
 * - 实现要点：
 *   1. 逐字节累加到 proto_line，遇到 \r 或 \n 触发一次解析
 *   2. 连续的空行直接忽略
 *   3. 超过 UART1_PROTO_LINE_MAX 后丢弃本行，并在行尾回复 ERR LINE
 * - 注意事项：必须在任务上下文调用（内部会调用水泵动作和阻塞发送）
 *
 * @param data 本次收到的数据
 * @param len  数据长度
 * @retval 无
 */
void Uart1Proto_Feed(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if (data == NULL)
  {
    return;
  }

  for (i = 0U; i < len; i++)
  {
    char c = (char)data[i];

    if ((c == '\r') || (c == '\n'))
    {
      if (proto_line_overflow != 0U)
      {
        Uart1Proto_Send("ERR LINE\r\n");
        proto_line_overflow = 0U;
        proto_line_len = 0U;
      }
      else if (proto_line_len > 0U)
      {
        proto_line[proto_line_len] = '\0';
        Uart1Proto_ProcessLine(proto_line);
        proto_line_len = 0U;
      }
      continue;
    }

    if (proto_line_overflow != 0U)
    {
      continue;
    }

    if (proto_line_len < (UART1_PROTO_LINE_MAX - 1U))
    {
      proto_line[proto_line_len] = c;
      proto_line_len++;
    }
    else
    {
      proto_line_overflow = 1U;
      proto_line_len = 0U;
    }
  }
}

/**
 * @brief 处理一整行命令
 *
 * @details
 * - 设计意图：把文本命令翻译成水泵/CoreXY 动作，支持 `key:value` 形式
 * - 实现要点：
 *   1. 用 strtok 取出首个词，再按 ':' 拆成 key/value
 *   2. init:1 请求碰撞回零；init:0 中止回零并停止电机
 *   3. pump:<0~100> 设置占空比；switch:1/0 开关泵
 *   4. STATUS 同时返回水泵和 CoreXY 状态；PING 保留用于调试
 * - 注意事项：proto_line 会被 strtok 原地修改，只在解析期间使用
 *
 * @param line 以 '\0' 结尾的一行命令
 * @retval 无
 */
static void Uart1Proto_ProcessLine(char *line)
{
  char *token;
  char *key;
  char *value;
  char *colon;
  uint8_t duty;

  token = strtok(line, " \t");
  if (token == NULL)
  {
    return;
  }

  /* 无冒号的命令：PING / STATUS */
  colon = strchr(token, ':');
  if (colon == NULL)
  {
    if (Uart1Proto_StrIeq(token, "PING") != 0)
    {
      Uart1Proto_Send("OK PONG\r\n");
      return;
    }

    if (Uart1Proto_StrIeq(token, "STATUS") != 0)
    {
      (void)snprintf(proto_reply, sizeof(proto_reply),
                     "STATUS PUMP=%s %u COREXY=%s\r\n",
                     (Pump_IsOn() != 0U) ? "ON" : "OFF",
                     (unsigned int)Pump_GetDuty(),
                     CoreXY_GetStateString());
      Uart1Proto_Send(proto_reply);
      return;
    }

    Uart1Proto_Send("ERR CMD\r\n");
    return;
  }

  /* 拆分 key:value */
  *colon = '\0';
  key = token;
  value = colon + 1;

  if (Uart1Proto_StrIeq(key, "INIT") != 0)
  {
    if (Uart1Proto_StrIeq(value, "1") != 0)
    {
      HAL_StatusTypeDef status = CoreXY_RequestHome();
      if (status == HAL_OK)
      {
        Uart1Proto_Send("OK INIT=1\r\n");
      }
      else if (status == HAL_BUSY)
      {
        Uart1Proto_Send("ERR BUSY\r\n");
      }
      else
      {
        Uart1Proto_Send("ERR COREXY\r\n");
      }
      return;
    }

    if (Uart1Proto_StrIeq(value, "0") != 0)
    {
      if (CoreXY_RequestAbort() == HAL_OK)
      {
        Uart1Proto_Send("OK INIT=0\r\n");
      }
      else
      {
        Uart1Proto_Send("ERR COREXY\r\n");
      }
      return;
    }

    Uart1Proto_Send("ERR ARG\r\n");
    return;
  }

  if (Uart1Proto_StrIeq(key, "PUMP") != 0)
  {
    if (Uart1Proto_ParsePercent(value, &duty) == 0)
    {
      Uart1Proto_Send("ERR ARG\r\n");
      return;
    }

    Pump_SetDuty(duty);
    (void)snprintf(proto_reply, sizeof(proto_reply), "OK PUMP=%u\r\n",
                   (unsigned int)Pump_GetDuty());
    Uart1Proto_Send(proto_reply);
    return;
  }

  if (Uart1Proto_StrIeq(key, "SWITCH") != 0)
  {
    if (Uart1Proto_StrIeq(value, "1") != 0)
    {
      Pump_On();
      (void)snprintf(proto_reply, sizeof(proto_reply), "OK SWITCH=1 %u\r\n",
                     (unsigned int)Pump_GetDuty());
      Uart1Proto_Send(proto_reply);
      return;
    }

    if (Uart1Proto_StrIeq(value, "0") != 0)
    {
      Pump_Off();
      Uart1Proto_Send("OK SWITCH=0\r\n");
      return;
    }

    Uart1Proto_Send("ERR ARG\r\n");
    return;
  }

  Uart1Proto_Send("ERR CMD\r\n");
}

/**
 * @brief 大小写不敏感字符串比较
 *
 * @details 用于命令关键字比较，避免上位机大小写不一致导致解析失败
 *
 * @param a 字符串 A
 * @param b 字符串 B
 * @retval 1 相等，0 不相等
 */
static int Uart1Proto_StrIeq(const char *a, const char *b)
{
  while ((*a != '\0') && (*b != '\0'))
  {
    if (toupper((unsigned char)*a) != toupper((unsigned char)*b))
    {
      return 0;
    }
    a++;
    b++;
  }

  return ((*a == '\0') && (*b == '\0')) ? 1 : 0;
}

/**
 * @brief 解析 0~100 的百分比参数
 *
 * @details
 * - 实现要点：strtol 解析十进制整数，要求整串都被消费且范围在 0~100
 * - 注意事项：非法输入返回 0，由调用者回复 ERR ARG
 *
 * @param text        待解析字符串
 * @param out_percent 解析结果输出
 * @retval 1 成功，0 失败
 */
static int Uart1Proto_ParsePercent(const char *text, uint8_t *out_percent)
{
  char *end = NULL;
  long value;

  value = strtol(text, &end, 10);
  if ((end == text) || (*end != '\0'))
  {
    return 0;
  }
  if ((value < 0L) || (value > 100L))
  {
    return 0;
  }

  *out_percent = (uint8_t)value;
  return 1;
}

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
