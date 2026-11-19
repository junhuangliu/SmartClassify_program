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
#include "uart_echo_test.h"
#include "corexy.h"
#include "WaterPump.h"
#include "cmsis_os2.h"
#include <stdio.h>
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
#define UART1_PROTO_JOB_DEPTH 2U
/* 取物区机械限位（mm），超出直接拒收，避免驱动机械臂顶到限位 */
#define UART1_PROTO_X_MAX     190U
#define UART1_PROTO_Y_MAX     260U
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
static osMessageQueueId_t proto_job_queue = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void Uart1Proto_ProcessLine(char *line);
static int Uart1Proto_StrIeq(const char *a, const char *b);
static uint8_t Uart1Proto_ParseUint(const char **cursor, uint16_t *value);
static void Uart1Proto_HandleGrabFrame(const char *frame);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 初始化命令解析模块
 *
 * @details 清空内部行缓冲和溢出标志；USART1 的 DMA 接收由 uart1_comm 负责。
 *          同时创建发送互斥锁，避免 Uart1RxTask 与 CoreXYTask 同时阻塞发送时互相踩踏，
 *          并创建抓取任务队列，把帧解析与机械动作解耦到不同任务。
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

  if (proto_job_queue == NULL)
  {
    proto_job_queue = osMessageQueueNew(UART1_PROTO_JOB_DEPTH, sizeof(Uart1Proto_Job_t), NULL);
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
 *   1. 逐字节累加到 proto_line，遇到 \r、\n 或 /（抓取帧帧尾）触发一次解析
 *   2. 连续的空行直接忽略
 *   3. 超过 UART1_PROTO_LINE_MAX 后丢弃本行，并在行尾回复 ERR LINE
 * - 注意事项：必须在任务上下文调用（内部会执行阻塞发送及泵/电机控制）
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

    if ((c == '\r') || (c == '\n') || (c == '/'))
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
 * - 设计意图：分别处理水泵开关、CoreXY 状态查询和抓取任务帧，避免各系统互相联动
 * - 实现要点：
 *   1. 用 strtok 取出首个词，再按 ':' 拆成 key/value
 *   2. 以 X 开头的词按 "X<x>,Y<y>,<shape>" 抓取帧解析，解析结果入队给 CoreXYTask
 *   3. switch:1/0 分别调用原有水泵启停函数；PUMP 占空比命令不支持
 *   4. STATUS 返回 CoreXY 初始化状态；PING 保留用于调试
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

  /* USER CODE BEGIN ECHO TEST HOOK */
  /* Temporary bring-up hook: see uart_echo_test.c. Remove this block to restore
     the original behaviour. */
  if (UartEchoTest_ProcessLine(line) != 0)
  {
    return;
  }
  /* USER CODE END ECHO TEST HOOK */

  token = strtok(line, " \t");
  if (token == NULL)
  {
    return;
  }

  /* 抓取任务帧：X<x>,Y<y>,<shape> */
  if ((token[0] == 'X') || (token[0] == 'x'))
  {
    Uart1Proto_HandleGrabFrame(token);
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
                     "STATUS COREXY=%s\r\n", CoreXY_GetStateString());
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

  if (Uart1Proto_StrIeq(key, "PUMP") != 0)
  {
    Uart1Proto_Send("ERR UNSUPPORTED\r\n");
    return;
  }

  if (Uart1Proto_StrIeq(key, "SWITCH") != 0)
  {
    if (Uart1Proto_StrIeq(value, "1") != 0)
    {
      WaterPump_Start();
      Uart1Proto_Send("OK SWITCH=1\r\n");
      return;
    }

    if (Uart1Proto_StrIeq(value, "0") != 0)
    {
      WaterPump_Stop();
      Uart1Proto_Send("OK SWITCH=0\r\n");
      return;
    }

    Uart1Proto_Send("ERR ARG\r\n");
    return;
  }

  Uart1Proto_Send("ERR CMD\r\n");
}

/**
 * @brief 解析一段十进制无符号整数
 *
 * @details
 * - 设计意图：避免在 MCU 上用 sscanf，手工解析更小更快且能严格控制错误位置
 * - 实现要点：从 *cursor 开始连续取数字，遇到非数字即停；至少 1 位数字才算成功，
 *   数值超过 uint16_t 上限直接判失败
 * - 数据流：输入为帧内字符指针，成功后把游标推进到数字之后，供逐个字段串联解析
 * - 注意事项：不跳前导空格，帧格式要求字段紧凑；不做符号解析（坐标/形状都非负）
 *
 * @param cursor 输入输出参数，指向当前待解析位置
 * @param value  输出参数，解析得到的数值
 * @retval 1 解析成功；0 无数字或溢出
 */
static uint8_t Uart1Proto_ParseUint(const char **cursor, uint16_t *value)
{
  const char *p = *cursor;
  uint32_t result = 0U;
  uint8_t digits = 0U;

  while ((*p >= '0') && (*p <= '9'))
  {
    result = (result * 10U) + (uint32_t)(*p - '0');
    if (result > 0xFFFFU)
    {
      return 0U;
    }
    digits++;
    p++;
  }

  if (digits == 0U)
  {
    return 0U;
  }

  *value = (uint16_t)result;
  *cursor = p;
  return 1U;
}

/**
 * @brief 解析并投递一条抓取任务帧
 *
 * @details
 * - 设计意图：把上位机 "X<x>,Y<y>,<shape>" 文本帧转成结构化任务，交给 CoreXYTask 执行
 * - 实现要点：
 *   1. 按 X→Y→shape 顺序逐字段解析，逗号与 'Y' 前缀必须存在，帧尾必须是 '\0'
 *   2. 校验坐标不超过取物区机械限位、形状号在 0~SHAPE_NULL-1 之间
 *   3. 校验通过后放入抓取任务队列（深度 UART1_PROTO_JOB_DEPTH），并回 OK GRAB
 * - 数据流：USART1 文本 -> 本函数 -> proto_job_queue -> CoreXYTask -> Grab_XY
 * - 注意事项：本函数运行在 Uart1RxTask，只做解析与入队，绝不能在这里调用 Grab_XY；
 *   队列满时回 ERR BUSY，让上位机重发，避免动作被静默丢弃
 *
 * @param frame 已去掉帧尾的帧内容，形如 "X120,Y30,3"
 * @retval 无
 */
static void Uart1Proto_HandleGrabFrame(const char *frame)
{
  Uart1Proto_Job_t job;
  const char *p = frame + 1U;
  uint16_t shape = 0U;

  if ((Uart1Proto_ParseUint(&p, &job.x) == 0U) || (*p != ','))
  {
    Uart1Proto_Send("ERR FRAME\r\n");
    return;
  }
  p++;

  if ((*p == 'Y') || (*p == 'y'))
  {
    p++;
  }
  if ((Uart1Proto_ParseUint(&p, &job.y) == 0U) || (*p != ','))
  {
    Uart1Proto_Send("ERR FRAME\r\n");
    return;
  }
  p++;

  if ((Uart1Proto_ParseUint(&p, &shape) == 0U) || (*p != '\0'))
  {
    Uart1Proto_Send("ERR FRAME\r\n");
    return;
  }

  if ((job.x > UART1_PROTO_X_MAX) ||
      (job.y > UART1_PROTO_Y_MAX) ||
      (shape >= (uint16_t)SHAPE_NULL))
  {
    Uart1Proto_Send("ERR RANGE\r\n");
    return;
  }

  job.shape = (TurnTable_Shape_t)shape;

  if ((proto_job_queue == NULL) ||
      (osMessageQueuePut(proto_job_queue, &job, 0U, 0U) != osOK))
  {
    Uart1Proto_Send("ERR BUSY\r\n");
    return;
  }

  (void)snprintf(proto_reply, sizeof(proto_reply),
                 "OK GRAB X=%u Y=%u SHAPE=%u\r\n",
                 (unsigned)job.x, (unsigned)job.y, (unsigned)shape);
  Uart1Proto_Send(proto_reply);
}

/**
 * @brief 取出上位机下发的一条抓取任务
 *
 * @details
 * - 设计意图：帧解析在 Uart1RxTask，机械动作在 CoreXYTask，靠队列解耦
 * - 实现要点：注册队列为空时立即返回 0，不阻塞调用者
 * - 注意事项：只在任务上下文调用；单消费者使用，无需额外加锁
 *
 * @param job 输出参数，取到任务时写入坐标与形状
 * @retval 1 取到任务；0 无任务或参数非法
 */
uint8_t Uart1Proto_TakeJob(Uart1Proto_Job_t *job)
{
  if ((job == NULL) || (proto_job_queue == NULL))
  {
    return 0U;
  }

  return (osMessageQueueGet(proto_job_queue, job, NULL, 0U) == osOK) ? 1U : 0U;
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

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
