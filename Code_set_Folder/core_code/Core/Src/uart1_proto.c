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
#include "Grab.h"
#include "servo.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @brief 可被 Set/MoveTo 引用的坐标点
 *
 * @details name 为上位机可发送的名称，value 指向 Grab.c 中定义的全局坐标变量；
 *          min/max 是该点的软件允许范围，Set 时据此校验：
 *          Destination 用作抓取放置点，限制在取物区；标定特殊点则可用 MoveTo 软限位。
 */
typedef struct
{
  const char *name;
  volatile int16_t *value;
  int16_t min;
  int16_t max;
} Uart1Proto_Point_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART1_PROTO_LINE_MAX  96U
#define UART1_PROTO_JOB_DEPTH 2U
/* 取物区机械限位（mm），超出直接拒收，避免驱动机械臂顶到限位 */
#define UART1_PROTO_X_MAX     190U
#define UART1_PROTO_Y_MAX     260U
/* MoveTo 软限位（mm）：比抓取区更宽，允许到达标定特殊点（X1=200、Y0=-15 等）；
   超出即拒收，防止误输入把机械臂带到极端位置 */
#define UART1_PROTO_MOVE_X_MIN (-50)
#define UART1_PROTO_MOVE_X_MAX 250
#define UART1_PROTO_MOVE_Y_MIN (-50)
#define UART1_PROTO_MOVE_Y_MAX 300
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

/* 可通过 Set,<名称>,<数值>/ 修改、可被 MoveTo 按名称引用的坐标点表；
   Destination 直接作为抓取放置点，限制在取物区；X/Y 标定点允许负值与更宽行程 */
static Uart1Proto_Point_t proto_points[] =
{
  { "Destination_X", &Destination_X, 0, (int16_t)UART1_PROTO_X_MAX },
  { "Destination_Y", &Destination_Y, 0, (int16_t)UART1_PROTO_Y_MAX },
  { "X0", &X0, UART1_PROTO_MOVE_X_MIN, UART1_PROTO_MOVE_X_MAX },
  { "Y0", &Y0, UART1_PROTO_MOVE_Y_MIN, UART1_PROTO_MOVE_Y_MAX },
  { "X1", &X1, UART1_PROTO_MOVE_X_MIN, UART1_PROTO_MOVE_X_MAX },
  { "Y1", &Y1, UART1_PROTO_MOVE_Y_MIN, UART1_PROTO_MOVE_Y_MAX },
  { "X2", &X2, UART1_PROTO_MOVE_X_MIN, UART1_PROTO_MOVE_X_MAX },
  { "Y2", &Y2, UART1_PROTO_MOVE_Y_MIN, UART1_PROTO_MOVE_Y_MAX },
};
#define UART1_PROTO_POINT_COUNT \
  ((uint8_t)(sizeof(proto_points) / sizeof(proto_points[0])))
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void Uart1Proto_ProcessLine(char *line);
static int Uart1Proto_StrIeq(const char *a, const char *b);
static uint8_t Uart1Proto_StrIeqN(const char *begin, const char *end, const char *name);
static uint8_t Uart1Proto_StartsWith(const char *text, const char *prefix);
static uint8_t Uart1Proto_ParseUint(const char **cursor, uint16_t *value);
static uint8_t Uart1Proto_ParseInt(const char **cursor, int16_t *value);
static uint8_t Uart1Proto_ParseCoord(const char **cursor, int16_t *value);
static uint8_t Uart1Proto_PostJob(const Uart1Proto_Job_t *job);
static void Uart1Proto_HandleGrabFrame(const char *args);
static void Uart1Proto_HandleMoveFrame(const char *args);
static void Uart1Proto_HandleSetFrame(const char *args);
static void Uart1Proto_HandleWaterPumpOn(void);
static void Uart1Proto_HandleWaterPumpOff(void);
static void Uart1Proto_HandleWaterPumpDuty(const char *args);
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
 *   1. 逐字节累加到 proto_line，遇到 \r、\n 或 /（任务帧帧尾）触发一次解析
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
 * - 设计意图：分别处理水泵开关、水泵调参、CoreXY 状态查询、抓取任务和移动任务，避免各系统互相联动
 * - 实现要点：
 *   1. 用 strtok 取出首个词，再按 ':' 拆成 key/value
 *   2. 以 "Grab," 开头的词按 "Grab,<x>,<y>" 解析（形状可选），结果入队给 CoreXYTask
 *   3. 以 "MoveTo," 开头的词按 "MoveTo,<x>,<y>" 解析，坐标可为数字或特殊点名称，结果入队给 CoreXYTask
 *   4. 以 "Set," 开头的词按 "Set,<名称>,<数值>" 直接改写 Grab.c 中的坐标变量
 *   5. MDown/MUp/SDown/SUp 只置执行标志位，动作分别由 GrabTask/ServoTask 执行
 *   6. Down/Up 为联动命令：同时置抓取电机与舵机1 的标志位
 *   7. WaterpumpOn / WaterpumpOff / Waterpump,<duty> 只置标志位或改变量，动作由 WaterPumpTask 执行
 *   8. switch:1/0 只置水泵启停标志，由 WaterPumpTask 执行；PUMP 占空比命令不支持
 *   9. PUMPTEST 只置开泵标志，直接连续吸水，用 WaterpumpOff/switch:0 停止
 *   9. STATUS 返回 CoreXY 初始化状态；PING 保留用于调试
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

  /* 任务帧：Grab,<x>,<y>（形状可选）与 MoveTo,<x>,<y> */
  if (Uart1Proto_StartsWith(token, "Grab,") != 0U)
  {
    Uart1Proto_HandleGrabFrame(token + 5);
    return;
  }

  if (Uart1Proto_StartsWith(token, "MoveTo,") != 0U)
  {
    Uart1Proto_HandleMoveFrame(token + 7);
    return;
  }

  if (Uart1Proto_StartsWith(token, "Set,") != 0U)
  {
    Uart1Proto_HandleSetFrame(token + 4);
    return;
  }

  /* 单动作命令：只置标志位，实际动作由 GrabTask / ServoTask 执行 */
  if (Uart1Proto_StrIeq(token, "MDown") != 0)
  {
    Grab_MotorBeginFlag = 1U;
    Uart1Proto_Send("OK MDOWN\r\n");
    return;
  }

  if (Uart1Proto_StrIeq(token, "MUp") != 0)
  {
    Grab_MotorEndFlag = 1U;
    Uart1Proto_Send("OK MUP\r\n");
    return;
  }

  if (Uart1Proto_StrIeq(token, "SDown") != 0)
  {
    Servo1_FallFlag = 1U;
    Uart1Proto_Send("OK SDOWN\r\n");
    return;
  }

  if (Uart1Proto_StrIeq(token, "SUp") != 0)
  {
    Servo1_RiseFlag = 1U;
    Uart1Proto_Send("OK SUP\r\n");
    return;
  }

  /* 联动命令：抓取电机与舵机1 同时动作 */
  if (Uart1Proto_StrIeq(token, "Down") != 0)
  {
    Grab_MotorBeginFlag = 1U;
    Servo1_FallFlag = 1U;
    Uart1Proto_Send("OK DOWN\r\n");
    return;
  }

  if (Uart1Proto_StrIeq(token, "Up") != 0)
  {
    Grab_MotorEndFlag = 1U;
    Servo1_RiseFlag = 1U;
    Uart1Proto_Send("OK UP\r\n");
    return;
  }

  /* 水泵命令：WaterpumpOn / WaterpumpOff / Waterpump,<duty> */
  if (Uart1Proto_StrIeq(token, "WaterpumpOn") != 0)
  {
    Uart1Proto_HandleWaterPumpOn();
    return;
  }

  if (Uart1Proto_StrIeq(token, "WaterpumpOff") != 0)
  {
    Uart1Proto_HandleWaterPumpOff();
    return;
  }

  if (Uart1Proto_StartsWith(token, "Waterpump,") != 0U)
  {
    Uart1Proto_HandleWaterPumpDuty(token + 10);
    return;
  }

  /* 无冒号的命令：PUMPTEST（直接开始吸水） / PING / STATUS */
  colon = strchr(token, ':');
  if (colon == NULL)
  {
    if (Uart1Proto_StrIeq(token, "PUMPTEST") != 0)
    {
      WaterPump_OffFlag = 0U;
      WaterPump_DutyUpdateFlag = 0U;
      WaterPump_OnFlag = 1U;
      Uart1Proto_Send("OK PUMPTEST ON\r\n");
      return;
    }

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

  if (Uart1Proto_StrIeq(key, "PUMPTEST") != 0)
  {
    /* 兼容旧的 PUMPTEST:<seconds> 写法，忽略秒数，直接连续吸水 */
    WaterPump_OffFlag = 0U;
    WaterPump_DutyUpdateFlag = 0U;
    WaterPump_OnFlag = 1U;
    Uart1Proto_Send("OK PUMPTEST ON\r\n");
    return;
  }

  if (Uart1Proto_StrIeq(key, "PUMP") != 0)
  {
    Uart1Proto_Send("ERR UNSUPPORTED\r\n");
    return;
  }

  if (Uart1Proto_StrIeq(key, "SWITCH") != 0)
  {
    /* 与 WaterpumpOn/Off 一样只置标志，硬件时序统一由 WaterPumpTask 执行，
       避免 Uart1RxTask 与 CoreXYTask 同时操作 TIM4 */
    if (Uart1Proto_StrIeq(value, "1") != 0)
    {
      WaterPump_OffFlag = 0U;
      WaterPump_DutyUpdateFlag = 0U;
      WaterPump_OnFlag = 1U;
      Uart1Proto_Send("OK SWITCH=1\r\n");
      return;
    }

    if (Uart1Proto_StrIeq(value, "0") != 0)
    {
      WaterPump_OnFlag = 0U;
      WaterPump_DutyUpdateFlag = 0U;
      WaterPump_OffFlag = 1U;
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
 * @brief 解析一段有符号十进制整数
 *
 * @details
 * - 设计意图：MoveTo/Set 需要支持负坐标（如 Y0=-15），无符号解析器不适用
 * - 实现要点：允许可选 '+'/'-' 前缀，之后至少 1 位数字；结果限制在 int16_t 范围，
 *   -32768 单独处理避免取负溢出；遇到非数字即停，由调用者检查后续分隔符
 * - 数据流：输入为帧内字符指针，成功后把游标推进到数字之后
 * - 注意事项：不跳前导空格；不接受纯符号或空字段
 *
 * @param cursor 输入输出参数，指向当前待解析位置
 * @param value  输出参数，解析得到的有符号数值
 * @retval 1 解析成功；0 格式非法或溢出
 */
static uint8_t Uart1Proto_ParseInt(const char **cursor, int16_t *value)
{
  const char *p = *cursor;
  uint8_t negative = 0U;
  uint8_t digits = 0U;
  uint32_t result = 0U;

  if ((*p == '-') || (*p == '+'))
  {
    negative = (*p == '-') ? 1U : 0U;
    p++;
  }

  while ((*p >= '0') && (*p <= '9'))
  {
    result = (result * 10U) + (uint32_t)(*p - '0');
    if (result > 32768U)
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

  if (negative != 0U)
  {
    *value = (int16_t)(-(int32_t)result);
  }
  else
  {
    if (result > 32767U)
    {
      return 0U;
    }
    *value = (int16_t)result;
  }

  *cursor = p;
  return 1U;
}

/**
 * @brief 解析 MoveTo 的一个坐标字段（数字或特殊点名称）
 *
 * @details
 * - 设计意图：让 MoveTo 同时兼容 MoveTo,120,30/ 与 MoveTo,X1,Y1/、MoveTo,Destination_X,Destination_Y/
 * - 实现要点：
 *   1. 先扫描到下一个 ',' 或 '\0'，得到字段文本区间 [begin, end)
 *   2. 用 Uart1Proto_StrIeqN 与 proto_points 表逐项比较，命中则取变量当前值
 *   3. 未命中名称时回退到 Uart1Proto_ParseInt 解析有符号数字
 * - 数据流：帧内字段 -> 坐标点变量当前值 或 十进制数 -> *value
 * - 注意事项：名称匹配不区分大小写；field 不允许前后空格；空字段视为非法
 *
 * @param cursor 输入输出参数，指向字段起始位置，成功后停在 ',' 或 '\0'
 * @param value  输出参数，解析得到的坐标值
 * @retval 1 解析成功；0 字段为空、名称未知且非数字、或数值溢出
 */
static uint8_t Uart1Proto_ParseCoord(const char **cursor, int16_t *value)
{
  const char *begin = *cursor;
  const char *end = begin;
  uint8_t index;

  while ((*end != ',') && (*end != '\0'))
  {
    end++;
  }

  for (index = 0U; index < UART1_PROTO_POINT_COUNT; index++)
  {
    if (Uart1Proto_StrIeqN(begin, end, proto_points[index].name) != 0U)
    {
      *value = *proto_points[index].value;
      *cursor = end;
      return 1U;
    }
  }

  return Uart1Proto_ParseInt(cursor, value);
}

/**
 * @brief 解析并投递一条抓取任务帧
 *
 * @details
 * - 设计意图：把上位机 "Grab,<x>,<y>" 文本帧转成结构化任务，交给 CoreXYTask 执行；
 *   串口不再下发形状，形状号默认取 SHAPE_CUBOID，仅用于转台工位选择
 * - 实现要点：
 *   1. 入参已跳过 "Grab," 前缀，按 x→y 顺序解析，逗号必须存在，帧尾必须是 '\0'
 *   2. 兼容旧格式 "Grab,<x>,<y>,<shape>"：y 后仍有逗号时继续解析可选形状号
 *   3. 校验坐标不超过取物区机械限位、形状号在 0~SHAPE_NULL-1 之间
 *   4. 校验通过后放入任务队列（深度 UART1_PROTO_JOB_DEPTH），并回 OK GRAB
 * - 数据流：USART1 文本 -> 本函数 -> proto_job_queue -> CoreXYTask -> Grab_XY
 * - 注意事项：本函数运行在 Uart1RxTask，只做解析与入队，绝不能在这里调用 Grab_XY；
 *   队列满时回 ERR BUSY，让上位机重发，避免动作被静默丢弃
 *
 * @param args 已去掉 "Grab," 前缀的参数字符串，形如 "120,30" 或 "120,30,3"
 * @retval 无
 */
static void Uart1Proto_HandleGrabFrame(const char *args)
{
  Uart1Proto_Job_t job;
  const char *p = args;
  uint16_t x = 0U;
  uint16_t y = 0U;
  uint16_t shape = (uint16_t)SHAPE_CUBOID;

  if ((Uart1Proto_ParseUint(&p, &x) == 0U) || (*p != ','))
  {
    Uart1Proto_Send("ERR FRAME\r\n");
    return;
  }
  p++;

  if (Uart1Proto_ParseUint(&p, &y) == 0U)
  {
    Uart1Proto_Send("ERR FRAME\r\n");
    return;
  }

  if (*p == ',')
  {
    p++;
    if ((Uart1Proto_ParseUint(&p, &shape) == 0U) || (*p != '\0'))
    {
      Uart1Proto_Send("ERR FRAME\r\n");
      return;
    }
  }
  else if (*p != '\0')
  {
    Uart1Proto_Send("ERR FRAME\r\n");
    return;
  }

  if ((x > UART1_PROTO_X_MAX) ||
      (y > UART1_PROTO_Y_MAX) ||
      (shape >= (uint16_t)SHAPE_NULL))
  {
    Uart1Proto_Send("ERR RANGE\r\n");
    return;
  }

  job.type = UART1_PROTO_JOB_GRAB;
  job.x = (int16_t)x;
  job.y = (int16_t)y;
  job.shape = (TurnTable_Shape_t)shape;

  if (Uart1Proto_PostJob(&job) == 0U)
  {
    Uart1Proto_Send("ERR BUSY\r\n");
    return;
  }

  (void)snprintf(proto_reply, sizeof(proto_reply),
                 "OK GRAB X=%u Y=%u SHAPE=%u\r\n",
                 (unsigned)x, (unsigned)y, (unsigned)shape);
  Uart1Proto_Send(proto_reply);
}

/**
 * @brief 解析并投递一条移动任务帧
 *
 * @details
 * - 设计意图：让上位机直接下发绝对坐标，手动移动机械臂到指定点；
 *   坐标既可以是数字，也可以是 Set 命令维护的特殊点名称（如 X1、Destination_Y）
 * - 实现要点：
 *   1. 入参已跳过 "MoveTo," 前缀，两个字段各自按 Uart1Proto_ParseCoord 解析：
 *      整段文本先与特殊点名称匹配，匹配不上再按有符号十进制数解析
 *   2. 用 MoveTo 专用软限位校验（允许负值与 X1=200 这类标定点），超出回 ERR RANGE
 *   3. 校验通过后放入任务队列并回 OK MOVE，速度由 CoreXYTask 固定为适中值
 * - 数据流：USART1 文本 -> 本函数 -> proto_job_queue -> CoreXYTask -> CoreXY_MoveTo
 * - 注意事项：本函数运行在 Uart1RxTask，只做解析与入队，绝不能在这里调用 CoreXY_MoveTo；
 *   特殊点名称读取的是变量当前值，Set 之后立即生效
 *
 * @param args 已去掉 "MoveTo," 前缀的参数字符串，如 "120,30"、"X1,Y1"、"Destination_X,Destination_Y"
 * @retval 无
 */
static void Uart1Proto_HandleMoveFrame(const char *args)
{
  Uart1Proto_Job_t job;
  const char *p = args;
  int16_t x = 0;
  int16_t y = 0;

  if ((Uart1Proto_ParseCoord(&p, &x) == 0U) || (*p != ','))
  {
    Uart1Proto_Send("ERR FRAME\r\n");
    return;
  }
  p++;

  if ((Uart1Proto_ParseCoord(&p, &y) == 0U) || (*p != '\0'))
  {
    Uart1Proto_Send("ERR FRAME\r\n");
    return;
  }

  if ((x < UART1_PROTO_MOVE_X_MIN) || (x > UART1_PROTO_MOVE_X_MAX) ||
      (y < UART1_PROTO_MOVE_Y_MIN) || (y > UART1_PROTO_MOVE_Y_MAX))
  {
    Uart1Proto_Send("ERR RANGE\r\n");
    return;
  }

  job.type = UART1_PROTO_JOB_MOVE;
  job.x = x;
  job.y = y;
  job.shape = SHAPE_NULL;

  if (Uart1Proto_PostJob(&job) == 0U)
  {
    Uart1Proto_Send("ERR BUSY\r\n");
    return;
  }

  (void)snprintf(proto_reply, sizeof(proto_reply),
                 "OK MOVE X=%d Y=%d\r\n", (int)x, (int)y);
  Uart1Proto_Send(proto_reply);
}

/**
 * @brief 处理 Set,<名称>,<数值>/ 命令：动态修改常用坐标点
 *
 * @details
 * - 设计意图：免去改宏重新编译，上位机可现场标定 Destination 与 X0/Y0/X1/Y1/X2/Y2
 * - 实现要点：
 *   1. 入参已跳过 "Set," 前缀，先取到下一个 ',' 之前的名称为止，再解析有符号十进制数值
 *   2. 名称大小写不敏感，必须命中 proto_points 表；未命中回 ERR NAME
 *   3. 按表中该点的 min/max 校验：Destination 限制在取物区，X/Y 标定点允许负值与更宽行程
 *   4. 成功后直接写目标变量，MoveTo 帧随后即可按名称使用新值
 * - 数据流：USART1 文本 -> proto_points[i].value -> MoveTo/抓取流程
 * - 注意事项：本函数运行在 Uart1RxTask；int16_t 单次写入在 Cortex-M 上原子
 *
 * @param args 已去掉 "Set," 前缀的参数字符串，形如 "X1,180"
 * @retval 无
 */
static void Uart1Proto_HandleSetFrame(const char *args)
{
  const char *p = args;
  const char *name_begin = p;
  int16_t value = 0;
  uint8_t index;

  while ((*p != ',') && (*p != '\0'))
  {
    p++;
  }

  if (*p != ',')
  {
    Uart1Proto_Send("ERR FRAME\r\n");
    return;
  }

  for (index = 0U; index < UART1_PROTO_POINT_COUNT; index++)
  {
    if (Uart1Proto_StrIeqN(name_begin, p, proto_points[index].name) != 0U)
    {
      break;
    }
  }

  if (index >= UART1_PROTO_POINT_COUNT)
  {
    Uart1Proto_Send("ERR NAME\r\n");
    return;
  }

  p++;

  if ((Uart1Proto_ParseInt(&p, &value) == 0U) || (*p != '\0'))
  {
    Uart1Proto_Send("ERR FRAME\r\n");
    return;
  }

  if ((value < proto_points[index].min) || (value > proto_points[index].max))
  {
    Uart1Proto_Send("ERR RANGE\r\n");
    return;
  }

  *proto_points[index].value = value;

  (void)snprintf(proto_reply, sizeof(proto_reply),
                 "OK SET %s=%d\r\n", proto_points[index].name, (int)value);
  Uart1Proto_Send(proto_reply);
}

/**
 * @brief 把解析完成的任务放入队列
 *
 * @details
 * - 设计意图：抓取帧和移动帧共用同一投递逻辑，避免重复写队列判空与错误处理
 * - 实现要点：队列未创建或已满时立即返回 0，由调用者回 ERR BUSY 让上位机重发
 * - 注意事项：只在任务上下文调用；Uart1RxTask 是唯一生产者，CoreXYTask 是唯一消费者
 *
 * @param job 待投递的任务
 * @retval 1 投递成功；0 队列不可用或已满
 */
static uint8_t Uart1Proto_PostJob(const Uart1Proto_Job_t *job)
{
  if ((proto_job_queue == NULL) || (job == NULL))
  {
    return 0U;
  }

  return (osMessageQueuePut(proto_job_queue, job, 0U, 0U) == osOK) ? 1U : 0U;
}

/**
 * @brief 处理 WaterpumpOn 命令：置开泵标志
 *
 * @details
 * - 设计意图：解析上下文只置标志位，实际启泵（含阀门时序和 200ms 延时）交给 WaterPumpTask，
 *   避免 Uart1RxTask 被水泵动作阻塞
 * - 实现要点：置 OnFlag 的同时清 OffFlag/DutyUpdateFlag，保证连续收到多条命令时"最后一条生效"
 * - 数据流：USART1 文本 -> 标志位 -> WaterPumpTask -> WaterPump_Start()
 * - 注意事项：标志为 volatile，单字节写入在 Cortex-M 上原子；只在任务上下文调用
 *
 * @param 无
 * @retval 无
 */
static void Uart1Proto_HandleWaterPumpOn(void)
{
  WaterPump_OnFlag = 1U;
  WaterPump_OffFlag = 0U;
  WaterPump_DutyUpdateFlag = 0U;
  Uart1Proto_Send("OK WATERPUMP ON\r\n");
}

/**
 * @brief 处理 WaterpumpOff 命令：置关泵标志
 *
 * @details
 * - 设计意图：同 WaterpumpOn，只置标志，实际停泵时序由 WaterPumpTask 执行
 * - 实现要点：置 OffFlag 的同时清 OnFlag/DutyUpdateFlag，保证最后一条命令生效
 * - 数据流：USART1 文本 -> 标志位 -> WaterPumpTask -> WaterPump_Stop()
 * - 注意事项：标志为 volatile；只在任务上下文调用
 *
 * @param 无
 * @retval 无
 */
static void Uart1Proto_HandleWaterPumpOff(void)
{
  WaterPump_OffFlag = 1U;
  WaterPump_OnFlag = 0U;
  WaterPump_DutyUpdateFlag = 0U;
  Uart1Proto_Send("OK WATERPUMP OFF\r\n");
}

/**
 * @brief 处理 Waterpump,<duty> 命令：动态修改水泵占空比
 *
 * @details
 * - 设计意图：让上位机在泵运行期间直接调转速，不需要先关再开
 * - 实现要点：
 *   1. 入参已跳过 "Waterpump," 前缀，解析 0~100 的十进制数，帧尾必须是 '\0'
 *   2. 直接改写 WaterPump_DutyPercent，并置 DutyUpdateFlag 通知 WaterPumpTask 刷新 CCR
 *   3. 越界回 ERR RANGE，格式错回 ERR FRAME
 * - 数据流：USART1 文本 -> WaterPump_DutyPercent + 标志 -> WaterPumpTask -> WaterPump_ApplyDuty()
 * - 注意事项：泵未运行时只保存数值，下次 WaterPump_Start() 自然使用新占空比
 *
 * @param args 已去掉 "Waterpump," 前缀的参数字符串，形如 "70"
 * @retval 无
 */
static void Uart1Proto_HandleWaterPumpDuty(const char *args)
{
  const char *p = args;
  uint16_t duty = 0U;

  if ((Uart1Proto_ParseUint(&p, &duty) == 0U) || (*p != '\0'))
  {
    Uart1Proto_Send("ERR FRAME\r\n");
    return;
  }

  if (duty > WATER_PUMP_DUTY_MAX)
  {
    Uart1Proto_Send("ERR RANGE\r\n");
    return;
  }

  WaterPump_DutyPercent = (uint8_t)duty;
  WaterPump_DutyUpdateFlag = 1U;

  (void)snprintf(proto_reply, sizeof(proto_reply),
                 "OK WATERPUMP DUTY=%u\r\n", (unsigned)duty);
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

/**
 * @brief 比较字段区间与名称是否相等（大小写不敏感）
 *
 * @details
 * - 设计意图：Set/MoveTo 的字段是从整行里截出来的区间 [begin, end)，长度不固定，
 *   直接用 StrIeq 需要先复制到临时缓冲区，这里做原地区间比较
 * - 实现要点：逐字符 toupper 比较，区间走完时要求 name 也结束；长度不同直接判不等
 * - 注意事项：begin 必须不大于 end，且都指向同一条以 '\0' 结尾的字符串
 *
 * @param begin 字段起始位置
 * @param end   字段结束位置（不含）
 * @param name  期望的名称
 * @retval 1 相等；0 不相等
 */
static uint8_t Uart1Proto_StrIeqN(const char *begin, const char *end, const char *name)
{
  while (begin != end)
  {
    if ((*name == '\0') ||
        (toupper((unsigned char)*begin) != toupper((unsigned char)*name)))
    {
      return 0U;
    }
    begin++;
    name++;
  }

  return (*name == '\0') ? 1U : 0U;
}

/**
 * @brief 大小写不敏感的前缀匹配
 *
 * @details
 * - 设计意图：抓取/移动帧以关键字加逗号开头，先匹配前缀再接参数解析，避免误判其它命令
 * - 实现要点：逐字符用 toupper 比较，prefix 走完即匹配成功，不要求 text 同时结束
 * - 注意事项：仅用于以 '\0' 结尾的行内字符串；空 prefix 视为匹配
 *
 * @param text   待检查的整行首个词，例如 "Grab,120,30,3"
 * @param prefix 期望的前缀，例如 "Grab,"
 * @retval 1 匹配；0 不匹配
 */
static uint8_t Uart1Proto_StartsWith(const char *text, const char *prefix)
{
  while (*prefix != '\0')
  {
    if (toupper((unsigned char)*text) != toupper((unsigned char)*prefix))
    {
      return 0U;
    }
    text++;
    prefix++;
  }

  return 1U;
}

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
