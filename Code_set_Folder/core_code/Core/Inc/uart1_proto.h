/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart1_proto.h
  * @brief          : USART1 上位机命令解析模块（文本协议）
  ******************************************************************************
  * @attention
  *
  * 协议约定（文本，大小写不敏感，以 \r 、 \n 或 / 结尾）：
  * - switch:1       调用原有 WaterPump_Start() 开启水泵/阀
  * - switch:0       调用原有 WaterPump_Stop() 关闭水泵/阀
  * - pump:<0~100>   不支持；原有水泵接口使用固定占空比
  * - STATUS         查询 CoreXY 通信初始化状态
 * - PING           连通测试
  * - X<x>,Y<y>,<shape>/  抓取任务帧：x/y 为取物点坐标(mm)，shape 为 0~5 形状号，
  *                       '/' 为帧尾，例如 X120,Y30,3/ 表示到 (120,30) 抓 3 号形状
  *
  * 全部自定义逻辑均放置于 USER CODE 区域，符合项目约束。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __UART1_PROTO_H__
#define __UART1_PROTO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "TurnTable.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 上位机下发的一次抓取任务
 *
 * @details 由 Uart1RxTask 解析帧后入队，CoreXYTask 取出并执行阻塞的 Grab_XY，
 *          形状号即转台工位号，两者共用 TurnTable_Shape_t，避免再做映射。
 */
typedef struct
{
  uint16_t x;              /* 取物点 X 坐标，单位 mm */
  uint16_t y;              /* 取物点 Y 坐标，单位 mm */
  TurnTable_Shape_t shape; /* 物料形状，取值 SHAPE_CUBOID~SHAPE_PENTA_PRISM */
} Uart1Proto_Job_t;

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief 初始化命令解析模块
 *
 * @details 清空内部行缓冲；不启动接收，USART1 接收仍由 uart1_comm 负责
 *
 * @param 无
 * @retval 无
 */
void Uart1Proto_Init(void);

/**
 * @brief 向解析模块喂入一段串口接收数据
 *
 * @details
 * - 设计意图：空闲中断可能把一条命令拆成多段，也可能一次带来多条命令
 * - 实现要点：按字节累加到行缓冲，遇到 \r 或 \n 才触发一次命令处理
 * - 注意事项：必须在 FreeRTOS 任务中调用，内部会调用阻塞发送及泵/电机控制
 *
 * @param data 本次收到的数据
 * @param len  数据长度
 * @retval 无
 */
void Uart1Proto_Feed(const uint8_t *data, uint16_t len);

/**
 * @brief 通过 USART1 发送一条字符串回复
 *
 * @details
 * - 实现要点：使用阻塞式 HAL_UART_Transmit，保证返回时缓冲区已发送完毕，
 *   避免 DMA 发送时临时缓冲区失效的问题；内部互斥锁串行化多任务发送
 * - 注意事项：回复很短，任务中阻塞时间可接受
 *
 * @param str 以 '\0' 结尾的字符串
 * @retval 无
 */
void Uart1Proto_Send(const char *str);

/**
 * @brief 取出上位机下发的一条抓取任务
 *
 * @details
 * - 设计意图：帧解析放在 Uart1RxTask，抓取动作由 CoreXYTask 执行，
 *   用单生产者/单消费者队列解耦，避免解析上下文长时间阻塞在机械动作上
 * - 实现要点：非阻塞读取，队列为空立即返回 0；任务由 X<x>,Y<y>,<shape>/ 帧产生
 * - 注意事项：仅在任务上下文调用；取出即消费，失败不重试
 *
 * @param job 输出参数，取到任务时写入坐标与形状
 * @retval 1 取到任务；0 无任务或参数非法
 */
uint8_t Uart1Proto_TakeJob(Uart1Proto_Job_t *job);

#ifdef __cplusplus
}
#endif

#endif /* __UART1_PROTO_H__ */
