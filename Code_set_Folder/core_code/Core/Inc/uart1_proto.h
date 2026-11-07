/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart1_proto.h
  * @brief          : USART1 上位机命令解析模块（文本协议）
  ******************************************************************************
  * @attention
  *
  * 协议约定（文本，大小写不敏感，以 \r 或 \n 结尾）：
  * - switch:1       调用原有 WaterPump_Start() 开启水泵/阀
  * - switch:0       调用原有 WaterPump_Stop() 关闭水泵/阀
  * - pump:<0~100>   不支持；原有水泵接口使用固定占空比
  * - STATUS         查询 CoreXY 通信初始化状态
 * - PING           连通测试
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

#ifdef __cplusplus
}
#endif

#endif /* __UART1_PROTO_H__ */
