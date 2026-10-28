/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ws2812_spi.h
  * @brief          : WS2812 灯带驱动（SPI 模拟时序）
  ******************************************************************************
  * @attention
  *
  * 本模块使用 SPI1 的 MOSI 线模拟 WS2812 单线时序：
  * - 原理：将 1 个 WS2812 bit 编码为 1 个 SPI 字节
  * - 编码：逻辑 0 -> 0xC0（11000000），逻辑 1 -> 0xF8（11111000）
  * - 顺序：默认采用 GRB 顺序
  * - 复位：发送完成后额外发送 0x00 保持低电平 >= 50us 实现锁存
  *
  * 全部自定义逻辑均放置于 USER CODE 区域，符合项目约束。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __WS2812_SPI_H__
#define __WS2812_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported define -----------------------------------------------------------*/
#ifndef WS2812_LED_NUM
#define WS2812_LED_NUM  1  /* 灯珠数量，调试时先用 1，后续按实际数量修改 */
#endif

/* Exported types ------------------------------------------------------------*/
/**
 * @brief RGB 颜色结构体
 */
typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} WS2812_Color_t;

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief 初始化 WS2812 驱动
 * 
 * @details
 * - 设计意图：初始化缓冲区并清空所有灯珠
 * - 实现要点：调用 WS2812_Clear() 将缓冲区置零
 * - 注意事项：需在 SPI1 初始化完成后调用（MX_SPI1_Init() 已在 main 中完成）
 * 
 * @param 无
 * @retval 无
 */
void WS2812_Init(void);

/**
 * @brief 设置指定灯珠颜色
 * 
 * @details
 * - 设计意图：设置缓冲区中指定索引的 RGB 值（GRB 顺序发送）
 * - 实现要点：仅更新缓冲区，不立即发送
 * - 注意事项：index 范围 0 ~ WS2812_LED_NUM-1
 * 
 * @param index 灯珠索引
 * @param r 红色分量
 * @param g 绿色分量
 * @param b 蓝色分量
 * @retval 无
 */
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 设置所有灯珠颜色
 * 
 * @details
 * - 设计意图：一键设置整条灯带颜色
 * - 实现要点：循环调用 WS2812_SetPixel
 * 
 * @param r 红色分量
 * @param g 绿色分量
 * @param b 蓝色分量
 * @retval 无
 */
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 清空所有灯珠（全灭）
 * 
 * @details
 * - 设计意图：将所有灯珠设置为黑色
 * - 实现要点：调用 WS2812_SetAll(0,0,0)
 * 
 * @param 无
 * @retval 无
 */
void WS2812_Clear(void);

/**
 * @brief 刷新灯带显示
 * 
 * @details
 * - 设计意图：将缓冲区数据编码并通过 SPI1 发送到 WS2812 灯带
 * - 实现要点：
 *   1. 将 RGB(GRB顺序) 数据编码为 SPI 字节流（0->0xC0, 1->0xF8）
 *   2. 使用 HAL_SPI_Transmit 阻塞方式发送
 *   3. 发送额外 0x00 字节实现复位锁存（>=50us）
 * - 注意事项：
 *   1. 低频刷新场景使用阻塞方式足够，不影响其他任务
 *   2. 发送过程中会短暂阻塞调用任务，刷新时间约几十微秒
 * 
 * @param 无
 * @retval HAL_StatusTypeDef HAL_OK 成功，其他值表示失败
 */
HAL_StatusTypeDef WS2812_Show(void);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_SPI_H__ */
