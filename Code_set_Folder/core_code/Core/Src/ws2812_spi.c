/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ws2812_spi.c
  * @brief          : WS2812 灯带驱动实现（SPI 模拟时序）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ws2812_spi.h"
#include "string.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* SPI 编码：用 1 个 SPI 字节表示 1 个 WS2812 bit */
#define WS2812_CODE_0  0xC0  /* 11000000 - 表示逻辑 0 */
#define WS2812_CODE_1  0xF8  /* 11111000 - 表示逻辑 1 */

/* 缓冲区大小：每个灯珠 24 字节编码 + 80 字节复位 */
#define WS2812_SPI_BUF_SIZE  ((WS2812_LED_NUM * 24) + 80)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern SPI_HandleTypeDef hspi1;

static uint8_t spi_buf[WS2812_SPI_BUF_SIZE];
static uint8_t rgb_buf[WS2812_LED_NUM][3]; /* GRB 顺序存储 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 初始化 WS2812 驱动
 * 
 * @details
 * - 设计意图：初始化颜色缓冲区并清空灯带
 * - 实现要点：调用 WS2812_Clear() 将所有灯珠置为黑色
 * - 注意事项：hspi1 已由 MX_SPI1_Init() 初始化
 * 
 * @param 无
 * @retval 无
 */
void WS2812_Init(void)
{
  /* USER CODE BEGIN WS2812_Init 0 */

  /* USER CODE END WS2812_Init 0 */

  WS2812_Clear();
  
  /* USER CODE BEGIN WS2812_Init 1 */

  /* USER CODE END WS2812_Init 1 */
}

/**
 * @brief 设置指定灯珠颜色
 * 
 * @details
 * - 设计意图：设置缓冲区中指定索引的 RGB 值
 * - 实现要点：存储为 GRB 顺序（WS2812 常用 GRB 格式）
 * - 注意事项：index 超出范围则不处理
 * 
 * @param index 灯珠索引
 * @param r 红色分量
 * @param g 绿色分量
 * @param b 蓝色分量
 * @retval 无
 */
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
  /* USER CODE BEGIN WS2812_SetPixel 0 */

  /* USER CODE END WS2812_SetPixel 0 */

  if (index >= WS2812_LED_NUM)
  {
    return;
  }
  
  /* WS2812 使用 GRB 顺序 */
  rgb_buf[index][0] = g;
  rgb_buf[index][1] = r;
  rgb_buf[index][2] = b;

  /* USER CODE BEGIN WS2812_SetPixel 1 */

  /* USER CODE END WS2812_SetPixel 1 */
}

/**
 * @brief 设置所有灯珠颜色
 * 
 * @details
 * - 设计意图：一键设置整条灯带颜色
 * - 实现要点：遍历所有灯珠调用 WS2812_SetPixel
 * 
 * @param r 红色分量
 * @param g 绿色分量
 * @param b 蓝色分量
 * @retval 无
 */
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
  /* USER CODE BEGIN WS2812_SetAll 0 */

  /* USER CODE END WS2812_SetAll 0 */

  for (uint16_t i = 0; i < WS2812_LED_NUM; i++)
  {
    WS2812_SetPixel(i, r, g, b);
  }

  /* USER CODE BEGIN WS2812_SetAll 1 */

  /* USER CODE END WS2812_SetAll 1 */
}

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
void WS2812_Clear(void)
{
  /* USER CODE BEGIN WS2812_Clear 0 */

  /* USER CODE END WS2812_Clear 0 */

  WS2812_SetAll(0, 0, 0);

  /* USER CODE BEGIN WS2812_Clear 1 */

  /* USER CODE END WS2812_Clear 1 */
}

/**
 * @brief 刷新灯带显示
 * 
 * @details
 * - 设计意图：将缓冲区数据编码并通过 SPI1 发送到 WS2812 灯带
 * - 实现要点：
 *   1. 将 RGB(GRB顺序) 数据编码为 SPI 字节流（bit 0->0xC0, bit 1->0xF8）
 *   2. 使用 HAL_SPI_Transmit 阻塞方式发送
 *   3. 发送额外 0x00 字节实现复位锁存（>=50us）
 * - 注意事项：
 *   1. 编码方式：先发最高位 MSB
 *   2. 若颜色异常可尝试切换编码（0xE0/0xFC）或调整 SPI 波特率
 *   3. 低频刷新场景阻塞方式足够
 * 
 * @param 无
 * @retval HAL_StatusTypeDef HAL_OK 成功，其他值表示失败
 */
HAL_StatusTypeDef WS2812_Show(void)
{
  /* USER CODE BEGIN WS2812_Show 0 */

  /* USER CODE END WS2812_Show 0 */

  uint16_t buf_idx = 0;
  
  /* 编码所有灯珠数据 */
  for (uint16_t led = 0; led < WS2812_LED_NUM; led++)
  {
    /* 按 GRB 顺序编码：G、R、B 各 8 bit */
    for (uint8_t color = 0; color < 3; color++)
    {
      uint8_t byte = rgb_buf[led][color];
      /* 从高位到低位编码每个 bit */
      for (int8_t bit = 7; bit >= 0; bit--)
      {
        if (byte & (1 << bit))
        {
          /* 逻辑 1 */
          spi_buf[buf_idx++] = WS2812_CODE_1;
        }
        else
        {
          /* 逻辑 0 */
          spi_buf[buf_idx++] = WS2812_CODE_0;
        }
      }
    }
  }
  
  /* 添加复位码：发送 0x00 保持低电平 >= 50us */
  for (uint16_t i = 0; i < 80; i++)
  {
    spi_buf[buf_idx++] = 0x00;
  }
  
  /* 通过 SPI1 发送数据 */
  HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi1, spi_buf, buf_idx, HAL_MAX_DELAY);
  
  return ret;

  /* USER CODE BEGIN WS2812_Show 1 */

  /* USER CODE END WS2812_Show 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
