# SmartClassify 项目 AGENTS 指南

## 项目概述
SmartClassify 是基于 STM32F407 + FreeRTOS 的嵌入式项目，使用 STM32CubeMX 生成框架，采用 arm-none-eabi-gcc + Makefile 构建。

## 目录结构
```
SmartClassify_program/
├── AGENTS.md        # 本文件（AI 项目指南）
├── README.md        # 项目简介（后续可补充）
├── opencode.json    # opencode 项目配置
├── 前期配置.txt     # 环境搭建与调试说明
├── log/             # 任务日志（按日期命名）
├── Code_set_Folder/
│   └── core_code/   # 主工程（Makefile、Core/、Drivers/、Middlewares/ 等）
└── reference/       # 参考资料
```

## 构建与调试
- **工程目录**：`Code_set_Folder/core_code/`
- **构建**：在 `Code_set_Folder/core_code/` 下执行 `make`（或 `make -j4`）
- **清理**：`make clean`
- **产物**：`build/core_code.elf`、`build/core_code.hex`、`build/core_code.bin`
- **调试/烧录**：参考 `前期配置.txt`，使用 OpenOCD + Cortex-Debug（需 arm-none-eabi-gdb）

## 开发约束（极为重要）

### CubeMX 生成代码约束
- **禁止随意改动自动生成区域**：`Drivers/`、`Middlewares/Third_Party/`、`startup_stm32f407xx.s`、`STM32F407xx_FLASH.ld`、`*.ioc` 等由 CubeMX/ST 生成的文件。
- **优先使用 USER CODE 区**：修改 `Core/Src/*.c`、`Core/Inc/*.h` 时，**只**在 `/* USER CODE BEGIN ... */` 和 `/* USER CODE END ... */` 之间新增/修改代码。
- **外设变更走 CubeMX**：如需新增或调整外设（UART/SPI/CAN/I2C/时钟等），应通过 `core_code.ioc` 用 STM32CubeMX 重新生成，再手动将用户代码合并到新生成的文件中。不要直接重写 `MX_*_Init` 函数的生成逻辑。

### FreeRTOS
- 当前任务定义主要在 `Core/Src/freertos.c`，入口在 `Core/Src/main.c`（`osKernelInitialize()` / `osKernelStart()`）。
- 添加新任务前，先阅读现有任务结构和 `cmsis_os2.h` 用法，遵循现有风格。

### 嵌入式注意事项
- 注意中断优先级（NVIC）、HAL 回调、RTOS 临界区。
- 修改前优先阅读相关头文件和调用链，避免破坏已有外设初始化顺序。

## 任务日志（强制）
每次完成一个**独立任务**（修复 bug、新增功能、重构、配置调整等）后，**必须**按以下规则记录日志：

- **位置**：`log/` 目录
- **命名**：使用 `YYYY-MM-DD.md`（如 `2026-10-28.md`），按日期自动创建/使用当日文件
- **方式**：**追加**到文件末尾，不得覆盖已有内容
- **时机**：任务实质性完成并做过必要的思考/修改后立即记录；若任务中断、阻塞也需记录
- **格式**（统一模板）：
```markdown
## [HH:MM:SS] <任务标题>

- **目的**: <简述要做什么>
- **涉及文件**: <如 Core/Src/main.c, Core/Src/freertos.c>
- **改动摘要**: <简短说明改动要点>
- **验证**: <如 make 编译结果、逻辑自查等>
- **结果**: <完成 | 部分完成 | 阻塞>
- **备注**: <问题、待办、注意事项（可选）>
```

## 代码习惯

### 基本原则
- **模仿优先**：新增代码前，先找同类文件看用法，尽量和周围代码风格保持一致
- **最小改动**：能不改动就不改动，只改必要的部分
- **严格用 USER CODE 区**：自定义代码一律放在 `/* USER CODE BEGIN ... */` 和 `/* USER CODE END ... */` 之间
- **不碰生成函数体**：不要修改 `MX_*_Init`、`SystemClock_Config`、`Error_Handler` 的生成逻辑

### 命名规范
- **函数**：沿用现有风格。自定义任务函数用 `XxxTask(void *argument)` 这类命名（参考 `StartDefaultTask`）
- **宏/枚举**：统一用大写加下划线，如 `LED_GPIO_Port`、`UART_BUF_SIZE`
- **变量**：局部变量用小写或小驼峰，静态变量用 `static` 修饰。全局变量尽量少用，实在要用再按文件风格处理
- **外设句柄**：沿用 CubeMX 生成的命名（如 `huart1`、`hspi1`、`hcan1`）

### C 语言细节
- **头文件包含**：优先 include 已有的头文件，只加必要的 include
- **类型**：优先使用 `stdint.h` 定义的类型（`uint8_t`、`uint16_t`、`uint32_t` 等）
- **魔法数字**：尽量用宏替换，提升可读性
- **注释**：能用清晰命名就别加注释。只在「为什么要这么做」这种不直观的逻辑时才加注释
- **简短函数**：函数职责单一，避免写超长函数

### 嵌入式 & FreeRTOS 注意事项
- **中断**：ISR 要尽可能短。耗时处理一律丢到 FreeRTOS 任务里，用队列、信号量或事件传递数据
- **中断共享变量**：如果变量会被 ISR 和任务同时访问，务必用 `volatile` 修饰
- **FreeRTOS API**：优先用 CMSIS-OS2 接口（`osThreadNew`、`osMessageQueueNew`、`osSemaphoreNew` 等），和现有 `main.c` 保持一致
- **FromISR 版 API**：在中断回调里必须用 `xxxFromISR` 系列 API，并正确处理 `pxHigherPriorityTaskWoken`
- **临界区**：能用 FreeRTOS 同步机制（队列/信号量/互斥量）解决就不要用裸临界区（`taskENTER_CRITICAL` 等）
- **栈大小**：新增任务时要评估栈用量，参考现有任务的 `stack_size` 设置
- **HAL 回调**：`HAL_UART_RxCpltCallback`、`HAL_GPIO_EXTI_Callback` 等放到合适的 `.c` 文件的 USER CODE 区，不要改 HAL 驱动本身
- **外设时序**：改动外设相关逻辑时，先理清调用顺序，避免破坏 `HAL_Init` -> `SystemClock_Config` -> 外设初始化 的顺序

## 函数注释规范（强制）

每个**函数定义**（新增或修改的自定义函数）都必须包含：

- **简洁使用说明（@brief）**：用一两句话说明函数的用途、使用场景，力求简短明了
- **详细思路（@details）**：必须说明设计意图、实现要点、注意事项。对于中断、RTOS、硬件时序相关的函数尤其要详细说明

### 模板
```c
/**
 * @brief 简洁使用说明：一句话概括函数用途、使用场景
 * 
 * @details 详细思路：
 * - 设计意图：为什么要写这个函数（解决什么问题）
 * - 实现要点：关键逻辑、算法、外设操作思路
 * - 数据流：输入来源、处理方式、输出去向（如有）
 * - 注意事项：中断/RTOS相关限制、时序要求、资源使用等
 * - 调用关系：何时调用、被谁调用、依赖哪些函数/外设（如有）
 * 
 * @param argument 参数说明（按实际情况填写）
 * @retval 返回值说明（按实际情况填写）
 */
```

### 要求
- 注释放在函数定义上方，紧贴函数
- 详细思路要讲「为什么」，而不是单纯复述代码
- 涉及 ISR、临界区、队列/信号量时，必须在详细思路中说明
- 涉及外设寄存器/HAL 调用时，说明目的和必要性
- 不要用空洞的套话（如「这是一个函数」）
- CubeMX 自动生成的函数（如 `MX_*_Init`、`SystemClock_Config`、`HAL_MspInit`）除非大幅改动，否则保持原有风格即可

## opencode 使用建议
- 工作时优先阅读本文件和 `前期配置.txt` 理解项目现状
- 涉及改动时先定位 `USER CODE` 区域
- 不要读取或建议修改 `build/`、`Drivers/`、`Middlewares/` 大量第三方代码，除非明确需要排查极特殊问题
- 改动后如需验证，可提示用户在 `Code_set_Folder/core_code/` 执行 `make`
