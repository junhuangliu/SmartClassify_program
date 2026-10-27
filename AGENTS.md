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

## opencode 使用建议
- 工作时优先阅读本文件和 `前期配置.txt` 理解项目现状
- 涉及改动时先定位 `USER CODE` 区域
- 不要读取或建议修改 `build/`、`Drivers/`、`Middlewares/` 大量第三方代码，除非明确需要排查极特殊问题
- 改动后如需验证，可提示用户在 `Code_set_Folder/core_code/` 执行 `make`
