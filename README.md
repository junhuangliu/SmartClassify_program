# SmartClassify

基于 STM32F407 + FreeRTOS 的嵌入式项目，使用 STM32CubeMX 生成框架，采用 arm-none-eabi-gcc + Makefile 构建。

## 目录结构
- `Code_set_Folder/core_code/` - 主工程（Core/、Drivers/、Middlewares/、Makefile）
- `log/` - AI 任务日志（按日期命名，追加记录）
- `reference/` - 参考资料
- `前期配置.txt` - 环境搭建、调试与烧录说明

## 快速开始
```bash
cd Code_set_Folder/core_code
make        # 编译
make clean  # 清理
```

## 开发约定
- 修改代码时优先使用 `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` 区域
- `Drivers/`、`Middlewares/`、`startup_stm32f407xx.s`、`STM32F407xx_FLASH.ld`、`*.ioc` 等由 CubeMX 生成的文件一般不直接修改
- 外设变更建议通过 `core_code.ioc` 在 STM32CubeMX 中重新生成后合并用户代码
