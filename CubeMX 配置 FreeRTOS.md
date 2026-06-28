# 使用 CubeMX 配置 FreeRTOS

### 1. 启用 FreeRTOS

在 CubeMX 中启用 FreeRTOS 功能，并选择 CMSIS-RTOS V2 接口标准：

<img src="assets/rtos/enable.png" width="600">

### 2. 配置 HAL timebase

FreeRTOS 会使用 SysTick 作为时间基准，因此我们需要将 HAL timebase 重新配置到另外的定时器。一般来说，我们选择功能最低级的 TIM 作为 HAL timebase，以将高级定时器留给其他功能使用。

对于 STM32F401CCUx，可以选择 TIM10 或 TIM11；对于 STM32F407IGHx，可以选择 TIM6 或 TIM7。

下图以 STM32F401CCUx 为例，选择 TIM11 作为 HAL timebase：

<img src="assets/rtos/HAL-timebase.png" width="600">

### 3. 提交 Git 仓库

点击 CubeMX 的 "Generate Code" 按钮，生成代码后，将代码提交到 Git 仓库中。

查看一下 Git 仓库的状态：

```bash
git status
```

提交代码：

```bash
git add .
git commit -m "初始化 FreeRTOS"
```

### 配置1ms定时器



