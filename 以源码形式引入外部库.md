# 以源码形式将外部库引入 C 项目

本文档介绍如何在项目中以源码形式引入外部库，而不是通过包管理工具（如 npm、pip、HomeBrew 等）来安装和管理依赖。

C/C++ 项目中，由于缺乏统一、现代化的包管理工具，很多时候我们需要直接将外部库的源码引入到项目中进行编译和使用。

本仓库的 `libs` 目录下存放了这次课程中使用到的一些外部库的源码，方便大家在项目中直接使用。

流程：

1. 将库源码拷贝到项目中的合适位置
2. 在项目的构建系统中添加库源码的编译规则，主要包含：
   - 包含头文件的路径（头文件搜索路径）
   - 源文件的路径（需要编译的所有源文件）
   - 需要链接的库文件（如果有的话）
   - 其他编译选项（如宏定义、编译器选项等）
3. 有些库可能需要用户再提供一个配置文件。例如，FreeRTOS 需要一个 `FreeRTOSConfig.h` 文件（CubeMX 可以自动生成）。

## CMSIS DSP

CMSIS DSP 是 ARM 提供的一个嵌入式数字信号处理库，包含了常用的信号处理算法和函数。本节课程中，我们将用到其中的 PID 控制器算法。

本仓库 `libs\CMSIS-DSP-1.17.0` 提供了一个精简版本（只包含 PID 算法相关内容）。感兴趣的同学可以查看官方完整版本：[文档](https://arm-software.github.io/CMSIS-DSP/latest/)、[源码和示例](https://github.com/ARM-software/CMSIS-DSP)。

CMSIS DSP 中的 PID 算法是一个简单原始的实现，支持的功能很少，但比较适合初学者学习和理解 PID 控制器的基本原理。

### 引入 CMSIS DSP 库

在你的项目中新建一个 `Libs` 目录，并将本仓库 `libs\CMSIS-DSP-1.17.0` 目录整个拷贝到 `Libs` 目录下，如图：

<img src="assets\libs\arm-dsp-lib.png" alt="引入 DSP 库" width="400"/>
<br/>

在 EIDE 侧边栏更新构建配置：

<img src="assets\libs\arm-dsp-config.png" alt="引入 DSP 库" width="400"/>
<br/>

> 想一想，为什么添加的头文件搜索路径是这个？

提交 Git：

```bash
git add .
git commit -m "引入 CMSIS DSP 库"
```
