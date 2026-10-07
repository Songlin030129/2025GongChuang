# 2025 工创搬运机器人

基于 **STM32F407VE + FreeRTOS** 的搬运机器人下位机工程，包含四轮底盘导航、机械臂运动控制、视觉识别通信、物料抓取与堆叠，以及串口屏显示和调试功能。

机器人根据二维码给出的两轮物料顺序，依次完成原料区取料、加工区放料与取料、暂存区放料，并在第二轮进行堆叠，最后返回停止区。

## 技术与开发环境

| 项目 | 配置 |
| --- | --- |
| 主控 | STM32F407VETx，Cortex-M4，单精度 FPU |
| 系统时钟 | 168 MHz，外部晶振 8 MHz |
| 固件库 | STM32 HAL、CMSIS |
| 实时系统 | FreeRTOS，CMSIS-RTOS v2，Tick 频率 1000 Hz |
| 开发语言 | C / C++，EIDE 配置为 C99 / C++11 |
| 工程环境 | Keil MDK、VS Code + Embedded IDE（EIDE） |
| EIDE 编译器 | ARM Compiler 6（AC6） |
| 外设配置 | STM32CubeMX，配置文件为 `2025/F407VE.ioc` |
| EIDE 默认下载方式 | OpenOCD + CMSIS-DAP，Flash 起始地址 `0x08000000` |

## 主要功能

- **底盘控制**：四轮运动学解算，结合 OPS 定位实现位置、航向闭环，支持视觉误差校准。
- **路径导航**：直线与圆弧插补、速度规划，以及起点、二维码区、原料区、加工区和暂存区之间的预设路线。
- **机械臂控制**：旋转、水平伸缩、垂直升降和夹爪动作，使用梯形轨迹规划与 PID 控制。
- **视觉通信**：通过串口切换二维码、物料、目标和色块识别模式，接收物料顺序与目标位置误差。
- **组合动作**：原料区取料、车载物料取放、地面取放和第二层堆叠。
- **人机交互**：按键启动与停止控制、串口屏显示二维码顺序、VOFA 调试接口。

## 任务流程

```text
空闲等待启动
    ↓
移动到二维码区 → 读取两轮物料顺序
    ↓
移动到原料区 → 按顺序取三件物料
    ↓
移动到加工区 → 视觉校准 → 依次放料、取料
    ↓
移动到暂存区 → 视觉校准 → 依次放料
    ↓
第一轮完成：返回原料区，执行第二轮
第二轮完成：返回停止区，输出用时，进入空闲
```

第一轮在暂存区进行地面放料，第二轮调用第二层放料动作。流程由 `Main_Task.cpp` 中的状态机组织；路径和取放动作分别由 `Paths` 与 `Motion` 模块执行。

## 目录结构

```text
2025GongChuang/
├── README.md
└── 2025/
    ├── F407VE.ioc                 # CubeMX 外设配置
    ├── test.code-workspace        # VS Code 工作区
    ├── .eide/                     # EIDE 工程与编译下载配置
    ├── MDK-ARM/                   # Keil 工程与启动文件
    ├── Core/                     # 外设初始化、中断、RTOS 启动
    ├── Drivers/                  # HAL、CMSIS 驱动
    ├── Middlewares/              # FreeRTOS 中间件
    └── UserCodes/
        ├── BSP/                  # CAN、UART 回调分发
        ├── UserApps/
        │   ├── main.cpp          # 应用任务创建入口
        │   ├── Main_Task/        # 总流程状态机
        │   ├── Chassis_Task/     # 底盘控制
        │   ├── Gimbal_Task/      # 机械臂控制
        │   ├── Key_Task/         # 按键扫描与事件处理
        │   └── Debug_Task/       # 调试任务
        └── Modules/
            ├── Algorithm/        # PID、低通滤波、梯形轨迹
            ├── Navigation/       # 路径插补与预设路线
            ├── Motion/           # 组合取放动作
            ├── Motors/           # 电机与串口舵机驱动
            ├── HMI/              # 视觉模块通信
            ├── OPS/              # 定位模块通信与坐标转换
            ├── LCD/              # 串口屏显示
            ├── KEY/              # 按键事件
            ├── LED/              # 指示灯
            └── VOFA_Debug/       # 串口调试
```

## 软件启动与任务划分

启动调用链：

```text
Core/Src/main.c
  → HAL 与外设初始化
  → MX_FREERTOS_Init()
  → StartDefaultTask()
  → UserCodes/UserApps/main.cpp 中的 Main()
  → 创建五个应用任务
```

| 任务 | 职责 | 每次循环后的延时 |
| --- | --- | --- |
| MainTask | 状态切换、路线与取放动作调度 | 20 ms |
| ChassisTask | 定位反馈、底盘闭环与电机输出 | 10 ms |
| GimbalTask | 机械臂反馈、轨迹执行与电机输出 | 5 ms |
| KEYTask | 按键扫描 | 30 ms |
| DBGTask | 调试命令处理 | 20 ms |

表中延时不包含任务内部计算、通信和等待时间。CAN 接收和 UART DMA 空闲回调由 `BSP` 分发到各设备模块。

## 硬件接口

以下引脚与参数对应当前源码配置。

### CAN

两路 CAN 均配置为 **1 Mbps**。

| 总线 | TX | RX | 用途 |
| --- | --- | --- | --- |
| CAN1 | PA12 | PA11 | 四个 DJI M2006 底盘电机，编号 1～4 |
| CAN2 | PB13 | PB12 | 达妙旋转电机、ZDT 水平伸缩与垂直升降电机 |

ZDT 水平电机编号为 1，垂直电机编号为 2；达妙电机在 `Gimbal::Init()` 中以 `Init(..., 6, 5, CONTROL_MODE_MIT)` 初始化。具体收发 ID 含义见 `DMMotor` 驱动。

### UART

各串口均配置为 **115200 baud、8 位数据、无校验、1 位停止位**。

| 接口 | TX | RX | 用途 |
| --- | --- | --- | --- |
| USART1 | PA9 | PA10 | 视觉模块（HMI） |
| USART2 | PD5 | PD6 | VOFA 调试接口 |
| USART3 | PB10 | PB11 | 已初始化，应用用途待扩展 |
| UART4 | PC10 | PC11 | FSUS 串口夹爪舵机，舵机 ID 为 3 |
| UART5 | PC12 | PD2 | LCD 串口屏 |
| USART6 | PC6 | PC7 | OPS 定位模块 |

### 按键与指示灯

| 器件 | 引脚 | 当前行为 |
| --- | --- | --- |
| KEY1 | PC0 | 单击设置启动标志 |
| KEY2 | PC1 | 单击回调预留，动作代码已注释 |
| KEY3 | PC2 | 单击关闭底盘与机械臂控制使能 |
| KEY4 | PC3 | 单击调用底盘位置复位，再复位 MCU |
| LED1 | PC4 | 底盘控制循环运行指示 |
| LED2 | PC5 | 机械臂控制循环运行指示 |

## 编译与下载

### Keil MDK

1. 安装 Keil MDK 和 STM32F4xx 设备支持包。
2. 打开 `2025/MDK-ARM/F407VE.uvprojx`。
3. 选择 `F407VE` 目标，并配置工程所需的 ARM 编译器。
4. 执行 Build，生成固件。
5. 根据实际调试器设置下载接口，通过 SWD 下载并运行。

### VS Code + EIDE

1. 安装 VS Code 和 Embedded IDE（`cl.eide`）扩展。
2. 打开 `2025/test.code-workspace`。
3. 在 EIDE 中配置 ARM Compiler 6 工具链路径。
4. 确认 STM32F4xx 设备支持包可用；工程配置引用 `Keil/STM32F4xx_DFP.2.14.0`。
5. 选择 `F407VE` 目标并执行构建，输出目录配置为 `2025/build/`。
6. 默认使用 OpenOCD、`cmsis-dap` 接口与 `stm32f4x` 目标下载；使用其他调试器时，在 EIDE 中切换并配置下载方式。

## 运行与调试

1. 按接口表连接电机、定位模块、视觉模块、夹爪舵机和串口屏。
2. 根据实车安装配置电机编号、零点、运动方向和机构参数。
3. 下载固件并完成上电初始化，将机器人放置在路线对应的起始位置。
4. 按 KEY1 启动任务。识别到二维码后，串口屏显示两轮物料顺序。
5. 运行中可通过 KEY3 关闭运动控制；重新开始一整次任务前复位 MCU。

调试串口使用 USART2。`Debug_Task.cpp` 中提供了多组已注释的参数绑定与反馈输出示例，可按需要启用，用于观察定位、目标位置、视觉误差和机械臂反馈。

## 视觉通信协议

HMI 使用固定 **13 字节** 数据包，结构体按 1 字节对齐：

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| `header` | 1 字节 | 帧头 `0xFF` |
| `type` | 1 字节 | 数据类型 |
| `u_data1` | 1 字节 | 模式、顺序编号等 |
| `u_data2` | 1 字节 | 颜色、顺序编号等 |
| `f_data1` | 4 字节 | 浮点数据 1 |
| `f_data2` | 4 字节 | 浮点数据 2 |
| `footer` | 1 字节 | 帧尾 `0xFE` |

浮点字段按 STM32 内存布局直接传输，发送端应使用小端 IEEE 754 单精度格式。

| `type` | 含义 | 数据内容 |
| --- | --- | --- |
| 1 | 设置识别模式，下位机发送 | `u_data1` 为模式，`u_data2` 为颜色编号 |
| 2 | 二维码结果，下位机接收 | `u_data1`、`u_data2` 分别为第一轮、第二轮顺序编号 |
| 3 | 目标误差，下位机接收 | `f_data1`、`f_data2` 分别为 X、Y 误差 |

识别模式：`0` 不识别、`1` 二维码、`2` 物料、`3` 目标、`4` 色块。

二维码顺序编号映射：

| 编号 | 物料顺序 |
| --- | --- |
| 1 | 123 |
| 2 | 132 |
| 3 | 213 |
| 4 | 231 |
| 5 | 312 |
| 6 | 321 |

颜色使用 `1`、`2`、`3` 编号，具体颜色对应关系需与视觉端约定一致。协议实现见 `UserCodes/Modules/HMI/HMI.h` 与 `HMI.cpp`。

## 参数调整入口

| 调整内容 | 文件位置（相对 `2025/`） |
| --- | --- |
| 各区域路线与路径点 | `UserCodes/Modules/Navigation/Paths.cpp` |
| 导航速度、加速度与插补 | `UserCodes/Modules/Navigation/Navigation.h`、`Navigation.cpp` |
| 底盘轮径、轮心距离与 PID | `UserCodes/UserApps/Chassis_Task/Chassis_Task.h` |
| 机械臂预设位置、速度与 PID | `UserCodes/UserApps/Gimbal_Task/Gimbal_Task.h` |
| 取放动作顺序 | `UserCodes/Modules/Motion/Motion.cpp` |
| 总流程与视觉模式切换 | `UserCodes/UserApps/Main_Task/Main_Task.cpp` |
| OPS 安装偏移与坐标转换 | `UserCodes/Modules/OPS/OPS.cpp` |
| 调试输出与参数绑定 | `UserCodes/UserApps/Debug_Task/Debug_Task.cpp` |

路径点、机构位置和控制参数对应当前场地与实车配置，更换场地或机械结构后需重新标定。外设变更通过 `F407VE.ioc` 管理，应用逻辑主要维护在 `UserCodes/` 中。
