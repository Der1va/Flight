# Flight_Hal 无人机飞行器项目硬件与软件架构梳理

本文档基于当前仓库代码整理，目标是帮助快速理解这个四轴飞行器项目的硬件组成、软件分层、任务调度、数据流和控制链路。项目入口是 `Core/Src/main.c`，应用逻辑主要集中在 `Application/`、硬件接口驱动集中在 `Interface/`，公共算法集中在 `Common/`。

## 1. 项目定位

`Flight_Hal` 是一个基于 STM32F103 系列 MCU、STM32 HAL 和 FreeRTOS 的小型四轴飞行器控制项目。它接收遥控器通过 SI24R1 无线模块发来的油门、姿态和功能指令，读取 MPU6050 姿态传感器和 VL53L1X 测距传感器，经过姿态解算、PID 控制和电机混控后，通过 4 路 PWM 驱动四个电机。

项目当前具备以下主要能力：

- 遥控数据接收与链路状态判断。
- 油门解锁状态机。
- MPU6050 陀螺仪/加速度计初始化、零偏校准和 IMU 读取。
- 四元数姿态解算，输出 pitch、roll、yaw。
- pitch、roll、yaw 串级 PID 控制。
- 偏航摇杆控制转向角速度，松杆后保持新的相对航向。
- VL53L1X 测距与定高 PID。
- 四电机 PWM 混控输出。
- 当前 pitch/roll/yaw 闭环已经完成实飞调通，可以实现较平稳飞行。
- 电池电压 ADC 采样，并通过无线回传。
- LED 状态指示。
- IP5305T 电源按键模拟关机。
- FreeRTOS 栈溢出和 malloc 失败 Hook。

## 2. 总体硬件架构

### 2.1 核心硬件

| 模块 | 器件/资源 | 代码位置 | 作用 |
| --- | --- | --- | --- |
| 主控 | STM32F103xB | `Core/`、`Drivers/` | 运行 HAL、FreeRTOS、飞控应用 |
| 姿态传感器 | MPU6050 | `Interface/Int_MPU6050.c` | 输出三轴陀螺仪和三轴加速度计原始数据 |
| 测距传感器 | VL53L1X | `Interface/Int_VL53L1X.c`、`Interface/fix_height/` | 用于定高高度测量 |
| 无线通信 | SI24R1/nRF24L01 类 2.4G 模块 | `Interface/Int_SI24R1.c` | 接收遥控器数据，回传电池电压 |
| 电机驱动 | 4 路 PWM | `Interface/Int_Motor.c`、`Core/Src/tim.c` | 输出四个电机占空比 |
| 电源管理 | IP5305T 类电源芯片按键控制 | `Interface/Int_IP5305T.c` | 模拟按键实现关机 |
| 电池检测 | ADC1 IN9 | `Interface/Int_BAT_ADC.c` | 读取电池分压后的电压 |
| 调试串口 | USART2 | `Common/common_debug.c` | `printf`/`Debug_Printf` 输出 |
| 状态灯 | LED1-LED4 | `Interface/Int_LED.c` | 显示遥控连接和飞行状态 |

### 2.2 MCU 外设与引脚分配

| 功能 | STM32 外设 | 引脚 | 说明 |
| --- | --- | --- | --- |
| MPU6050 | I2C1 400 kHz | PB6=SCL, PB7=SDA | 姿态传感器总线 |
| VL53L1X | I2C2 400 kHz | PB10=SCL, PB11=SDA | 测距传感器总线 |
| SI24R1 SPI | SPI1 | PA5=SCK, PA6=MISO, PA7=MOSI | 无线模块数据总线 |
| SI24R1 CS | GPIO | PA4 | 软件片选 `SPI1_NSS` |
| SI24R1 CE | GPIO | PA8 | 收发使能 `SI_EN` |
| 电机 1 | TIM3_CH1 | PB4 | `Left_Motor_top` |
| 电机 2 | TIM4_CH4 | PB9 | `Left_Motor_bottom` |
| 电机 3 | TIM2_CH2 | PA1 | `Right_Motor_top` |
| 电机 4 | TIM1_CH3 | PA10 | `Right_Motor_bottom` |
| 电池电压 | ADC1_IN9 | PB1 | 连续转换，软件读取 |
| 电池检测使能 | GPIO | PB5 | `BAT_ADC_EN`，低电平使能 |
| 电源按键 | GPIO | PB2 | `POWER_KEY` |
| VL53L1X 复位 | GPIO | PB12 | `VX_XSHUT` |
| LED1 | GPIO | PA12 | 遥控连接指示 |
| LED2 | GPIO | PA11 | 遥控连接指示 |
| LED3 | GPIO | PB15 | 飞行状态指示 |
| LED4 | GPIO | PB14 | 飞行状态指示 |
| 调试串口 | USART2 | PA2=TX, PA3=RX | 115200 8N1 |

### 2.3 PWM 参数

四路电机 PWM 使用 `TIM1`、`TIM2`、`TIM3`、`TIM4`，配置相同：

- 预分频：`4 - 1`
- 自动重装载：`1000 - 1`
- PWM 模式：`PWM1`
- 初始比较值：`0`
- 应用层最终把电机速度限制到 `0 ~ 700`

在 72 MHz 系统时钟、APB 定时器倍频规则下，实际 PWM 频率需要结合对应定时器输入时钟计算。当前代码层面使用 `__HAL_TIM_SET_COMPARE()` 直接写比较值。

## 3. 软件分层

项目可以按以下层次理解：

```text
Application/
  飞控任务、通信任务、电源任务、LED任务、状态机、混控

Common/
  PID、IMU姿态解算、滤波、调试输出、公共数据结构

Interface/
  MPU6050、SI24R1、VL53L1X、电机、LED、电池ADC、电源芯片接口

Core/
  STM32CubeMX生成的 main、GPIO、I2C、SPI、TIM、ADC、USART 初始化

FreeRTOS/
  调度器、任务、队列、信号量、heap_4、Cortex-M3 port

Drivers/
  STM32 HAL 与 CMSIS
```

### 3.1 启动流程

启动路径如下：

```text
main()
  HAL_Init()
  SystemClock_Config()
  MX_GPIO_Init()
  MX_USART2_UART_Init()
  MX_TIM1_Init()
  MX_TIM2_Init()
  MX_TIM3_Init()
  MX_TIM4_Init()
  MX_SPI1_Init()
  MX_I2C1_Init()
  MX_I2C2_Init()
  MX_ADC1_Init()
  Int_SI24R1_Init()
  App_FreeRTOS_Start()
    创建 FreeRTOS 任务
    vTaskStartScheduler()
```

注意：`Int_SI24R1_Init()` 在调度器启动前执行，使用 `HAL_Delay()` 等待模块检查通过；MPU6050 和 VL53L1X 初始化则在 `Flight_Task` 中执行。

## 4. FreeRTOS 任务架构

任务创建集中在 `Application/Application.c`。

| 任务 | 周期/等待 | 优先级 | 栈大小 | 主要职责 |
| --- | --- | --- | --- | --- |
| `Power_Task` | 等待信号量，最长 10000 tick | 4 | 128 | 收到关机信号后调用 `Int_IP5305T_Stop()` |
| `comm_Task` | `vTaskDelay(10)` | 4 | 512 | 接收遥控数据、更新连接/飞行状态、处理关机、采样并准备回传电压 |
| `Flight_Task` | `vTaskDelayUntil(..., 6)` | 3 | 512 | 姿态解算、PID、定高 PID、混控和 PWM 输出 |
| `LED_Task` | `vTaskDelayUntil(..., 100)` | 1 | 128 | 根据遥控连接和飞行状态刷新 LED |

### 4.1 任务间共享数据

主要共享变量：

| 变量 | 定义位置 | 作用 |
| --- | --- | --- |
| `remote_state` | `Application/Application.c` | 遥控连接状态 |
| `flight_state` | `Application/Application.c` | 飞行器状态 |
| `remote_data` | `Application/Application.c` | 遥控器发送的油门、姿态、功能指令 |
| `fix_height` | `Application/Application.c` | 定高目标高度 |
| `back_buff` | `Application/Application.c` | 无线回传缓冲区 |

当前共享变量多数直接读写，没有使用互斥锁保护。由于任务逻辑比较简单、数据宽度较小，这种写法便于理解；如果后续扩大功能，`remote_data` 的快照读取、一次性指令位清除等位置需要重点留意并发一致性。

## 5. 遥控通信链路

### 5.1 SI24R1 配置

SI24R1 驱动位于 `Interface/Int_SI24R1.c`。

关键参数：

- 地址宽度：`5` 字节。
- Payload 宽度：`17` 字节。
- 地址：`0x0A 0x01 0x06 0x0E 0x01`。
- RF 信道：`40`。
- 自动应答：开启。
- RX 模式：`CONFIG = 0x0f`。
- TX 模式：`CONFIG = 0x0e`。
- RF_SETUP：`0x06`。

接收流程：

```text
Int_SI24R1_RxPacket()
  读取 STATUS 和 FIFO_STATUS
  如果 RX_DR 有效，或 RX FIFO 非空：
    读取 17 字节 payload
    FLUSH_RX
    返回成功
  否则返回失败
```

通信任务每轮调用 `App_Receive_data()`，接收成功后会短暂切到 TX 模式，把 `back_buff` 回传，再切回 RX 模式。

### 5.2 遥控数据帧

遥控数据解析位于 `Application/Application_receive.c`。

接收帧长度为 `17` 字节：

| 字节 | 含义 |
| --- | --- |
| 0 | 帧头 `'c'` |
| 1 | 帧头 `'z'` |
| 2 | 帧头 `'t'` |
| 3-4 | `thr` 油门，高字节在前 |
| 5-6 | `yaw` |
| 7-8 | `pitch` |
| 9-10 | `roll` |
| 11 | `shut_down` 关机指令 |
| 12 | `fix_height` 定高切换指令 |
| 13-16 | 前 13 字节累加和，32 位高字节在前 |

解析成功后写入：

```c
remote_data.thr
remote_data.yaw
remote_data.pitch
remote_data.roll
remote_data.shut_down
remote_data.fix_height
```

### 5.3 回传数据帧

当前回传缓冲区 `back_buff` 只填充电池电压：

| 字节 | 含义 |
| --- | --- |
| 0 | `'B'` |
| 1 | `'V'` |
| 2 | 电池电压 mV 高字节 |
| 3 | 电池电压 mV 低字节 |
| 4-16 | 置 0 |

`Int_BAT_ADC_Read_mV()` 使用 ADC 原始值换算：

```text
voltage_mV = ADC / 4095 * 3300 * 2
```

这里的 `*2` 表示硬件上存在 1:1 分压。

## 6. 飞行状态机

飞行状态定义在 `Common/common_config.h`：

```text
FLIGHT_STATE_IDLE      空闲/待解锁
FLIGHT_STATE_NORMAL    正常飞行
FLIGHT_STATE_STOPPED   当前代码中实际表示定高模式
FLIGHT_STATE_ERROR     遥控断连后的错误/降落处理
```

### 6.1 油门解锁状态机

解锁逻辑位于 `App_process_unlock()`：

```text
FREE
  油门 >= 900 -> MAX，记录进入时间

MAX
  油门离开高位：
    如果高位保持 >= 1000 tick -> LEAVE_MAX
    否则回到 FREE

LEAVE_MAX
  油门 <= 100 -> MIN，记录进入时间

MIN
  低位保持 >= 1000 tick -> UNLOCK
  期间油门离开低位 -> FREE

UNLOCK
  解锁完成
```

`FLIGHT_STATE_IDLE` 下，解锁成功后进入 `FLIGHT_STATE_NORMAL`。

### 6.2 飞行状态转移

```text
IDLE
  解锁成功 -> NORMAL

NORMAL
  remote_data.fix_height == 1 -> STOPPED，并清除 fix_height 指令位
  remote_state == DISCONNECTED -> ERROR

STOPPED
  持续更新 fix_height 为当前 VL53L1X 距离
  remote_data.fix_height == 1 -> NORMAL，并清除 fix_height 指令位
  remote_state == DISCONNECTED -> ERROR

ERROR
  等待飞控任务把四个电机逐步降到 0 并通知通信任务
  然后回到 IDLE
```

遥控连接判断由 `App_process_connect_state()` 完成。接收成功时立即进入 `REMOTE_STATE_CONNECTED` 并清零 `retry_count`；接收失败累计到 `MAX_RETRY_COUNT = 20` 后进入 `REMOTE_STATE_DISCONNECTED`。

## 7. 姿态解算与控制链路

### 7.1 传感器初始化

`App_Flight_Start()` 依次完成：

```text
Int_MPU6050_Init()
启动四路电机 PWM
Int_VL53L1X_Init()
```

MPU6050 初始化包含：

- 复位并等待 `PWR_MGMT_1` 读到 `0x40`。
- 退出休眠。
- 陀螺仪量程设置为 ±2000 dps。
- 加速度计量程设置为 ±2g。
- 采样率配置为 500 Hz。
- DLPF 配置为 184 Hz。
- 等待静止并采集 100 个样本做零偏校准。

VL53L1X 初始化包含：

- 通过 `VX_XSHUT` 复位。
- 等待 BootState。
- `VL53L1X_SensorInit()`。
- 距离模式设置为 long。
- Timing budget 和 inter-measurement 都设置为 20 ms。
- 启动测距。

### 7.2 单次飞控循环

`Flight_Task` 每 6 tick 执行一次：

```text
App_Calculate_Euler_Angles()
  读取 MPU6050 IMU 原始数据
  陀螺仪一阶低通
  加速度计 Kalman 滤波
  四元数姿态解算，得到 pitch/roll/yaw

App_flight_pid_process()
  pitch 外环角度 PID -> gyro_y 内环角速度 PID
  roll 外环角度 PID -> gyro_x 内环角速度 PID
  yaw 摇杆输入 -> 目标航向积分 -> yaw 外环角度 PID -> gyro_z 内环角速度 PID

如果 flight_state == STOPPED：
  每 4 次飞控循环执行一次定高 PID

App_flight_control_motor()
  根据状态机决定电机速度
  限幅并写入四路 PWM
```

### 7.3 PID 结构

PID 实现在 `Common/common_pid.c`。

基础 PID：

```text
error = measurement - target
integral += error
derivative = error - last_error
output = Kp * error + Ki * integral * PID_TIME + Kd * derivative / PID_TIME
```

串级 PID：

```text
外环 PID 输出 -> 内环 PID target
```

当前主要参数：

| 控制轴 | 外环 PID | 内环 PID |
| --- | --- | --- |
| pitch | `Kp=-7.0, Ki=0, Kd=0` | gyro_y: `Kp=2.7, Ki=0, Kd=0.35` |
| roll | `Kp=-7.0, Ki=0, Kd=0` | gyro_x: `Kp=2.7, Ki=0, Kd=0.35` |
| yaw | `Kp=-1.5, Ki=0, Kd=0` | gyro_z: `Kp=-3.5, Ki=0, Kd=0` |
| 定高 | `Kp=-0.2, Ki=0, Kd=0`，并叠加垂直速度阻尼 | 无内环 |

其中 pitch 目标来自：

```text
pitch_input = remote_data.pitch - 500
pitch_input 经过约 ±10 的摇杆死区
pitch_pid.target = pitch_input / 75.0
```

roll 目标来自：

```text
roll_input = remote_data.roll - 500
roll_input 经过约 ±10 的摇杆死区
roll_pid.target = roll_input / 75.0
```

偏航控制已经从早期“目标角固定为 0”的写法，改为“摇杆控制转向速度，松杆后保持新方向”的相对航向保持：

```text
yaw_input = 500 - remote_data.yaw
yaw_input 经过约 ±10 的摇杆死区

未进入 NORMAL/STOPPED 飞行状态时：
  yaw_target = euler_angles.yaw

进入 NORMAL 或 STOPPED 后：
  yaw_rate_target = yaw_input * 90.0 / 490.0
  yaw_target += yaw_rate_target * PID_TIME

yaw_pid.measurement = euler_angles.yaw
yaw_pid.target = yaw_target
gyro_z_pid.measurement = gyro_z_deg_s
Common_PID_Calculate_chain(&yaw_pid, &gyro_z_pid)
```

这样处理后，遥控 yaw 摇杆不是直接映射成一个固定角度，而是映射成最大约 `90 deg/s` 的目标偏航角速度；摇杆回中时 `yaw_target` 不再变化，外环继续把机体拉回到刚刚累积出的目标航向。由于当前硬件姿态估计没有磁力计参与，yaw 属于基于陀螺积分的相对航向保持，适合短时间抑制自旋和保持方向，但不等价于长期绝对航向锁定。

### 7.4 电机混控

四电机对象定义在 `Application/Application_flight.c`：

| 电机变量 | 定时器通道 | 引脚 |
| --- | --- | --- |
| `Left_Motor_top` | TIM3_CH1 | PB4 |
| `Left_Motor_bottom` | TIM4_CH4 | PB9 |
| `Right_Motor_top` | TIM2_CH2 | PA1 |
| `Right_Motor_bottom` | TIM1_CH3 | PA10 |

`FLIGHT_STATE_NORMAL` 下混控公式：

```text
Left_Motor_top.speed     = thr + pitch_output - roll_output + yaw_output
Left_Motor_bottom.speed  = thr - pitch_output - roll_output - yaw_output
Right_Motor_top.speed    = thr + pitch_output + roll_output - yaw_output
Right_Motor_bottom.speed = thr - pitch_output + roll_output + yaw_output
```

`FLIGHT_STATE_STOPPED` 当前实际表示定高模式，混控公式改用进入定高瞬间保存的 `fix_height_base_thr` 作为基础油门，并额外叠加高度环输出：

```text
Left_Motor_top.speed     = fix_height_base_thr + pitch_output - roll_output + yaw_output + height_output
Left_Motor_bottom.speed  = fix_height_base_thr - pitch_output - roll_output - yaw_output + height_output
Right_Motor_top.speed    = fix_height_base_thr + pitch_output + roll_output - yaw_output + height_output
Right_Motor_bottom.speed = fix_height_base_thr - pitch_output + roll_output + yaw_output + height_output
```

限幅策略：

- `pitch_output` 限制到 `-200 ~ 200`。
- `roll_output` 限制到 `-200 ~ 200`。
- `yaw_output` 限制到 `-100 ~ 100`。
- `height_output` 限制到 `-80 ~ 80`，只在定高模式下叠加到四个电机。
- 四个电机最终速度限制到 `0 ~ 700`。
- 如果 `remote_data.thr <= 50`，四个电机强制为 `0`。

`FLIGHT_STATE_ERROR` 下，四个电机速度每轮减 `2`，全部降到 `0` 后通知通信任务，让状态机回到 `IDLE`。

## 8. LED 与故障指示

LED 低电平点亮，高电平熄灭。

`LED_Task` 使用 LED1/LED2 显示遥控连接：

- `REMOTE_STATE_CONNECTED`：LED1、LED2 亮。
- `REMOTE_STATE_DISCONNECTED`：LED1、LED2 灭。

LED3/LED4 显示飞行状态：

- `IDLE`：慢闪。
- `NORMAL`：快闪。
- `STOPPED`：常亮。
- `ERROR`：熄灭。

系统 Hook：

- `vApplicationStackOverflowHook()`：关中断后点亮 LED3 并死循环。
- `vApplicationMallocFailedHook()`：关中断后点亮 LED4 并死循环。

## 9. 目录与文件角色

| 路径 | 角色 |
| --- | --- |
| `Core/Src/main.c` | 系统入口、外设初始化、FreeRTOS 启动 |
| `Core/Src/gpio.c` | GPIO 输出、LED、片选、使能脚配置 |
| `Core/Src/i2c.c` | I2C1/I2C2 配置 |
| `Core/Src/spi.c` | SPI1 配置 |
| `Core/Src/tim.c` | 四路 PWM 定时器配置 |
| `Core/Src/adc.c` | ADC1 IN9 电池采样配置 |
| `Core/Src/usart.c` | USART2 调试串口配置 |
| `Application/Application.c` | FreeRTOS 任务创建和任务主体 |
| `Application/Application_receive.c` | 遥控帧解析、连接状态、飞行状态机、解锁逻辑 |
| `Application/Application_flight.c` | 传感器读取、姿态解算调用、PID、定高、混控 |
| `Common/common_config.h` | 共享状态枚举和数据结构 |
| `Common/common_pid.c` | PID 和限幅 |
| `Common/common_imu.c` | 四元数姿态解算 |
| `Common/common_filter.c` | 低通滤波和 Kalman 滤波 |
| `Common/common_debug.c` | `printf` 到 USART2 |
| `Interface/Int_SI24R1.c` | SI24R1 SPI 驱动 |
| `Interface/Int_MPU6050.c` | MPU6050 I2C 驱动 |
| `Interface/Int_VL53L1X.c` | VL53L1X 应用封装 |
| `Interface/fix_height/` | VL53L1X 官方/移植驱动 |
| `Interface/Int_Motor.c` | PWM 电机输出封装 |
| `Interface/Int_BAT_ADC.c` | 电池 ADC 读取 |
| `Interface/Int_IP5305T.c` | 电源按键控制 |
| `Interface/Int_LED.c` | LED 控制 |
| `FreeRTOS/` | FreeRTOS 内核与 Cortex-M3 移植 |
| `CMakeLists.txt` | 用户源文件、FreeRTOS、接口层、公共层加入构建 |
| `CMakePresets.json` | Debug/Release CMake preset |

## 10. 数据流总览

```text
遥控器
  -> SI24R1 2.4G
  -> SPI1
  -> Int_SI24R1_RxPacket()
  -> App_Receive_data()
  -> remote_data / remote_state / flight_state

MPU6050
  -> I2C1
  -> Int_MPU6050_Read_IMU()
  -> 滤波
  -> Common_IMU_GetEulerAngle()
  -> euler_angles

VL53L1X
  -> I2C2
  -> Int_VL53L1X_Read_Distance()
  -> fix_height_pid

remote_data + euler_angles + imu_data + fix_height
  -> PID
  -> 电机混控
  -> TIM1/TIM2/TIM3/TIM4 PWM
  -> 四个电机

电池电压
  -> ADC1_IN9
  -> Int_BAT_ADC_Read_mV()
  -> back_buff
  -> SI24R1 回传
```

## 11. 当前实现的几个注意点

- `Application/copy.c` 看起来像历史备份文件，当前 `CMakeLists.txt` 没有把它加入构建，分析主线时应以 `Application/Application_flight.c` 为准。
- 代码里的部分中文注释显示为乱码，后续维护时建议统一源文件编码，避免误改注释或字符串。
- `FLIGHT_STATE_STOPPED` 名称容易误解，当前逻辑中它实际承担“定高模式”的含义。
- `remote_data.fix_height` 和 `remote_data.shut_down` 是一次性指令位，通信任务和状态机中会清除；后续增加任务时要注意并发读写。
- `Int_MPU6050_Init()` 含有等待静止和 100 次校准采样，飞控任务启动初期可能会明显等待。
- `Int_VL53L1X_Read_Distance()` 当前只读取距离，没有显式检查新数据就绪或清中断；定高控制若要更稳定，可以后续补充测距状态检查。
- `printf` 通过 `HAL_UART_Transmit(..., HAL_MAX_DELAY)` 阻塞输出，调试信息过多时会影响实时性。
- 当前 CMake 链接选项启用了 `-u _printf_float`，支持浮点打印，但也会增加固件体积。
