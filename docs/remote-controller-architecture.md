# 无人机遥控器项目硬件软件架构梳理

本文基于当前 `Remote_Hal` 工程源码整理，主要用于快速理解遥控器端的硬件连接、软件分层、FreeRTOS 任务和无线数据链路。

## 1. 项目定位

`Remote_Hal` 是无人机遥控器端固件，运行在 STM32F103 系列 MCU 上。它负责采集摇杆和按键输入，将油门、偏航、俯仰、横滚以及功能按键状态打包，通过 SI24R1 无线模块发送给飞控端；同时接收飞控端回传的状态信息，并在 OLED 屏幕上显示遥控器/飞机状态。

整体数据路径可以概括为：

```text
摇杆 ADC + 按键 GPIO
        |
        v
输入采集与业务处理 Application_process_data
        |
        v
Remote_Data 控制数据
        |
        +--> SI24R1 无线发送 Application_transmit_data
        |
        +--> OLED 状态显示 Application_display
```

## 2. 硬件架构

### 2.1 主控与时钟

- 主控：STM32F103xB 系列，工程定义为 `STM32F103xB`。
- 系统时钟：外部 HSE，经 PLL 倍频到 72 MHz。
- RTOS Tick：FreeRTOS 配置为 1 kHz，即 1 tick = 1 ms。

### 2.2 摇杆输入

摇杆使用 ADC1 的 4 个通道采样，并通过 DMA 循环搬运到内存缓冲区。

| 功能量 | ADC 缓冲区 | ADC 通道 | MCU 引脚 |
| --- | --- | --- | --- |
| 油门 `thr` | `adc_buffer[0]` | ADC1_IN1 | PA1 |
| 偏航 `yaw` | `adc_buffer[1]` | ADC1_IN6 | PA6 |
| 俯仰 `pitch` | `adc_buffer[2]` | ADC1_IN2 | PA2 |
| 横滚 `roll` | `adc_buffer[3]` | ADC1_IN3 | PA3 |

ADC 配置要点：

- ADC1 开启扫描模式。
- 连续转换模式开启。
- 规则通道数量为 4。
- DMA 使用 `DMA1_Channel1`，循环模式，半字对齐。
- `Int_Joystick_Init()` 调用 `HAL_ADC_Start_DMA()` 后，`Int_Joystick_Read()` 直接读取 DMA 缓冲区。

### 2.3 按键输入

按键使用 GPIO 上拉输入，按下时读取为低电平。

| 按键事件 | MCU 引脚 | 代码枚举 |
| --- | --- | --- |
| 左摇杆按下 | PB2 | `KEY_LEFT_X` / `KEY_LEFT_X_LONG` |
| 右摇杆按下 | PB10 | `KEY_RIGHT_X` / `KEY_RIGHT_X_LONG` |
| 上键 | PB11 | `KEY_UP` |
| 右键 | PB12 | `KEY_RIGHT` |
| 左键 | PB13 | `KEY_LEFT` |
| 下键 | PB14 | `KEY_DOWN` |

`Int_KEY_get()` 做了简单消抖，并通过按下持续时间区分普通按键和长按事件。当前逻辑中，长按阈值约为 1000 tick，也就是 1 秒。

### 2.4 无线通信 SI24R1

SI24R1 通过 SPI1 通信。SPI1 使用重映射引脚：

| SI24R1 信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| SCK | PB3 | SPI1_SCK |
| MISO | PB4 | SPI1_MISO |
| MOSI | PB5 | SPI1_MOSI |
| CS/NSS | PA15 | 软件片选 `SPI1_NSS` |
| CE/EN | PB7 | 模块使能 `SI_EN` |

无线参数：

- 地址宽度：5 字节。
- 当前地址：`0A 01 06 0E 01`。
- 载荷宽度：17 字节。
- 射频信道：40。
- 自动应答：开启 pipe0。
- 发送后会切回接收模式，以便接收对端回传数据。

### 2.5 OLED 显示

OLED 使用 GPIO 模拟 SPI 时序驱动，不走硬件 SPI 外设。

| OLED 信号 | MCU 引脚 |
| --- | --- |
| CS | PA4 |
| SCLK | PA5 |
| SDIN/SDA | PA7 |
| RST | PB0 |
| DC | PB1 |

显示驱动维护一个 `OLED_GRAM[128][8]` 显存缓冲区，绘制字符、汉字和条形图后，通过 `OLED_Refresh_Gram()` 刷新到屏幕。

### 2.6 电源保持/唤醒控制

工程中存在 `IP5305T` 电源管理控制接口：

| 功能 | MCU 引脚 | 说明 |
| --- | --- | --- |
| `POWER_KEY` | PB15 | 周期性拉低后拉高，模拟电源按键动作 |

`power_Task` 每 10 秒调用一次 `Int_IP5305T_Start()`，当前实现为 PB15 拉低 100 ms 后释放。

### 2.7 调试串口

USART1 用于日志输出：

| 功能 | MCU 引脚 | 配置 |
| --- | --- | --- |
| USART1_TX | PA9 | 115200 8N1 |
| USART1_RX | PA10 | 115200 8N1 |

`printf`/`Debug_Printf` 最终通过 `HAL_UART_Transmit(&huart1, ...)` 阻塞发送。

## 3. 软件目录结构

```text
Remote_Hal
├── Core/                 CubeMX 生成的 HAL 初始化、main、外设配置
├── Drivers/              STM32 HAL 与 CMSIS
├── FreeRTOS/             FreeRTOS 内核、portable、heap_4
├── Common/               通用工具与调试输出
├── Interface/            板级外设接口封装
│   ├── Int_Joystick.*    摇杆 ADC/DMA 读取
│   ├── Int_KEY.*         按键扫描
│   ├── Int_SI24R1.*      SI24R1 无线驱动
│   ├── Int_IP5305T.*     电源管理按键控制
│   └── OLED/             OLED 显示底层与字库
└── Application/          应用层任务、数据处理、发送、显示
```

当前工程使用 CMake 构建，顶层目标名为 `Remote_Hal`。用户代码源文件在 `CMakeLists.txt` 中集中加入目标。

## 4. 启动流程

启动主流程位于 `Core/Src/main.c`：

1. `HAL_Init()` 初始化 HAL 和 SysTick。
2. `SystemClock_Config()` 配置 72 MHz 系统时钟。
3. 初始化 GPIO、DMA、USART1、SPI1、ADC1。
4. `Int_SI24R1_Init()` 检测并初始化 SI24R1，默认进入接收模式。
5. `App_FreeRTOS_Start()` 创建应用任务并启动调度器。

启动后代码主要运行在 FreeRTOS 任务中，`while (1)` 主循环不再承担业务逻辑。

## 5. FreeRTOS 任务架构

任务创建集中在 `Application/Application.c`。

| 任务 | 优先级 | 栈大小 | 周期 | 职责 |
| --- | ---: | ---: | ---: | --- |
| `power_Task` | 4 | 128 words | 10000 ms | 周期触发 IP5305T 电源按键 |
| `comm_Task` | 3 | 256 words | 10 ms | 打包并发送遥控数据，接收回传 |
| `key_Task` | 2 | 128 words | 20 ms | 扫描按键，产生功能事件和微调量 |
| `joystick_Task` | 2 | 128 words | 10 ms | 读取摇杆，映射并限幅到控制量 |
| `oled_Task` | 1 | 128 words | 50 ms | 初始化和刷新 OLED 显示 |

调度特点：

- 通信任务优先级高于输入处理和显示，用于保证控制数据较高频率发送。
- 摇杆任务和按键任务同优先级，分别处理连续量和离散事件。
- OLED 刷新频率较低，优先级最低。
- `remote_data` 是多个任务共享的全局结构体；一次性事件 `shut_down`、`fix_height` 在发送任务中使用临界区读取并清零。

## 6. 输入处理逻辑

### 6.1 摇杆数据处理

`APP_Process_Joystick_Data()` 完成以下步骤：

1. 从 `adc_buffer` 读取 4 路原始 ADC 值。
2. 将 0-4095 映射到 0-1000。
3. 做方向反转：当前公式为 `1000 - raw * 1000 / 4095`。
4. 扣除摇杆零偏校准值。
5. 叠加按键产生的俯仰/横滚微调量。
6. 使用 `Com_limit_int16()` 限幅到 0-1000。
7. 写入全局 `remote_data`。

控制量含义：

| 字段 | 含义 | 范围 |
| --- | --- | --- |
| `thr` | 油门 | 0-1000 |
| `yaw` | 偏航 | 0-1000 |
| `pitch` | 俯仰 | 0-1000 |
| `roll` | 横滚 | 0-1000 |

### 6.2 按键数据处理

`APP_Process_KEY_Data()` 根据按键事件修改控制状态：

| 按键 | 当前作用 |
| --- | --- |
| 上键 | 俯仰微调 +10 |
| 下键 | 俯仰微调 -10 |
| 左键 | 横滚微调 -10 |
| 右键 | 横滚微调 +10 |
| 左摇杆短按 | 置位 `remote_data.shut_down`，发送关机/停机事件 |
| 右摇杆短按 | 置位 `remote_data.fix_height`，发送定高事件 |
| 右摇杆长按 | 执行摇杆零偏校准 |

摇杆校准逻辑会连续读取 10 次摇杆值，计算平均偏移量，并清空按键微调量。

## 7. 无线数据协议

遥控器发送的数据由 `Application_transmit_data.c` 打包，固定 17 字节。

| 字节下标 | 内容 |
| ---: | --- |
| 0 | 帧头 `'c'` |
| 1 | 帧头 `'z'` |
| 2 | 帧头 `'t'` |
| 3-4 | `thr`，高字节在前 |
| 5-6 | `yaw`，高字节在前 |
| 7-8 | `pitch`，高字节在前 |
| 9-10 | `roll`，高字节在前 |
| 11 | `shut_down` 一次性事件 |
| 12 | `fix_height` 一次性事件 |
| 13-16 | 前 13 字节累加和，32 位，高字节在前 |

发送流程：

1. 从 `remote_data` 复制控制量。
2. 进入临界区读取并清零 `shut_down`、`fix_height`。
3. 计算 32 位累加校验。
4. SI24R1 切换到 TX 模式。
5. 调用 `Int_SI24R1_TxPacket()` 发送 17 字节数据。
6. SI24R1 切回 RX 模式。
7. 如果发送成功，尝试读取对端回传数据到 `post_buff`。

当前显示层会识别回传数据中以 `'B' 'V'` 开头的电压帧，并将 `post_buff[2..3]` 解释为毫伏值。

## 8. OLED 显示内容

`APP_display_Show()` 当前显示内容包括：

- 第一行：显示 3 个中文字形，字形来自 `Interface/OLED/Inf_font.c` 的 `oled_CH_1616`。
- 第二行：显示回传电压，格式如 `V:3.80V`。
- 第三行：显示 `THR:` 和油门条形图。

显示层只负责把已有数据画出来，不直接采样硬件，也不直接参与无线通信。

## 9. 关键共享数据

### 9.1 `Remote_Data`

定义于 `Application_process_data.h`：

```c
typedef struct
{
    int16_t thr;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t shut_down;
    uint8_t fix_height;
} Remote_Data;
```

该结构体是遥控器端最核心的业务数据。摇杆任务和按键任务写入它，通信任务读取它，显示任务也会读取部分字段。

### 9.2 `post_buff`

`post_buff[17]` 是无线回传数据缓冲区。通信任务接收并更新，显示任务读取其中的电压信息。

## 10. 代码阅读入口

建议按下面顺序阅读：

1. `Core/Src/main.c`：启动流程和外设初始化顺序。
2. `Application/Application.c`：FreeRTOS 任务创建和周期。
3. `Application/Application_process_data.c`：摇杆、按键到控制量的转换。
4. `Application/Application_transmit_data.c`：遥控数据打包和 SI24R1 发送流程。
5. `Interface/Int_SI24R1.c`：无线模块寄存器配置、TX/RX 切换。
6. `Interface/Int_Joystick.c`：ADC DMA 缓冲读取。
7. `Interface/Int_KEY.c`：按键扫描和长短按判断。
8. `Application/Application_display.c`：OLED 显示业务内容。
9. `Interface/OLED/Inf_OLED.c`：OLED 底层绘图和刷屏。

## 11. 当前架构特点与注意事项

- 项目已经形成了 `Core -> Interface -> Application` 的三层结构，外设细节大多被封装在 `Interface` 层。
- 控制数据发送频率理论上为 100 Hz，摇杆采样处理频率也是 100 Hz。
- `printf` 最终是阻塞 UART 发送，调试日志过多时可能影响任务实时性和栈占用。
- `remote_data` 和 `post_buff` 是跨任务共享数据，目前只对一次性事件读取清零加了临界区；若后续数据竞争变复杂，可以考虑队列、事件组或更明确的快照机制。
- OLED 与 SI24R1 共用部分 SPI 常见引脚编号含义，但 OLED 当前是 GPIO 模拟时序，SI24R1 才使用硬件 SPI1。
- `Interface/OLED/Inf_font.c` 的中文字库文件编码比较敏感，修改字模时应避免普通 UTF-8 文本往返导致乱码或字节损坏。

## 12. 一句话总览

这个遥控器固件的核心是：STM32F103 通过 ADC/DMA 采集双摇杆，通过 GPIO 扫描按键，在 FreeRTOS 任务中生成 `Remote_Data` 控制量，再用 SI24R1 以 17 字节固定帧周期发送给无人机，同时把回传电压和油门状态显示到 OLED 上。
