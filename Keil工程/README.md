# 第16队 STM32F103C8T6 自动行驶小车 Keil5 工程

## 1. 打开与编译

1. 安装 Keil MDK5 和 `Keil.STM32F1xx_DFP` 器件包（STM32F103C8）。
2. 双击 `Project.uvprojx` 打开工程。
3. 在 Options for Target → Device 中确认器件为 `STM32F103C8`。
4. 编译：`Project → Build Target`（F7）。
5. 下载：ST-Link 接 `SWDIO / SWCLK / GND / 3.3V`，`Flash → Download`（F8）。
6. 串口：PA9→CH340 RX，PA10→CH340 TX，GND 共地，115200 8N1。

工程使用 ARMCC V5（`uAC6=0`）。如果本机只有 ARMCLANG V6，Keil 会提示迁移，
选择“Use default compiler version 6”即可；源码为 C90 风格，不依赖编译器扩展。

已用 Keil MDK 5.36 + `Keil.STM32F1xx_DFP 2.4.1` 实测编译：
**0 Error / 0 Warning**，Code 13,986 B、RO 2,190 B、RW 172 B、ZI 1,636 B，
Flash 约 16.4 KB、RAM 约 1.8 KB，在 MDK-Lite 32 KB 限制内。
`Project.hex` 是这次编译出的现成固件，可直接用 STM32CubeProgrammer/ST-Link Utility 烧录；
改代码后按 F7 会重新生成 `Objects\Project.hex`。

注意：工程器件是 `STM32F103C8`（ST）。如果你在 Options 里看到 `APM32F103C8`
（Geehy 兼容片），说明打开的是旧副本；以本目录的工程为准。

## 2. 目录

| 目录 | 内容 |
|---|---|
| `Start/` | 启动文件、CMSIS、`stm32f10x.h` |
| `Library/` | STM32F10x StdPeriph V3.5.0 |
| `System/` | DWT 微秒延时（可在中断内使用） |
| `Hardware/` | 软件 I2C、MPU6050、OLED、OLED 汉字点阵 |
| `Bsp/` | GPIO、PWM、编码器、ADC、USART、5 ms 时基 |
| `App/` | 配置、IMU、控制、路径状态机、OLED 界面、调试 |
| `User/` | `main.c`、中断向量、StdPeriph 配置 |

## 3. 控制流程

- TIM1 更新中断：200 Hz（5 ms）。
- 中断内：读 TIM3/TIM4 编码器 → 读 MPU6050 陀螺仪 Z → 积分航向 →
  路径状态机 → 双轮速度 PI → PWM。
- 主循环：按键扫描（20 ms）、OLED（5 Hz）、调试串口（10 Hz）。
- 路径：Q1 直线；Q2 直线-右弧-直线-左弧；Q3 左转 38.66°-对角线-右转 38.66°-
  右弧-右转 38.66°-对角线-左转 38.66°-左弧；Q4 重复 Q3 四次。

## 4. 首次上电必须标定

所有参数集中在 `App/app_config.h`。

### 4.1 电机方向
架空车轮，按 KEY_START 启动 Q1，观察两轮是否同向向前。
- 左轮反转：交换 TB6612 的 AO1/AO2，或把 `MOTOR_LEFT_INVERT` 改为 1。
- 右轮反转：交换 BO1/BO2，或把 `MOTOR_RIGHT_INVERT` 改为 1。

### 4.2 编码器符号
手推车前进，串口 `L=`、`R=` 应为正。若为负：
- 交换该电机 C1/C2，或把 `ENCODER_LEFT_SIGN` / `ENCODER_RIGHT_SIGN` 改为 -1。

### 4.3 轮径/计数当量
`COUNTS_PER_MM` 默认按 MG513（11 线 × 4 × 30 = 1320 count/rev）和 65 mm 轮径：
`1320 / (π×65) = 6.46 count/mm`。
实测方法：车沿直线推 1000 mm，记录串口 `d=` 的变化量 Δ，改成
`COUNTS_PER_MM = Δ / 1000`。

### 4.4 灰度方向/极性
用手在黑线与白底之间移动灰度板，串口 `g=` 的 8 位位图应翻转。
- 黑线为高（默认 `GRAY_BLACK_IS_HIGH=1`）。
- 若左右相反，把 `GRAY_REVERSE` 改为 1。
- 若某一位始终不变，检查对应 OUT 线/排线。

### 4.5 陀螺仪零偏
上电后保持静止 2 s，串口打印 `IMU calibration`。若航向漂移大：
- 检查 MPU6050 是否水平、I2C 是否稳定；
- 重新上电校准；
- 圆弧终点会自动把航向校正为 ±180°。
- 若手动左转（逆时针）时串口 `yaw` 反而变小，把 `app_config.h` 的
  `GYRO_Z_SIGN` 改为 `-1.0f`。

### 4.6 速度/航向/循迹参数
| 参数 | 默认 | 作用 |
|---|---|---|
| `SPEED_KP/KI/KFF` | 1.10 / 18 / 3.27 | 速度环 |
| `YAW_KP/KD` | 8.0 / 1.5 | 直线航向 |
| `LINE_KP/KD` | 260 / 45 | 圆弧循迹 |
| `TURN_KP` | 5.0 | 38.66°转向 |
| `SPEED_SET` | 600 mm/s | 按键可调 300~900 |

先低速（300~450 mm/s）跑通 Q1/Q2，再逐步提速；不要一开始就 900 mm/s。

## 5. 串口输出

```
t=12345 run=1 mode=3 seg=2 d=1234 yaw=-12 g=24 L=450 R=452 bat=12.1V
```

- `t`：5 ms 计数×5，单位 ms
- `run`：0 停止 / 1 运行
- `mode`：1~4 对应 Q1~Q4
- `seg`：当前片段序号
- `d`：累计里程 mm
- `yaw`：航向角（°）
- `g`：8 位灰度位图（十进制，bit0=OUT1）
- `L/R`：左右轮速 mm/s
- `bat`：电池电压

## 6. 已知假设（装车前确认）

1. 电机为 WHEELTEC MG513，11 线编码器，减速比 30，12 V。
2. 轮径 65 mm；轮距 150 mm；如实测不同，改 `app_config.h`。
3. 编码器 VCC 接 3.3 V；若该批次必须 5 V，C1/C2 需加 1 kΩ 串联和 3.3 V 钳位。
4. 灰度模块黑线输出高电平；OUT 高电平最高约 4 V，已接在 FT 引脚。
5. TB6612 模块丝印以实物为准；VM=12 V，VCC=3.3 V，STBY=3.3 V。
6. 蜂鸣器为 5 V 有源，经 S8050 驱动；无源蜂鸣器需改代码输出 2 kHz PWM。

## 7. 常见问题

| 现象 | 检查 |
|---|---|
| Keil 编译报 `DWT`/`CoreDebug` 未定义 | 已改为地址访问，确认 `System/Delay.c` 是最新版本 |
| 下载后无反应 | BOOT0=0，SWD 接线，按复位；串口是否有 `BOOT` 字样 |
| OLED 不亮 | PB8/PB9 是否接反，3.3 V，地址 0x3C |
| MPU6050 读不到 | PB10/PB11 是否接反，AD0 接地，地址 0x68 |
| 电机不转 | STBY 是否 3.3 V，VM 是否有 12 V，PWMA/PWMB 是否有波形 |
| 编码器计数不变 | 编码器 VCC/GND，C1/C2 是否接对，TIM3/TIM4 是否使能 |
| 灰度位图不变 | 5 V 供电，OUT1~8 顺序，黑线是否在探头下方 |
| 小车原地转/跑偏 | 电机方向、编码器符号、`YAW_KP`、起跑角度 |
