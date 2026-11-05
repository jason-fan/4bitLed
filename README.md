---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 495757a73352da7aae1b306638d4f5fe_6fc58cb4d39611f1a0da525400393706
    ReservedCode1: spjYox+UeoGO6KI5I6kQ/GjVQ2qch4gHHmpgQUSwn4dH1fvuJt9lZjSnBo9rgf3W9RPo3Rs2CbzxE/MSrPWRRWD2VUQHyEQWil+b+kXqnUzx8tYNC4QuJjvw8J8Zo7WHeB1yNmt/3t4hFF+jRWYOolBEujVR1eYwJdXnbcdk+ypm1LF3eAGI4wuKrIs=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 495757a73352da7aae1b306638d4f5fe_6fc58cb4d39611f1a0da525400393706
    ReservedCode2: spjYox+UeoGO6KI5I6kQ/GjVQ2qch4gHHmpgQUSwn4dH1fvuJt9lZjSnBo9rgf3W9RPo3Rs2CbzxE/MSrPWRRWD2VUQHyEQWil+b+kXqnUzx8tYNC4QuJjvw8J8Zo7WHeB1yNmt/3t4hFF+jRWYOolBEujVR1eYwJdXnbcdk+ypm1LF3eAGI4wuKrIs=
---









# project3 - STM8S103F3P6 I2C 从机 + 四位共阴 8 段数码管

在 D:\stm8s_project\project2 基础上，以相同风格（SDCC + 面向对象结构体/函数指针、无官方库、无第三方依赖）新建的 I2C 从机 + 数码管显示工程。

## 硬件接线（TSSOP20）

| 功能 | 引脚 | 说明 |
|---|---|---|
| I2C SDA | PB4 | 硬件 I2C 从机数据线（开漏，需外部 4.7k 上拉） |
| I2C SCL | PB5 | 硬件 I2C 从机时钟线（开漏，需外部 4.7k 上拉） |
| 段选 a | PA1 | 共阴数码管 a 段（经限流电阻接阳极） |
| 段选 b | PA2 | 共阴数码管 b 段 |
| 段选 c | PA3 | 共阴数码管 c 段 |
| 段选 d | PC4 | 共阴数码管 d 段 |
| 段选 e | PC3 | 共阴数码管 e 段 |
| 段选 f | PC5 | 共阴数码管 f 段 |
| 段选 g | PC7 | 共阴数码管 g 段 |
| 段选 dp | PC6 | 小数点段 |
| 位选 1 | PD3 | 第 1 位公共阴极（低电平选中） |
| 位选 2 | PD2 | 第 2 位公共阴极（低电平选中） |
| 位选 3 | PD1 | 第 3 位公共阴极（低电平选中；**PD1 与 SWIM 调试脚共用**，占用后无法在线 SWIM 调试——这是 12 个 LED GPIO 与 I2C/UART 在 20 脚封装共存下的唯一可行方案） |
| 位选 4 | PD4 | 第 4 位公共阴极（低电平选中） |
| UART TX | PD5 | 115200 8N1 调试输出 |
| UART RX | PD6 | 115200 8N1 调试输入（回显） |

> IO 资源评估：四位共阴 8 段数码管需要 8 段选 + 4 位选 = 12 个 GPIO。
> TSSOP20 实际引出的 GPIO 仅为 PA1/PA2/PA3、PB4/PB5、PC3/PC4/PC5/PC6/PC7、
> PD1(SWIM)/PD2/PD3/PD4/PD5/PD6（PD0、PD7、PB0-PB3、PC0-PC2 均未引出）。
> 本设计段选用 PA1/PA2/PA3/PC4/PC3/PC5/PC7/PC6（按 a..g,dp 顺序）、位选用 PD3/PD2/PD1/PD4（DIG1..DIG4 顺序），
> **完全避开** I2C（PB4/PB5）与 UART（PD5/PD6），因此 UART 调试输出可保留；
> 这是 12 个 LED GPIO 与 I2C/UART 在 20 脚封装共存下的唯一可行方案。
> 注意 PD1 与 SWIM 调试脚共用，占用后无法在线 SWIM 调试。
> 大电流/大尺寸数码管建议位选端加 NPN 三极管驱动，小尺寸可直接 GPIO 驱动。

## 功能

- 硬件 I2C 从机（PB4=SDA、PB5=SCL，7 位从机地址默认 `0x50`，见 main.c 的 `I2C_SLAVE_ADDR`）
- 面向对象封装：I2CS / SEGLED / UART 均为结构体 + 函数指针绑定方法（参照 project2）
- 启动演示：上电后**从右到左逐位填充 `8.` 的四步动画（低位→高位）**，每步点亮区域向左扩张一位。
  位编号约定：**位 4 最右 = 低位（`digits[3]`），位 1 最左 = 高位（`digits[0]`）**；
  第 1 步仅位 4（最右）显示 `8.`（其余三位全灭）→ 第 2 步位 3~4 → 第 3 步位 2~4 → 第 4 步四位全部 `8.`，
  即 `   8. →  8.8. → 8.8.8. → 8.8.8.8.`；点亮位段码 `0xFF`（`0x7F` 的 `8` + `0x80` 的 dp），
  未点亮位段码 `0x00`（全灭）；**第 1~3 步每步停留 `DEMO_STEP_MS`（main.c 默认 `500` ms），
  第 4 步（`8.8.8.8.`）单独保持 `DEMO_LAST_HOLD_MS`（默认 `2000` ms，即 2 秒）**，两个宏均在
  main.c 配置区可调；第 4 步保持时间到后**不停止**，立即转入小数点跑马灯；演示为一次性播放，不循环、不重播，
  计时基于 TIM1 1ms 中断的非阻塞计数，ISR 内无延时；
  演示序列不再引用 segled 的 `SEGCODE_Count()` / `SEGCODE_CharAt()`（这两个接口保留供其他用途）
- 小数点跑马灯（启动演示结束后自动进入，无限循环）：四位中**始终只有一位点亮 dp 段（段码 `0x80`），
  其余三位全灭（段码 `0x00`）**，点亮位置从低位向高位移位（右→左）`位4→位3→位2→位1→位4`，
  即从演示结束时的最右位 4 起步，依次向左移动，到位 1 后回到位 4；
  节拍由宏 `MARQUEE_STEP_MS` 控制（main.c 默认 `300` ms）；同样由 TIM1 1ms 中断驱动，
  非阻塞、ISR 内无延时；跑马灯期间 I2C 读回的 4 字节同步为当前位 `.`、其余三位空格 `' '`
  （读回内容与屏幕一致）；主机任意一次完整写入会立即取消跑马灯（与取消启动演示同一条路径），
  此后显示完全由主机数据控制，不再自动恢复
- I2C 演示协议：
  - 主机写 4 字节（ASCII 字符）→ 数码管逐位显示：每个字节经 segled 模块的
    `SEGCODE_FromChar()` 字符→段码映射表（const 表存放于 Flash）解析，可识别字符包括
    `'0'-'9'`、`'A'`、`'b'`、`'C'`、`'d'`、`'E'`、`'F'`、`'G'`、`'H'`、`'I'`、`'J'`、
    `'L'`、`'n'`、`'O'`、`'P'`、`'q'`、`'r'`、`'S'`、`'t'`、`'U'`、`'y'`、`'-'`、`'.'`
    （`'.'` 仅点亮小数点段 dp，其余段熄灭），**不在表中的任意字符显示空格（段码 0x00，全灭）**；
    UART 打印 `I2C WR: 0x.. ...`（接收字节的十六进制）；**任意主机写入会立即取消启动演示
    或小数点跑马灯**，此后显示完全由主机数据控制，不再自动恢复演示/跑马灯
  - 主机读 4 字节 → 回传当前显示的 4 个字符（启动演示期间为点亮位 `'8'`、未点亮位空格 `' '`，
    按位 1→位 4 顺序回传，故第 1 步回传 `"   8"`、第 2 步 `"  88"`、第 4 步回传 `"8888"`；
    小数点跑马灯期间为当前点亮位 `.` 与其余三位空格 `' '`；主机写入后为最近写入的 4 个原始字符）
- 数码管动态扫描：TIM1 1ms 中断逐位点亮，4ms 完成一轮刷新（约 250Hz/位，无闪烁）
- UART 调试：115200 8N1，上电打印欢迎信息、I2C 写事件打印、串口输入回显
- 中断向量归位 main 单元：TIM1(IRQ11)、UART1_RX(IRQ18)、I2C(IRQ19) 均定义在 main.c，
  子模块只暴露普通处理函数/ISR 委托函数（规避 SDCC 向量表只从 main 单元生成的问题）

## 编译烧录（cmd，避免 PowerShell 重定向编码问题）

```bat
cd /d D:\stm8s_project\project3
build.bat
rem 或 make -f Makefile

rem 烧录
D:\sdcc\bin\stm8flash.exe -c stlinkv2 -p stm8s103f3 -w project3.ihx
```

## I2C 从机测试（主机侧示例）

```c
/* 主机写 4 字节 ASCII：显示 "1.2A"（'.' 只亮 dp 段） */
i2c_start(); i2c_write(0xA0); i2c_write('1'); i2c_write('.'); i2c_write('2'); i2c_write('A'); i2c_stop();

/* 主机写 4 字节 ASCII：其中 'X' 不在可识别字符表内，该位显示空格（全灭） */
i2c_start(); i2c_write(0xA0); i2c_write('9'); i2c_write('X'); i2c_write('3'); i2c_write('.'); i2c_stop();

/* 主机读 4 字节：读回最近写入的 4 个原始字符 */
i2c_start(); i2c_write(0xA1); v[0]=i2c_read(ACK); ... v[3]=i2c_read(NACK); i2c_stop();
```

## 文件清单

| 文件 | 说明 |
|---|---|
| main.c | 应用编排 + 中断向量表（TIM1 / UART1_RX / I2C） |
| i2c_slave.h / i2c_slave.c | I2C 从机对象：7 位地址、事件/错误中断状态机、写回调/读请求回调 |
| segled.h / segled.c | 四位共阴 8 段数码管对象：12 GPIO 动态扫描、Flash 字符→段码映射表与 `SEGCODE_FromChar()` / `SEGCODE_Count()` / `SEGCODE_CharAt()` 接口（映射表是字符与段码的唯一数据源，供 I2C 写入映射使用；启动演示已改为逐位填充动画，不再引用计数/取字符接口） |
| uart.h / uart.c | UART 对象（复用 project2 实现）：115200 调试串口 |
| build.bat | Windows 编译脚本 |
| Makefile | Makefile 构建脚本 |
| .vscode/ | VS Code 任务（编译/烧录）与 IntelliSense 配置 |
*（内容由AI生成，仅供参考）*
*（内容由AI生成，仅供参考）*
*（内容由AI生成，仅供参考）*
*（内容由AI生成，仅供参考）*
