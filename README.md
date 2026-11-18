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

# project3 - STM8S103F3P6 I2C 从机 + 四位数码管（软件模拟 I2C 从机）

STM8S103F3P6（TSSOP20）+ SDCC 的 I2C 从机工程，接收主机（`project4`）转发来的字符流，
在四位数码管上滚动显示。工程风格参照 `project2`：结构体 + 函数指针的面向对象封装、
不使用官方 SPL 库、`main.c` 集中定义中断向量。

**从机 I2C 已由片内硬件外设改为 GPIO 软件模拟（轮询式 bit-bang）**，主机侧也同步改为软件
模拟并降速，两端在低速总线上完成稳定通信。

## 1. 硬件接线（TSSOP20）

| 功能 | 引脚 | 说明 |
|---|---|---|
| I2C_SCL | PB4（第 12 脚） | 软件模拟 I2C 时钟，开漏 + 外部 4.7k 上拉 |
| I2C_SDA | PB5（第 11 脚） | 软件模拟 I2C 数据，开漏 + 外部 4.7k 上拉 |
| 段选 a | PA1 | 共阴数码管 a 段 |
| 段选 b | PA2 | 共阴数码管 b 段 |
| 段选 c | PA3 | 共阴数码管 c 段 |
| 段选 d | PC4 | 共阴数码管 d 段 |
| 段选 e | PC3 | 共阴数码管 e 段 |
| 段选 f | PC5 | 共阴数码管 f 段 |
| 段选 g | PC7 | 共阴数码管 g 段 |
| 段选 dp | PC6 | 小数点段 |
| 位选 1 | PD3 | 第 1 位（最左 / 高位）公共阴极，低电平选中 |
| 位选 2 | PD2 | 第 2 位 |
| 位选 3 | PD1 | 第 3 位（**与 SWIM 调试脚共用**，占用后无法在线 SWIM 调试） |
| 位选 4 | PD4 | 第 4 位（最右 / 低位） |
| UART1_TX | PD5 | 115200 8N1 调试输出 |
| UART1_RX | PD6 | 调试输入 |

> IO 资源：四位共阴 8 段数码管需要 8 段选 + 4 位选 = 12 个 GPIO。TSSOP20 实际引出的
> GPIO 只有 PA1~PA3、PB4/PB5、PC3~PC7、PD1~PD6，本设计**完全避开 I2C（PB4/PB5）与
> UART（PD5/PD6）**，因此调试串口得以保留；代价是 PD1（SWIM）被位选占用。
> 大电流/大尺寸数码管建议位选加 NPN 三极管驱动，小尺寸可直接 GPIO 驱动。

## 2. 功能

- **软件模拟 I2C 从机**：PB4=SCL、PB5=SDA 开漏 GPIO，7 位地址 `0x50`，
  由 `i2c.poll()` 在 `while(1)` 里高频采样 SCL/SDA 边沿并推进位状态机
- **主循环禁止 `wfi`/`halt`**：从机靠轮询采样，一旦休眠就会错过 START 与所有 SCL 边沿；
  UART 服务被节流为"每 16 次 poll 排空一次 TX 缓冲"，以保住采样率
- **字符流显示**：主机每转发一个字符组一帧 `{0x10, pos, ch, 0x00}`，从机自行维护排位，
  实现"从右往左滚动 + 小数点叠加 + 回车清屏"
- **启动动画**：上电从右到左逐位填充 `8.`（`   8. →  8.8. → 8.8.8. → 8.8.8.8.`），
  第 4 步保持 `DEMO_LAST_HOLD_MS`（默认 2000ms）后转入小数点跑马灯
- **小数点跑马灯**：只有一位点亮 dp（段码 `0x80`），其余全灭，位 4 → 位 1 循环，
  节拍 `MARQUEE_STEP_MS`（默认 300ms）
- **主机接管即接管到底**：任何一次合法的字符写入都会立刻停掉动画，此后显示完全由主机数据
  控制，不再自动恢复
- **数码管动态扫描**：TIM1 1ms 中断逐位点亮，4ms 一轮（约 250Hz/位，无闪烁）
- **UART 调试**：115200 8N1，`SLAVE ready` 横幅、每个显示字符回显、引脚自检报告

## 3. I2C 从机位状态机（软件模拟）

`i2c_slave.c` 内部用一个相位变量推进，所有采样都在 `poll()` 里完成，**不阻塞、不用中断**：

| 相位 | 名称 | 动作 |
|---|---|---|
| 0 | `BB_PH_IDLE` | 等 SCL 高期间 SDA 下降沿 = START |
| 1 | `BB_PH_ADDR` | 逐位移入 8 位地址，与 `addr7` 比对 |
| 2/3 | `BB_PH_ADDR_ACK` / `ACK2` | 匹配则拉低 SDA 应答，否则放弃 |
| 4 | `BB_PH_RX` | 逐位移入数据字节入 `rx_buf` |
| 5/6 | `BB_PH_RX_ACK` / `ACK2` | 给出 ACK，准备下一字节 |
| 7 | `BB_PH_TX` | 主机读时逐位移出 `tx_buf` |
| 8 | `BB_PH_TX_ACK` | 读主机的 ACK/NACK，NACK 后收尾 |
| 9 | `BB_PH_WAIT_STOP` | 等 SCL 高期间 SDA 上升沿 = STOP，然后回调 `onWrite()` |

- 引脚方向切换：只有需要拉低 SDA 时才 `DDR|=SDA`，其余时间保持输入让外部上拉；
- `I2C_SCL_PIN = 0x10`（PB4）、`I2C_SDA_PIN = 0x20`（PB5）；
- **`GPIO_PB_CR2` 中的对应位必须保持为 0**：PB 口 EXTI 一旦使能，总线边沿会触发 Port B
  中断并跳入空向量 `0x000000`，导致整机 HOLD。

## 4. 协议

主机写入帧（每帧 4 字节，从机只看前 3 字节）：

| CMD | 参数 | 从机行为 |
|---|---|---|
| `0x10` | `{0x10, pos, ch, 0x00}` | `ch` 按**原始 ASCII** 处理；`pos` 被忽略，排位由从机自己维护 |
| `0x01` | `{0x01, 0x5A, 0x00, 0x00}` | PING，把 `tx_buf` 置为 `"PING"`，**不触碰显示** |
| 其它 | — | 忽略，显示保持不变 |

`ch` 的处理规则（`disp_push_char()`）：

| 输入字符 | 行为 |
|---|---|
| 复位后第一个字符 | `disp_take_over()`：停演示/跑马灯、清屏，字符显示在最右位 digit4 |
| `.` | 只把 dp 段叠加到最右位，不移动内容 |
| 可显示字符（命中 `SEGCODE_FromChar()` 索引表） | 整体右移一位，新字符进最右位 |
| `\r` | 清屏（4 位全灭），光标回到最右位 |
| 其它（空格、`\n` 等，段码 0x00） | 忽略，显示不动 |

主机读（`0xA1`）：回传当前 4 位显示的字符镜像 `tx_buf`，与屏幕内容一致
（演示期间为 `'8'` / 空格，跑马灯期间为 `'.'` / 空格，主机接管后为最近写入的字符）。

可显示字符集：`0-9`、`A b C d E F G H I J L n O P q r S t U y`、`-`、`.`；
表外字符一律显示空格（段码 `0x00` 全灭）。

## 5. 显示逻辑

```c
static void disp_push_char(uint8_t ch)
{
    if (!g_disp_host) disp_take_over();      /* 首次接管：停动画 + 清屏 */
    if (ch == '\r') { disp_clear(); return; } /* 回车清屏 */
    if (ch == '.')  { seg.digits[DISP_LAST] |= SEG_DP_BIT; return; } /* 叠小数点 */

    code = ch_to_disp(ch);
    if (code == 0x00u) return;               /* 表外字符忽略 */

    for (i = 0; i + 1u < I2C_DISP_LEN; i++) { /* 整体右移一位 */
        seg.digits[i] = seg.digits[i + 1u];
        i2c.tx_buf[i] = i2c.tx_buf[i + 1u];
    }
    seg.digits[DISP_LAST] = code;            /* 新字符进最右位 */
    i2c.tx_buf[DISP_LAST] = ch;
}
```

函数内只写扫描缓冲，不阻塞、不打印长串，可安全地在 I2C 采样上下文里调用。

## 6. 主循环

```c
while (1) {
    i2c.poll(&i2c);            /* 采样 SCL/SDA，推进位状态机 —— 必须连续调用 */

    if (++svc >= 16u) {        /* 每 16 次 poll 服务一次 UART */
        svc = 0;
        uart.tx_service(&uart);
    }
}
```

> **不要在这里加 `wfi`**：从机无法用 SCL 边沿中断唤醒（PB 口 EXTI 只有单一触发边沿配置），
> 只能靠高频轮询采样；休眠会直接睡过 START 条件，I2C 完全失效。

中断使用情况：`TIM1_UPD (IRQ10 → __interrupt(11))` 负责 1ms 动态扫描与动画计时；
硬件 I2C 中断（IRQ19）随硬件从机方案一并弃用；`main.c` 仍保留 UART1_RX 向量。

## 7. 编译烧录

```bat
cd /d D:\stm8s_project\project3
build.bat
rem 或 make

D:\sdcc\bin\stm8flash.exe -c stlinkv2 -p stm8s103f3 -w project3.ihx
```

> 烧录日志重定向建议走 `cmd`：`cmd /c "... > flash.log 2>&1"`，
> PowerShell 直接 `>` 偶发 `PermissionError 13`（文件被串口终端占用/编码问题）。

## 8. UART 输出示例

```
SLAVE ready
#SELFTEST ...
8
8
1
2
.
```

每个 `0x10` 帧里的字符都会原样回显一次（`\r` 打印为换行），便于与主机日志逐字对齐。

## 9. 文件清单

| 文件 | 说明 |
|---|---|
| main.c | 应用编排、中断向量（TIM1 / UART1_RX）、演示与跑马灯、`i2c_on_write` 协议解析、`disp_*` 显示逻辑、引脚自检 |
| i2c_slave.h / i2c_slave.c | **软件模拟 I2C 从机**：GPIO 位状态机、`poll()` 采样、`rx_buf`/`tx_buf`、`onWrite`/`onRequest` 回调 |
| i2c_slave.hw_backup.c.txt | 历史备份：早期硬件 I2C 从机实现（寄存器 + 中断状态机），不参与编译 |
| segled.h / segled.c | 四位共阴 8 段数码管：12 GPIO 动态扫描 + Flash 字符→段码映射表（`SEGCODE_FromChar()`） |
| uart.h / uart.c | UART1 调试输出（TX 环形缓冲 + 非阻塞 printf） |
| build.bat / Makefile / flash.bat | 构建与烧录脚本 |
| .vscode/ | VS Code 任务与 IntelliSense 配置 |
| README.md | 本文档 |

## 10. 踩坑记录

| 现象 | 根因 | 处理 |
|---|---|---|
| 从机黑屏，动画停在某一位，整机 HOLD | PB4/PB5 的 `CR2` 被置 1 → 总线边沿触发 PB EXTI → 跳空向量 `0x000000` | `CR2` 对应位强制清零；主机侧同步加空向量兜底 |
| 黑屏但 MCU 未跑飞 | `onWrite` 把裸字节直接喂给字符段码表，非字符映射到 `0x00` → 四位全灭 | 改为字符流语义，表外字符忽略不动显示 |
| 从机日志停打、I2C 采样被饿死 | 主循环里整行 `uart.printf/puts` 阻塞发送 | `tx_service` 改非阻塞，每 16 次 poll 才服务一次 |
| 通信完全无响应 | 主循环残留 `wfi`，睡过 START 与所有 SCL 边沿 | 去掉 `wfi`，改纯轮询 |
| `printf` 多参数输出后程序异常 | `onWrite` 内 `W:%x,%x,%x,%x` 变参调用破坏返回栈 | 去掉多参数调试打印，改单字符 `putc` 回显 |
| 优化器报 `warning 110` | 条件流被优化器重排，非 bug | `#pragma disable_warning 110` 局部屏蔽 |
| 主机发 `8888` 从机只显示 2 位 | 主机侧单变量接收覆盖（非从机问题） | 主机 UART 改 64 字节 RX FIFO（见 project4） |

*（内容由AI生成，仅供参考）*
