# 固件代码说明（IMU_Car.ino 逐段讲解）

> 对应文件：[code/arduino/IMU_Car.ino](../code/arduino/IMU_Car.ino) ｜ 约 790 行（含注释）｜ 单文件
> 目标读者：大二物联网工程，会 C 语言基础、刚接触 Arduino。
> 阅读顺序建议：先看 §2 的整体结构，再按 §4 的函数顺序读，最后看 §6 的算法详解。

---

## 1. 运行环境与依赖

| 项目 | 内容 |
| --- | --- |
| 开发板 | ESP32 DevKit V1（30 针） |
| 核心版本 | arduino-esp32 **3.x**（本文在 3.3.11 上编译验证） |
| 外部库 | **RF24** by TMRh20（Arduino 库管理器安装，仅此一个） |
| IMU 库 | **不需要**。直接操作 ICM20689 寄存器（见 §5.2 说明原因） |
| 编译状态 | 已用 `-Wall -Wextra` 编译通过，**零错误零告警** |

### 1.1 关于"文件夹名必须等于文件名"

Arduino IDE 的硬性要求：**`IMU_Car.ino` 必须放在名为 `IMU_Car` 的文件夹里**。

本仓库为了目录整洁把文件放在 `code/arduino/IMU_Car.ino`，所以：

- **方法一（推荐）**：新建 `IMU_Car` 文件夹，把 `.ino` 复制进去，用 IDE 打开。
- **方法二**：直接双击打开，IDE 会提示"文件需要在同名文件夹中"，点确定让它自动创建。
- **方法三**：用 `arduino-cli` / VS Code + Arduino 插件，可以绕过 IDE 的限制（但仍建议遵守约定）。

---

## 2. 整体结构

单文件，但用注释分成了 **12 个章节**，顺序是"从下往上"堆依赖的（后面的代码只依赖前面定义的）：

| 章节 | 内容 | 行数占比 | 说明 |
| --- | --- | --- | --- |
| §1 | 引脚定义 | 小 | 全部 `PIN_*` 宏 |
| §2 | 可调参数 | 中 | **调参只需要改这一块** |
| §3 | 数据类型 | 小 | `CtrlPacket`、`RunMode` |
| §4 | 全局状态 | 小 | 所有 `s_` 前缀的静态变量 |
| §5 | 通用工具 | 小 | `crc8`、`mapAxis`、`tickLed` |
| §6 | ICM20689 驱动 | 大 | I2C 读写 + 寄存器初始化 |
| §7 | Mahony AHRS | 中 | 姿态解算核心 |
| §8 | nRF24L01 | 小 | 初始化 + 发送 |
| §9 | 标定 | 中 | 零偏 + 水平基准 |
| §10 | 按键 | 小 | 模式切换 |
| §11 | `setup()` | 大 | = 步骤 1 |
| §12 | `loop()` | 大 | = 步骤 2 |

### 2.1 为什么把类型定义放最前面

Arduino IDE 会在编译前**自动生成函数原型**，插在文件靠前的位置。如果 `CtrlPacket` 定义在后面，原型里的 `const CtrlPacket &pkt` 就会报"未定义类型"。

**这是 Arduino 新手最容易被坑的编译错误**，解决方式就是把所有 `struct`/`enum` 定义放在文件顶部（`#include` 之后）。

### 2.2 命名约定

| 前缀 | 含义 | 例子 |
| --- | --- | --- |
| `PIN_` | 引脚号 | `PIN_NRF_CE` |
| `IMU_REG_` | 寄存器地址 | `IMU_REG_PWR_MGMT_1` |
| `FLAG_` | 包里的标志位 | `FLAG_ARMED` |
| `s_` | 文件内静态全局变量（static global） | `s_roll` |
| `MODE_` | 运行模式枚举 | `MODE_DRIVE` |

---

## 3. 核心数据结构：CtrlPacket

```cpp
#pragma pack(push, 1)
typedef struct {
  uint8_t  head;      // [0]
  uint8_t  seq;       // [1]
  uint8_t  flags;     // [2]
  int8_t   vx;        // [3]
  int8_t   vy;        // [4]
  int8_t   tilt;      // [5]
  int16_t  panDelta;  // [6-7]
  int16_t  roll10;    // [8-9]
  int16_t  pitch10;   // [10-11]
  uint8_t  dtMs;      // [12]
  uint8_t  crc;       // [13]
} CtrlPacket;
#pragma pack(pop)
```

### 3.1 `#pragma pack(push,1)` 到底在干什么

**背景**：C 编译器为了让 CPU 访问更快，会把每个字段对齐到它的自然边界，必要时插入"填充字节"。

**但对本结构体，实测加不加 pack 都是 14 字节。** 我们用 xtensa-esp32-elf-g++ 实测过两种写法的 `sizeof`，结果都是 14：

```
有 #pragma pack(1)  →  sizeof = 14
没有 #pragma pack   →  sizeof = 14      ← 本例恰好天然对齐
```

原因看布局就明白：前 6 个字段都是 1 字节（占满偏移 0~5），接下来第一个 `int16` 落在偏移 **6**，而 6 是偶数——**天然满足 2 字节对齐，不需要插入填充**。后面两个 `int16` 依次落在 8、10，也都对齐。最后两个 `uint8` 占 12、13，总长 14，而结构体对齐要求是 2，14 已经是 2 的倍数，末尾也不用补。

**那为什么还要保留 `#pragma pack(1)`？** 三个理由，都不是"现在就必须"：

| 理由 | 说明 |
| --- | --- |
| ① 抗未来改动 | 一旦调整字段顺序（比如把 `dtMs` 挪到 `tilt` 后面）或新增字段，立刻就会冒出填充字节，而 `sizeof` 悄悄从 14 变成 16——不报错，只是两端对不上 |
| ② 抗编译器差异 | 不同编译器/目标（尤其 ARM 上 8 字节对齐规则、`int32`/指针宽度不同的平台）的默认对齐规则并不完全一致 |
| ③ 明确表达意图 | 看到 `pack(1)` 的读者立刻知道"这是要被逐字节解析的线上格式"，不需要去算对齐 |

**真正把契约锁死的是这一行**（紧跟 `pack(pop)` 之后）：

```cpp
static_assert(sizeof(CtrlPacket) == 14, "CtrlPacket must be exactly 14 bytes");
```

这是**编译期断言**：大小一旦不等于 14，编译直接失败，不会等到联调时才发现"数据都是乱的"。我们验证过它确实生效——故意改成 `== 99` 后编译报 `static assertion failed: CtrlPacket must be exactly 14 bytes`。

> **给大二同学的经验**：协议结构体应当**同时**保留 `#pragma pack(1)` 和 `static_assert`。前者表达意图并提供冗余保护，后者保证"改坏了立刻知道"。只靠其中任何一个，都不如两个一起用。

### 3.2 内存布局图（小端）

```
偏移  0    1    2    3    4    5    6  7    8  9   10 11   12   13
     ┌────┬────┬────┬────┬────┬────┬──────┬──────┬──────┬────┬────┐
     │ A5 │seq │flg │ vx │ vy │tl  │panDl │roll10│pitch │dtMs│crc │
     └────┴────┴────┴────┴────┴────┴──────┴──────┴──────┴────┴────┘
      uint8 × 6          int16 小端（低字节在前）× 3
```

**为什么 `panDelta` 用 int16 而不是 int8**：云台增量单位是 0.1°，单帧最大 30° = 300，超出 int8 的 ±127 范围。

**为什么不用 float 传 roll/pitch**：float 是 4 字节，两个就 8 字节；用 `×10` 的 int16 只需 4 字节，且避免了两端浮点精度/字节序差异。**嵌入式通信协议里"定点数化"是标准做法**。

### 3.3 车端怎么解

Python 侧（示意）：

```python
import struct
head, seq, flags, vx, vy, tilt, pan, roll10, pitch10, dt, crc = \
    struct.unpack('<BBBBbbBhhhBB', buf)
```

`<` 表示小端。字段顺序必须与上表**逐字节对应**。

---

## 4. 函数逐个讲

### 4.1 工具函数（§5）

#### `crc8()`

```cpp
static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}
```

逐位计算的 CRC-8，多项式 `0x07`（CRC-8/ATM 标准）。虽然没有查表法快，但 14 字节的数据量下耗时 < 5µs，完全够用，且代码好懂。

> **为什么要加 CRC**：nRF24L01 自带 16 位硬件 CRC，理论上够了。但应用层再加一层 CRC-8 有实际价值——它能抓出"硬件 CRC 通过但结构体对不齐"这类软件层错误，联调时能直接定位是哪一端的问题。

#### `mapAxis()` —— 手势映射核心

```cpp
static int8_t mapAxis(float deg, float dead, float full, int8_t invert) {
  float magnitude = fabsf(deg) - dead;
  if (magnitude <= 0.0f) return 0;                    // ① 死区

  float s = magnitude / (full - dead);                // ② 归一化
  if (s > 1.0f) s = 1.0f;

  int8_t out = (int8_t)(powf(s, MAP_GAMMA) * OUT_MAX + 0.5f);  // ③ 非线性
  out = (int8_t)(out * invert);                       // ④ 方向
  return (deg > 0.0f) ? out : (int8_t)(-out);
}
```

四个步骤逐行看：

| 步 | 代码 | 作用 |
| --- | --- | --- |
| ① | `magnitude = |deg| - dead` | 把 [-dead, +dead] 压缩成 0，消除手抖 |
| ② | `s = magnitude/(full-dead)`，上限 1 | 把 [dead, full] 线性映射到 [0,1] |
| ③ | `powf(s, 1.6) * 100 + 0.5` | 非线性压缩小角度；`+0.5` 是四舍五入 |
| ④ | `* invert` 后补符号 | 方向修正后把符号加回去 |

**为什么不用 `if/else` 分段线性**：`s^gamma` 只用一个参数就能连续调节"软硬程度"，比分段线性少 4~5 个参数，调起来更快。这是游戏手柄摇杆曲线的标准做法。

**为什么 `+ 0.5f` 能四舍五入**：`(int)` 强制转换是**截断**而非舍入，`2.9 → 2`。加 0.5 后 `3.4 → 3.9 → 3`，`2.6 → 3.1 → 3`，实现了四舍五入。

> 这个函数的参数全在 §2.4 的宏里，**改手感不用动函数本身**。

#### `tickLed()`

```cpp
if (s_calibrating)   digitalWrite(HIGH);              // 常亮
if (!s_radioOk)      digitalWrite((now/100)%2);        // 10Hz 快闪
else                 digitalWrite(now%1000 < 50);      // 1Hz 心跳
```

三种状态用同一个 LED 表示，**不看串口也能知道板子状态**。这个在装进手套后就特别有用了（不方便插串口线时）。

### 4.2 ICM20689 驱动（§6）

> 代码里所有函数/宏都用 `imu*` / `IMU_REG_*` 这种**与器件无关**的命名，就是为了以后再换传感器时不用改名。

#### `imuWrite` / `imuRead`

```cpp
static bool imuRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // false = 不发 STOP
  if ((int)Wire.requestFrom((uint8_t)IMU_I2C_ADDR, len) != (int)len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}
```

**关键点：`endTransmission(false)`**

这个 `false` 表示"发完寄存器地址后**不产生 STOP 信号**"，保持总线占用，紧接着发 ReStart 去读数据。这是 I2C 的标准"先写地址再读数据"时序。

如果写成 `endTransmission()`（默认 true），中间会插一个 STOP，部分芯片会复位内部地址指针，导致读出错误数据。

**返回值检查**是必须的：读失败时 `requestFrom` 返回实际读到的字节数，不等于 `len` 就说明总线有问题。**不检查返回值是新手最常见的错误**——数据错了但程序还在跑，最后怪传感器。

#### `imuInit()` —— ICM20689 的初始化顺序

```cpp
imuRead(IMU_REG_WHO_AM_I, &id, 1);          // ① 先验身份，必须是 0x98
imuWrite(IMU_REG_PWR_MGMT_1, 0x80);         // ② 软复位
delay(100);                                  //    必须等！复位需要时间
imuWrite(IMU_REG_PWR_MGMT_1, 0x01);         //    唤醒 + 时钟源 = 陀螺 X 轴 PLL
imuWrite(IMU_REG_PWR_MGMT_2, 0x00);         // ③ 六轴全部使能
imuWrite(IMU_REG_SMPLRT_DIV, 0x04);         //    200Hz
imuWrite(IMU_REG_CONFIG, 0x03);             // ④ 陀螺 DLPF = 41Hz
imuWrite(IMU_REG_ACCEL_CONFIG2, 0x03);      //    加速度 DLPF = 44.8Hz ← 别忘！
imuWrite(IMU_REG_GYRO_CONFIG,  0x08);       // ⑤ ±500 °/s
imuWrite(IMU_REG_ACCEL_CONFIG, 0x08);       //    ±4 g
```

**为什么第一步就验 `WHO_AM_I`**：ICM20689 固定返回 `0x98`。如果读回来是 `0x68`，说明手上是 MPU6050；是 `0x00`，说明 I2C 根本没通（常见原因：**CS 没拉高**）。**先确认器件型号，再谈精度**。代码把这个值存进 `s_imuWhoAmI`，失败时打印出来，省去大量猜测。

**为什么写两次 `PWR_MGMT_1`**：第一次 `0x80` 是复位（bit7=DEVICE_RESET），复位后所有寄存器回默认值。第二次 `0x01` 是"唤醒 + 选时钟源"。**少写第二次，芯片会一直休眠，读出来全是 0**。

**为什么时钟源要选 PLL（0x01）**：默认是内部 RC 振荡器，随温度漂移大，会让陀螺零偏跟着漂。用 X 轴陀螺的 PLL 作为时钟源，稳定性好得多。

**为什么 `PWR_MGMT_2` 要显式写 0**：这个寄存器是"逐轴使能/待机"开关，值 0 表示六轴全部工作。不能假设复位默认值一定是全使能——显式写一次只花一行代码，省掉"某个轴恒为 0"的诡异问题。

**为什么加速度 DLPF 单独写 `ACCEL_CONFIG2`**：见 [debug_faq §3.5](debug_faq.md)。简单说，**这是 ICM20689 与 MPU6050 最大的寄存器差异**：MPU6050 的加速度和陀螺共用一个 DLPF（在 `CONFIG`），ICM20689 分开了。只写 `CONFIG` 的话加速度还是 218Hz 宽带。

**三重回读校验**：

```cpp
if ((g & 0x18) != 0x08) return false;   // 陀螺量程 FC_SEL[4:3]
if ((a & 0x18) != 0x08) return false;   // 加速度量程 AFS_SEL[4:3]
if ((f & 0x07) != 0x03) return false;   // 加速度 DLPF ← 这一条最容易被忽略
```

写完立刻读回来对比。**这一步能提前抓出 90% 的接线故障**，比等到后面发现"数据抖动异常"再回头查要快得多。

#### `imuReadRaw()`

```cpp
uint8_t b[14];
imuRead(IMU_REG_ACCEL_XOUT_H, b, 14);   // 从 0x3B 一次读 14 字节
r.ax = (int16_t)((b[0] << 8) | b[1]);   // 高字节在前（大端！）
```

**两个要点**：

1. **一次突发读 14 字节**（加速度 6 + 温度 2 + 陀螺 6）。比读 6 次快 6 倍，而且能保证六轴是**同一时刻**的采样——分开读会导致加速度和陀螺差几十毫秒，Mahony 融合时会出错。
2. **ICM20689 的数据寄存器是大端**（高字节在低地址），而 ESP32 是小端。所以要手工拼接：`(高 << 8) | 低`。**别忘了强转 `(int16_t)`**，否则负数会变成大正数。

> **好消息**：ICM20689 与 MPU6050 的数据寄存器布局**完全一致**（都是 `0x3B` 起 14 字节，加速度在前、温度在中间、陀螺在后）。所以换传感器时，这个函数一行都不用改——这也是为什么整个姿态解算部分（§7）完全不受影响。

#### `remapImu()`

把芯片坐标系旋转到手套坐标系，4 个预设。实现上先读入局部变量，最后一次性写回：

```cpp
float tx = ax, ty = ay, tz = az;   // 先拷贝
...
ax = tx; ay = ty; az = tz;         // 最后写回
```

**为什么先拷贝**：如果直接 `ax = -ay; ay = ax;`，第二句用的就是已经被改过的 `ax`，结果全错。这是"就地交换"的经典陷阱。

> 三个预设置的旋转矩阵行列式都是 +1（真旋转），这点很重要——如果行列式是 -1（镜像），Mahony 的重力方向会完全对不上，姿态永远收敛不了。

#### `accelFilter()`

4 点滑动平均，用**累加和**实现，避免每次循环 4 次加法：

```cpp
s_accSum[i] -= s_accBuf[i][s_accIdx];   // 减去要出去的
s_accBuf[i][s_accIdx] = in[i];          // 存入新的
s_accSum[i] += in[i];                   // 加上新的
out[i] = s_accSum[i] / ACC_WIN;
```

第一次调用时把整个缓冲区填满当前值（`if (!s_accReady)`），否则启动瞬间输出会从 0 慢慢爬上来，造成姿态初值错误。

**为什么只滤加速度不滤陀螺**：见 [分层说明](IMU人体姿态控制小车%20分层说明及具体实施步骤.md) §7.2。简单说：加速度是慢信号（重力方向），滤波有益；陀螺是快信号（真实动作），滤波 = 延迟 = 手感变肉。

### 4.3 Mahony（§7）

见 §6.1 的算法详解。这里只说实现细节：

- 四元数用 **4 个独立的 `float` 全局变量**（`s_q0~s_q3`）而不是 `float q[4]`。原因：访问快，且不会被写越界。
- `mahonyReset()` 和 `mahonyInitFromAccel()` 分开两个函数，前者用于回零，后者用于上电定初值。
- `mahonyUpdate()` 里 `gx, gy, gz` 是**传值**（`float`），函数内直接修改不会影响调用者——这点很重要，因为函数内部会加上反馈量。

### 4.4 nRF24L01（§8）

#### `radioInit()`

配置顺序有讲究：

```cpp
SPI.begin(18, 19, 23, 5);      // ① 先启动 SPI 总线
if (!s_radio.begin()) return false;   // ② 再初始化 radio
if (!s_radio.isChipConnected()) return false;  // ③ 检查芯片在位
s_radio.setChannel(...);               // ④ 最后配参数
```

**为什么 `setChannel` 必须在 `begin()` 之后**：`begin()` 会把所有寄存器恢复默认值（信道变回 76 的默认值？不，默认是 76 但速率是 1Mbps）。如果在 `begin()` 前设置参数，会被 `begin()` 覆盖掉。

**`isChipConnected()` 的价值**：它能区分两种失败——"SPI 完全没通"和"通了但芯片型号不对"。前者查线，后者查供电/模块。**没有这个检查，两种情况的报错信息一模一样，会浪费大量时间**。

**关自动应答的三个好处**：

| | 开 ACK | 关 ACK（本项目） |
| --- | --- | --- |
| 单帧耗时 | ~1ms + 重发 | < 1ms |
| 失败阻塞 | 可能阻塞 750ms | 不阻塞 |
| 供电压力 | 重发时电流持续 115mA | 脉冲极短 |

代价是"不保证送达"。但 50Hz 下丢一帧只损失 20ms，**比阻塞 750ms 好得多**。

#### `radioSend()`

```cpp
static bool radioSend(const CtrlPacket &pkt) {
  if (!s_radioOk) return false;
  return s_radio.write(&pkt, (uint8_t)sizeof(pkt));
}
```

用 `const &` 传参避免拷贝（虽然只有 14 字节，但这是好习惯）。`s_radioOk` 检查让 BENCH 模式下也能安全调用。

### 4.5 标定（§9）

#### `calibrateGyro()`

```cpp
for (uint16_t i = 0; i < samples; i++) {
  if (imuReadRaw(r)) { sum[0] += r.gx; ... got++; }
  delay(2);
}
s_gyroBias[0] = (sum[0] / got) / GYRO_LSB_PER_DPS;
```

- 1000 点 × 2ms = 2 秒。
- **除以 `got` 而不是 `samples`**：如果中间有几帧读失败，用实际成功的数量算平均才准确。
- 结果单位是 **°/s**（不是 LSB），这样后面减的时候直接减，不用每次换算。

#### `captureLevel()`

采 500ms 的姿态均值作为零点。用 `while (millis() - t0 < ...)` 而不是 `for` 循环固定次数，因为每帧的解算耗时可能略有不同。

#### 一个容易被忽略的细节

`setup()` 里调用 `captureLevel()` 之前必须先 `mahonyInitFromAccel()`，否则四元数还是初始的 `q0=1`，姿态要向真实值收敛，500ms 内采到的均值是错的（偏向 0）。**顺序反了会导致水平基准永远偏几度，而且很难查**。

### 4.6 按键（§10）

```cpp
if (pressed && !s_btnLast) { ... }              // 下降沿（刚按下）
else if (pressed && !s_btnLong && (now - s_btnDownMs > 1500)) { ... }  // 长按
else if (!pressed && s_btnLast) { ... }         // 上升沿（刚松开）
```

**状态机三态**：松开 → 按下（记录时刻）→ 长按判定 → 松开（判断是短按还是长按）。

`(now - s_btnDownMs > 30)` 的 30ms 是**消抖**：机械按键按下瞬间会弹跳多次，不加消抖会被识别成"按了好几次"。

长按结束后 `s_lastUs = micros();` 这一句很关键——`captureLevel()` 阻塞了 500ms，如果不重置时间戳，下一帧的 `dt` 会是 0.5 秒，四元数积分会出错。

### 4.7 `setup()`（§11）

流程严格对应文档步骤 1：

| 代码 | 对应文档 |
| --- | --- |
| `Serial.begin` | §6.1 |
| `Wire.begin` + `imuInit` | §6.2 |
| `calibrateGyro` | §6.3 |
| `mahonyInitFromAccel` + `captureLevel` | §6.5 / §6.6 |
| `radioInit` | §6.4 |
| 变量清零 + 协议自检 | §6.7 |

**设计原则：任何一步失败都不 `while(1)` 死等。** 原因：

- IMU 失败 → 继续尝试初始化 nRF，这样能同时看到两个模块的状态，一次排查完。
- nRF 失败 → 进入 BENCH 模式，姿态功能仍可用（能完成 G4~G7 的验证）。

**这是嵌入式调试的重要思路：让失败的部分可见，而不是让整个系统停摆。**

### 4.8 `loop()`（§12）

这是全文最需要理解结构的函数。骨架：

```cpp
void loop() {
  // ⓪ 调度器：没到时间就提前返回
  if ((int32_t)(nowMs - s_nextMs) < 0) { tickLed(); pollButton(); return; }
  s_nextMs += FRAME_MS;
  if ((int32_t)(nowMs - s_nextMs) > 200) s_nextMs = nowMs + FRAME_MS;

  // ① 计算 dt（步骤 2-1）
  // ② 读传感器 + 滤波（步骤 2-2）
  // ③ Mahony 解算（步骤 2-3）
  // ④ 映射（步骤 2-4）
  // ⑤ 组包（步骤 2-5）
  // ⑥ 发送（步骤 2-6）
  // ⑦ 打印（步骤 2-7）
  // ⑧ tickLed()
}
```

#### `(int32_t)(nowMs - s_nextMs) < 0` 为什么这样写

`millis()` 是 `uint32_t`，**49.7 天会溢出回绕**。直接 `if (nowMs < s_nextMs)` 在回绕时会判断错误（0 < 4294967295 成立，导致死等 49 天）。

正确写法是**先做减法再转有符号比较**。两个 `uint32_t` 相减，回绕后差值仍然正确（模运算性质），转成 `int32_t` 后正负号就对。

**这是所有基于 `millis()` 的定时器都必须注意的坑**，考试和面试都常考。

#### 为什么 `s_nextMs += FRAME_MS` 而不是 `s_nextMs = nowMs + FRAME_MS`

`+=` 保证了长期平均周期严格等于 20ms（每次累加固定值），不会因为单帧耗时而累积误差。

#### `dt` 限幅

```cpp
if (dt < 0.001f) dt = 0.001f;
if (dt > 0.100f) dt = 0.100f;
```

下限 1ms 防止除以零效应；上限 100ms 防止卡顿导致四元数积分发散。

#### 为什么 `sampleOk` 要单独一个变量

`imuReadRaw()` 失败时不更新 `rollRel/pitchRel`，它们保持 0。但**不能因此就给车发 `vx=0` 之外的指令**——所以我们用 `sampleOk` 控制：失败时置 `FLAG_CALIB` 位，让车端知道"这帧无效，保持停车"。

#### 发送失败的处理

```cpp
bool txOk = radioSend(pkt);
if (!txOk && s_radioOk) pkt.flags |= FLAG_TXFAIL;
```

注意：这里修改 `pkt.flags` 是在**发送之后**，所以这个标志位实际上会在**下一帧**才被对方看到（因为本帧已经发出去了）。这是有意为之——它是一个"上一帧发送失败"的历史标志。

---

## 5. 几个设计决策的解释（答辩常被问）

### 5.1 为什么用单文件而不是拆成多个 .h/.cpp

大二阶段，**单文件的可读性 > 模块化的可维护性**。学生调试时最怕"这个函数在哪"，一个文件里 Ctrl+F 就能找到。

真要做成工程化项目，建议拆成：

```
IMU_Car/
├── IMU_Car.ino        // setup/loop
├── config.h           // §2 全部参数
├── protocol.h         // CtrlPacket + CRC
├── imu_driver.h/.cpp  // ICM20689 驱动
├── ahrs.h/.cpp        // Mahony
└── radio_link.h/.cpp  // nRF24
```

### 5.2 为什么不用现成的 ICM20689 库

**注意：本项目组其实已经装了 `ICM20689` 库**（作者 Inhwan Wee，v1.1.3，[github.com/finani/ICM20689](https://github.com/finani/ICM20689)，在 `Documents/Arduino/libraries/` 下）。它质量不错，支持 I2C 和 SPI 两种模式。

**本固件的寄存器配置就是对着这个库的源码 + 数据手册核对过的**，可以交叉验证：

| 配置项 | 本固件写入 | 该库对应常量 | 一致？ |
| --- | --- | --- | --- |
| 陀螺 DLPF 41Hz | `CONFIG = 0x03` | `GYRO_DLPF_41HZ = 0x03` | ✅ |
| 加速度 DLPF 45Hz | `ACCEL_CONFIG2 = 0x03` | `ACCEL_DLPF_45HZ = 0x03` | ✅ |
| 陀螺 ±500dps | `GYRO_CONFIG = 0x08` | `GYRO_FS_SEL_500DPS = 0x08` | ✅ |
| 加速度 ±4g | `ACCEL_CONFIG = 0x08` | `ACCEL_FS_SEL_4G = 0x08` | ✅ |
| `WHO_AM_I` | 期望 `0x98` | `whoAmI() != 152` 报错 | ✅ |
| 数据起始地址 | `0x3B` 读 14 字节 | `ACCEL_OUT = 0x3B` | ✅ |

但最终仍选择直接操作寄存器，原因如下：

| 方案 | 优点 | 缺点 |
| --- | --- | --- |
| `ICM20689` 库（I2C 模式） | 现成的 `readAccGyro()`，几十行就能跑；自带陀螺标定和加速度六面标定 | 抽象层厚，看不到寄存器；**初始化固定写 16G/2000dps**，要改量程得再调 API；`begin()` 只返回错误码，不好定位 |
| Adafruit / I2Cdevlib 系列 | 生态成熟 | 对 ICM20689 支持不完整或版本混乱 |
| **直接寄存器（本项目）** | 完全可控、零依赖、能加三重回读校验、**驱动只有约 110 行** | 需要查数据手册 |

**三个具体原因**：

1. **能加"写完回读校验"**。`imuInit()` 写完量程和 DLPF 后逐项读回比对，写不进去直接初始化失败。用库的话你根本不知道它有没有真的写进去——**这是调试阶段价值最高的一段代码**。
2. **错误信息更可诊断**。库的 `begin()` 返回 `-3`；本固件直接打印 `WHO_AM_I=0x68, expect 0x98`，一眼就知道是"模块拿错了"还是"线没通"。
3. **量程要精确控制**。库默认 16G/2000dps，对本项目浪费 4 倍分辨率；直接写寄存器一次到位。

**什么时候该改用库**：如果手上的模块**只有 SPI 引出**，直接改本固件的 SPI 实现不如直接用 `ICM20689` 库（SPI 模式）省事——那种情况下库是更好的选择，不必教条。

> 另外——**读数据手册的能力是物联网专业的基本功**。ICM20689 的寄存器章节不长，配套 PDF 就在库的 `extras/` 目录里，正好拿来练。能对着手册写出驱动，是这门专业的硬技能。

### 5.3 为什么所有参数都用 `#define` 而不是变量

```cpp
#define VX_DEAD_DEG 4.0f
```

- 编译期常量，不占 RAM，能被优化。
- 集中在 §2 一块，**调参时不用在 790 行里找**。
- 缺点：不能运行时修改。如果要做"串口实时调参"，需要改成变量 + 命令解析（见 §8 扩展点）。

### 5.4 为什么用 `static` 修饰所有函数和全局变量

`static` 在文件作用域下表示"仅本文件可见"。单文件项目里看起来多余，但：

- 防止和 RF24 库内部的同名符号冲突
- 编译器能更好地优化（知道外部不可见）
- 以后拆分成多文件时不用改

**这是好习惯，建议养成。**

---

## 6. 关键算法详解

### 6.1 Mahony 姿态融合（§7 章）

目标：把陀螺的"短期准、长期漂"和加速度计的"长期准、短期吵"融合起来。

#### 步骤 ① 归一化加速度

```cpp
float norm = sqrtf(ax*ax + ay*ay + az*az);
ax *= inv; ay *= inv; az *= inv;
```

静止时加速度矢量 = 重力矢量 = (0, 0, 1g)。归一化后得到**单位重力方向向量**，后面只用方向不用大小（大小会被运动加速度污染）。

#### 步骤 ② 用四元数估计重力方向

四元数对应的旋转矩阵第三列（即"世界上方向"在机体坐标系的表示）：

```
halfvx = q1*q3 - q0*q2
halfvy = q0*q1 + q2*q3
halfvz = q0*q0 - 0.5 + q3*q3
```

注意这里的 `half` 前缀：真实值是这些的 2 倍，但后面叉积也会带系数 2，所以约掉了。

**验证方法**（想确认公式对不对）：代入 `q = (1,0,0,0)`（单位四元数），应得 `halfv = (0, 0, 0.5)`——即"世界上方向"在机体坐标系里就是 +Z，符合"平放时 Z 轴朝上"的直觉。

#### 步骤 ③ 叉积求误差

```cpp
halfex = (ay*halfvz - az*halfvy);
```

叉积 `measured × estimated` 的几何意义：**从估计方向转到测量方向所需的旋转轴**，其模长正比于角度误差。这就是"姿态误差"的度量。

#### 步骤 ④ PI 反馈

```cpp
// 积分项：补偿残余陀螺零偏
s_iFB[0] += MAHONY_TWO_KI * halfex * dt;
gx += s_iFB[0];
// 比例项：快速纠正姿态
gx += MAHONY_TWO_KP * halfex;
```

**为什么 PI 而不是只有 P**：只有 P 时，如果陀螺有恒定零偏，稳态会有固定角度误差（P 控制器的稳态误差）。加上 I 项，误差会被积分到零，同时 I 项的稳态值正好等于那个零偏——**自动辨识并补偿了零偏**。

这就是为什么 §6.3 的静态标定只要求"大致对"，剩下的零头由 I 项在线吃掉。

**为什么 P 的系数用 `Kp` 而代码里叫 `MAHONY_TWO_KP`**：Mahony 原论文里反馈量是 `2*Kp*error`，所以代码里存的是 `2*Kp`。这是常见的"文档与实现差个系数"的情况，**看代码时以代码为准**。

#### 步骤 ⑤ 一阶龙格库塔积分

```cpp
gx *= 0.5f * dt;
float qa = q0, qb = q1, qc = q2;
q0 += (-qb*gx - qc*gy - q3*gz);
q1 += ( qa*gx + qc*gz - q3*gy);
q2 += ( qa*gy - qb*gz + q3*gx);
q3 += ( qa*gz + qb*gy - qc*gx);
```

这是四元数微分方程 `q̇ = 0.5 · q ⊗ ω` 的离散化。

**为什么用 `qa/qb/qc` 三个临时变量**：四个方程必须用**同一时刻**的旧值。如果边算边更新，`q1` 的计算会用到刚更新的 `q0`，结果完全错。**用 `qa,qb,qc` 存旧值，`q0` 因为最后才被覆盖所以不用备份**——这个技巧很经典，但容易看漏。

#### 步骤 ⑥ 归一化

积分会让四元数的模缓慢偏离 1，必须每帧修正：

```cpp
norm = sqrtf(q0²+q1²+q2²+q3²);  // 应接近 1
q *= 1/norm;
```

不归一化的话，几十秒后四元数模长会漂到 1.001 或 0.999，姿态角误差随之增大。

### 6.2 欧拉角转换

```cpp
roll  = atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²));
pitch = asin(2(q0q2 - q3q1));      // 必须 clamp 到 [-1, 1]
yaw   = atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²));
```

**`pitch` 必须钳位**：`asin` 的定义域是 [-1,1]，浮点误差会让参数变成 1.0000001，`asin` 返回 NaN，然后整个姿态就废了。这是**四元数转欧拉角最经典的坑**。

**万向锁**：当 pitch = ±90° 时 roll 和 yaw 会耦合、roll 跳变。手势控制不会让手腕到 ±90°，所以不影响。**但如果发现"手抬到最高时角度乱跳"，就是这个原因**——限制 `full` 角度在 30° 以内也顺带规避了这个问题。

---

## 7. 参数调整速查

**所有参数都在 §2（文件开头 60 行内）**，改完重新编译上传即可。

| 想改什么 | 改哪个宏 | 范围建议 | 影响 |
| --- | --- | --- | --- |
| 控制帧率 | `FRAME_MS` | 20~33 | 越小越跟手，大了会跳帧 |
| 加速度平滑度 | `ACC_WIN` | 2~8 | 大了稳但滞后 |
| 调试输出频率 | `DEBUG_LEVEL` | 0/1/2 | 2 最详细但耗时 |
| 死区 | `VX_DEAD_DEG`、`VY_DEAD_DEG` | 3~8 | 大了不抖但迟钝 |
| 满量程 | `VX_FULL_DEG`、`VY_FULL_DEG` | 20~35 | 小了灵敏但易饱和 |
| 非线性强度 | `MAP_GAMMA` | 1.2~2.2 | 大了柔和，小了灵敏 |
| 方向反了 | `VX_INVERT` 等 | +1/-1 | 只改符号 |
| 安装方向 | `IMU_MOUNT` | 0~3 | 改完必须重标定 |
| 姿态跟随速度 | `MAHONY_TWO_KP` | 0.6~2.0 | 大了快但抖 |
| 零偏补偿强度 | `MAHONY_TWO_KI` | 0~0.05 | 大了会振荡 |
| 云台死区 | `PAN_DEAD_DPS` | 8~20 | 大了防误触 |
| 云台灵敏度 | `PAN_GAIN` | 0.5~2.0 | 大了跟手但易过冲 |
| 无线信道 | `NRF_CHANNEL` | 避开 0~22 | 与 WiFi 冲突时改 |
| 发射功率 | `NRF_PA_LEVEL` | MIN~LOW | 别用 MAX（会掉电） |

---

## 8. 二次开发扩展点

按难度排序，都是很好的加分项：

### 8.1 简单（1~2 天）

| 扩展 | 做法 | 代码位置 |
| --- | --- | --- |
| 串口实时调参 | 解析 `Serial` 输入，把 `#define` 改成变量 | 新增 `parseSerialCmd()`，在 loop 末尾调用 |
| 电池电压监测 | GPIO34（ADC1）接分压电阻测电池 | 加进 `CtrlPacket.dtMs` 旁新字段 |
| 手势锁定 | 双按按键锁定/解锁（防误触） | `pollButton()` |
| 姿态数据记录 | 通过串口以 CSV 存到电脑，用于离线分析 | `DEBUG_CSV` |

### 8.2 中等（3~5 天）

| 扩展 | 做法 |
| --- | --- |
| 手势识别 | 用加速度/角速度的**模板匹配**识别"挥手""画圈"，扩展 `flags` 里的模式位 |
| 双击/长按手势 | 在 `pollButton()` 里加时间窗口判定 |
| 卡尔曼滤波对比 | 用同一组数据跑 Mahony 和 EKF，写进报告做对比实验（**答辩很加分**） |
| 加速度计校准 | 六面法标定加速度零偏，写进 `s_accBias[]` |

### 8.3 进阶（1~2 周）

| 扩展 | 做法 |
| --- | --- |
| WiFi 图传 | ESP32-CAM 做视频回传（但 2.4G 会和 nRF 抢信道，需改 nRF 到 5G 频段不存在——只能错开信道或改用 ESP-NOW） |
| 换成 ESP-NOW | 抛弃 nRF24L01，直接用 ESP32 自带 WiFi 的 ESP-NOW 协议，省一个模块 |
| 9 轴融合 | 加 HMC5883L 磁力计，解决 yaw 漂移（但要处理电机磁干扰） |
| 手势录制回放 | 把手势序列存 SPIFFS，实现"录制一段动作让车重放" |

---

## 9. 与需求的对照检查

| 文档要求 | 代码实现 | 位置 |
| --- | --- | --- |
| 步骤 1（1）串口初始化 | `Serial.begin(115200)` | `setup()` 开头 |
| 步骤 1（2）I2C + ICM20689 + 量程 + 关休眠 | `Wire.begin` + `imuInit()`（含 WHO_AM_I + 三重回读校验） | §6、§11.1 |
| 步骤 1（2）静态校准求零偏 | `calibrateGyro(1000)` | §9 |
| 步骤 1（3）SPI + nRF24 + 参数 + 发送模式 | `radioInit()` | §8 |
| 步骤 1（3）Mahony 四元数初始化 | `mahonyReset()` + `mahonyInitFromAccel()` | §7 |
| 步骤 1 控制量清零 | `pkt` 初始化全 0 | §12 |
| 步骤 2-1 计算 dt | `micros()` 差值 + 限幅 | `loop()` ① |
| 步骤 2-2 读六轴 + 滤波 | `imuReadRaw()` + `accelFilter()` | §6 |
| 步骤 2-3 Mahony → roll/pitch | `mahonyUpdate()` + `mahonyGetEuler()` | §7 |
| 步骤 2-3 yaw 不参与控制 | yaw 只算不用于映射 | §7 |
| 步骤 2-4 死区 + 非线性 + 限幅 | `mapAxis()` | §5 |
| 步骤 2-4 输出 vx/vy/pan/tilt | 四个变量 + 模式区分 | `loop()` ④ |
| 步骤 2-5 组包 | `CtrlPacket` + CRC-8 | §3 |
| 步骤 2-6 非阻塞发送 + 丢帧 | `radioSend()` 关 ACK | §8 |
| 步骤 2-7 串口调试输出 | `DEBUG_LEVEL` / `DEBUG_CSV` | §12 ⑦ |
| 步骤 2-8 帧率控制 | 绝对时刻调度器（替代 `delay`） | §12 ⓪ |

---

## 10. 建议的阅读与练手顺序

给大二同学的实操建议：

1. **第一遍：只看 `setup()` 和 `loop()`**（第 11、12 章），把整体流程搞清楚，其他函数当黑盒。
2. **第二遍：改参数做实验**。把 `VX_DEAD_DEG` 从 4 改到 10，看串口输出如何变化，建立"参数 → 现象"的直觉。
3. **第三遍：读 `mapAxis()` 和 `mahonyUpdate()`**，配合文档 §6.1 一步步对照。这两个函数是本项目的技术核心，答辩一定会问。
4. **第四遍：自己加一个功能**。比如加一个"连按两次切换模式"，或者把死区做成串口可调。**能改才能算学会。**
5. **推荐练习**：把 `DEBUG_CSV` 设成 1，用串口绘图器录一段手部动作，导出数据用 Excel/Python 画图——这既是调试手段，也是结题报告里很漂亮的图。
