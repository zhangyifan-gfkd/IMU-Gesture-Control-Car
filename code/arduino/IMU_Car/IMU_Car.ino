/*
 * =============================================================================
 *  IMU_Car.ino  ——  “IMU 人体姿态控制小车” 手套采集端固件（ESP32）
 * =============================================================================
 *  硬件平台 : ESP32 DevKit（ESP32-WROOM-32 / ESP32-WROVER 均可）
 *  传感器   : ICM20689（六轴：三轴加速度 + 三轴陀螺仪，I2C，WHO_AM_I = 0x98）
 *  无线     : nRF24L01+（2.4G，SPI，3.3V）
 *  第三方库 : RF24 by TMRh20（Arduino 库管理器搜索 "RF24" 安装）
 *             —— IMU 走直接寄存器读写，不需要装任何 IMU 库
 *                （若想改用现成的 ICM20689 库，见 docs/code_walkthrough.md §5.2）
 *
 *  本文件实现（对应《分层说明及具体实施步骤》步骤 1 与步骤 2-1 ~ 2-8）：
 *    步骤 1   上电初始化：I2C/ICM20689、陀螺零偏标定、SPI/nRF24L01、Mahony 四元数
 *    步骤 2-1 采样本帧时间间隔 dt
 *    步骤 2-2 读取 ICM20689 原始数据 + 滑动平均滤波
 *    步骤 2-3 Mahony 六轴姿态解算 -> roll / pitch
 *    步骤 2-4 手势映射：死区 -> 非线性增益 -> 限幅
 *    步骤 2-5 组包（带帧头 + CRC-8）
 *    步骤 2-6 nRF24L01 非阻塞发送（丢帧不补发）
 *    步骤 2-7 串口调试输出（人读格式 / CSV 绘图格式二选一）
 *    步骤 2-8 固定 20ms 帧周期调度
 *
 *  ---------------------------------------------------------------- 使用提示 --
 *  1) Arduino IDE 要求「文件夹名 == 主文件名」。若打开时报错，请把本文件放到
 *     一个名为 IMU_Car 的文件夹里，即 IMU_Car/IMU_Car.ino。
 *  2) 开发板选择 "ESP32 Dev Module"，上传速度 921600。
 *  3) 上电前把戴手套的手平放静止 2 秒（标定 + 水平基准），串口会提示进度。
 *  4) 串口输出全部使用 ASCII 英文，避免串口监视器中文乱码；中文说明见注释。
 *
 *  作者 / 版本 : 物联网工程 大创项目组   v1.0.0
 * =============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>

#define FW_VERSION "1.0.0"

/* =============================================================================
 * 1. 引脚定义  （见 docs/wiring_diagram.md，改线必须同步改这里）
 * ========================================================================== */
#define PIN_I2C_SDA     21   // ICM20689 SDA
#define PIN_I2C_SCL     22   // ICM20689 SCL
#define PIN_NRF_CE       4   // nRF24L01 CE
#define PIN_NRF_CSN      5   // nRF24L01 CSN
#define PIN_NRF_SCK     18   // nRF24L01 SCK  (ESP32 VSPI)
#define PIN_NRF_MISO    19   // nRF24L01 MISO
#define PIN_NRF_MOSI    23   // nRF24L01 MOSI
#define PIN_LED          2   // 板载 LED（状态指示）
#define PIN_BTN          0   // 板载 BOOT 按键：短按切模式，长按重标定

/* =============================================================================
 * 2. 可调参数
 * ========================================================================== */

/* --- 2.1 帧率与滤波 --- */
#define FRAME_MS          20     // 主循环周期 20ms -> 50Hz
#define ACC_WIN            4     // 加速度滑动平均窗口（4 点 ≈ 80ms）
#define DEBUG_LEVEL        1     // 0=静默 1=每 5 帧一行 2=每帧一行
#define DEBUG_CSV          0     // 1=输出 CSV（Arduino 串口绘图器用）

/* --- 2.2 ICM20689 量程（与初始化写入的寄存器必须一致） --- */
// 加速度 ±4g  -> 8192 LSB/g      陀螺 ±500°/s -> 65.5 LSB/(°/s)
// 注：ICM20689 在相同量程下的灵敏度与 MPU6050 完全一致（16 bit 满量程均分），
//     所以换传感器时这两个常量不用改；改量程才需要同步改。
#define ACC_LSB_PER_G   8192.0f
#define GYRO_LSB_PER_DPS  65.5f
#define IMU_I2C_ADDR      0x68   // AD0(SDO) 接 GND；若接 VCC 则改为 0x69

/* --- 2.3 安装方向预设：把芯片坐标系旋转到「手套坐标系」 ---
 *  手套坐标系约定：X = 手背朝指尖方向，Y = 拇指侧，Z = 手背朝上
 *  0 : 芯片 X/Y/Z 与手套 X/Y/Z 同向（最常见：模块立着贴在手套背面）
 *  1 : 芯片绕 Z 轴逆时针 90°
 *  2 : 芯片绕 X 轴 180°（模块上下颠倒）
 *  3 : 芯片绕 Y 轴 180°（模块左右对调）
 *  改这里之后一定要重新上电标定，并在串口里确认 roll/pitch 方向正确。
 */
#define IMU_MOUNT          0

/* --- 2.4 手势映射参数（步骤 2-4） --- */
#define VX_DEAD_DEG      4.0f     // 前后死区
#define VX_FULL_DEG     30.0f     // 前后满量程
#define VY_DEAD_DEG      4.0f     // 左右死区
#define VY_FULL_DEG     30.0f     // 左右满量程
#define TILT_DEAD_DEG    2.0f
#define TILT_FULL_DEG   30.0f
#define MAP_GAMMA        1.6f     // 非线性指数：>1 小角度更细腻，<1 小角度更灵敏
#define OUT_MAX          100      // 输出限幅 ±100
#define VX_INVERT        (+1)     // 前倾方向反了就把 +1 改成 -1
#define VY_INVERT        (+1)
#define TILT_INVERT      (+1)

/* --- 2.5 云台水平增量（用陀螺角速度做相对增量，规避 yaw 漂移） --- */
#define PAN_DEAD_DPS     8.0f     // 手部扭转角速度死区，低于此视为噪声
#define PAN_GAIN         1.0f     // 扭转角速度 -> 云台角度 的增益
#define PAN_MAX_STEP_DEG 30.0f    // 单帧最大云台转角，防止甩手把云台打飞
#define PAN_INVERT       (+1)
#define PAN_IN_DRIVE      0       // 0=仅云台模式允许平移云台；1=行进模式也可

/* --- 2.6 标定 --- */
#define GYRO_CAL_SAMPLES 1000     // 陀螺零偏采样点数（约 1~2 秒）
#define LEVEL_WARMUP_MS  500      // 标定后等待姿态收敛再取水平基准

/* --- 2.7 nRF24L01 --- */
#define NRF_CHANNEL        76     // 避开 WiFi 1/6/11 信道
#define NRF_PA_LEVEL  RF24_PA_LOW // 无外置 PA 时不要用 RF24_PA_MAX，否则易掉电复位

/* =============================================================================
 * 3. 数据类型（放在文件最前面：Arduino 会自动生成函数原型，原型里可能用到它们）
 * ========================================================================== */

/* --- 3.1 空中通信包：手 -> 车，固定 14 字节，必须与树莓派端解码严格一致 ---
 * 说明：本结构体字段顺序恰好天然对齐（加不加 pack 实测都是 14 字节），
 *       pack(1) 是为了抵抗将来的字段增删/顺序调整，以及不同编译器的默认对齐差异；
 *       真正锁死契约的是下面的 static_assert。
 */
#define PKT_HEAD 0xA5

/* flags 位定义 */
#define FLAG_ARMED       0x01  // 1=手势有效（已越过死区）
#define FLAG_MODE_GIMBAL 0x02  // 1=云台模式  0=行进模式
#define FLAG_IMU_OK      0x04  // 1=IMU（ICM20689）工作正常
#define FLAG_CALIB       0x08  // 1=正在标定（此帧控制量无效）
#define FLAG_TXFAIL      0x10  // 1=上一帧射频发送失败

#pragma pack(push, 1)
typedef struct {
  uint8_t  head;      // [0]     帧头 0xA5
  uint8_t  seq;       // [1]     帧序号，每发一帧 +1，用于统计丢包率
  uint8_t  flags;     // [2]     FLAG_*
  int8_t   vx;        // [3]     前后速度  -100..100
  int8_t   vy;        // [4]     左右速度  -100..100
  int8_t   tilt;      // [5]     云台俯仰  -100..100
  int16_t  panDelta;  // [6-7]   云台水平增量，单位 0.1°（有符号，小端）
  int16_t  roll10;    // [8-9]   当前 roll，单位 0.1°，调试/上位机用
  int16_t  pitch10;   // [10-11] 当前 pitch，单位 0.1°
  uint8_t  dtMs;      // [12]    本帧实际耗时 ms，看帧率是否稳定
  uint8_t  crc;       // [13]    前 13 字节的 CRC-8（poly 0x07, init 0x00）
} CtrlPacket;
#pragma pack(pop)

// 编译期断言：一旦结构体大小偏离协议约定（例如误删上面那行 #pragma pack），编译直接失败
static_assert(sizeof(CtrlPacket) == 14, "CtrlPacket must be exactly 14 bytes");

/* --- 3.2 运行模式 --- */
typedef enum { MODE_DRIVE = 0, MODE_GIMBAL = 1 } RunMode;

/* =============================================================================
 * 4. 全局对象与状态
 * ========================================================================== */
static RF24  s_radio(PIN_NRF_CE, PIN_NRF_CSN);
static const uint8_t NRF_ADDR[6] = {'G', 'L', 'V', '0', '1'};

static bool     s_imuOk       = false;
static uint8_t  s_imuWhoAmI   = 0;      // 上次读到的 WHO_AM_I，出错时打印用
static bool     s_radioOk     = false;
static bool     s_calibrating = false;
static RunMode  s_mode        = MODE_DRIVE;

static float s_gyroBias[3]  = {0, 0, 0};   // 陀螺零偏，单位 °/s
static float s_levelRoll    = 0.0f;        // 上电水平基准（°）
static float s_levelPitch   = 0.0f;

static float s_roll = 0.0f, s_pitch = 0.0f, s_yaw = 0.0f;

/* 加速度滑动平均缓冲 */
static float   s_accBuf[3][ACC_WIN];
static float   s_accSum[3] = {0, 0, 0};
static uint8_t s_accIdx    = 0;
static bool    s_accReady  = false;

/* 时间戳 */
static uint32_t s_lastUs  = 0;
static uint32_t s_frameNo = 0;
static uint32_t s_nextMs  = 0;

/* 按键 */
static bool     s_btnLast   = true;
static uint32_t s_btnDownMs = 0;
static bool     s_btnLong   = false;

/* =============================================================================
 * 5. 通用工具
 * ========================================================================== */

/* CRC-8：多项式 0x07，初值 0x00（与树莓派端 / 上位机一致） */
static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

/* 步骤 2-4 的核心：单轴映射。死区 -> 归一化 -> 非线性增益 -> 限幅取整 */
static int8_t mapAxis(float deg, float dead, float full, int8_t invert) {
  float magnitude = fabsf(deg) - dead;
  if (magnitude <= 0.0f) return 0;                    // 1) 死区：消除手抖

  float s = magnitude / (full - dead);                // 2) 归一化到 0..1
  if (s > 1.0f) s = 1.0f;                             //    超量程饱和

  int8_t out = (int8_t)(powf(s, MAP_GAMMA) * OUT_MAX + 0.5f);  // 3) 非线性增益
  out = (int8_t)(out * invert);                       // 4) 方向修正
  return (deg > 0.0f) ? out : (int8_t)(-out);
}

/* 状态灯：心跳闪烁；射频异常时快闪；标定时长亮 */
static void tickLed() {
  uint32_t now = millis();
  if (s_calibrating) {
    digitalWrite(PIN_LED, HIGH);
    return;
  }
  if (!s_radioOk) {                       // 射频没起来：100ms 快闪
    digitalWrite(PIN_LED, ((now / 100) % 2) ? HIGH : LOW);
    return;
  }
  digitalWrite(PIN_LED, (now % 1000 < 50) ? HIGH : LOW);   // 心跳
}

/* =============================================================================
 * 6. IMU 驱动：ICM20689（六轴，I2C，直接寄存器操作，不依赖第三方库）
 *
 *   与 MPU6050 的差异 —— 换传感器时最容易踩的三个坑：
 *     ① WHO_AM_I(0x75) 返回 0x98（十进制 152），不是 MPU6050 的 0x68
 *     ② 加速度的数字低通在 ACCEL_CONFIG2(0x1D)，不在 CONFIG(0x1A)；
 *        CONFIG(0x1A) 只作用于陀螺。只写 CONFIG 的话加速度仍是 218Hz 宽带，
 *        姿态会明显更抖，而且很难查出原因。
 *     ③ PWR_MGMT_2(0x6C) 必须显式写 0x00 使能加速度和陀螺，不能假设默认值。
 *
 *   两器件相同的地方（所以姿态解算部分一行都不用改）：
 *     - 数据寄存器布局一致：从 0x3B 起连续 14 字节
 *       = 加速度 XYZ(6) + 温度(2) + 陀螺 XYZ(6)，均为高字节在前
 *     - GYRO_CONFIG(0x1B) / ACCEL_CONFIG(0x1C) 的量程位都在 bit[4:3]
 *     - PWR_MGMT_1(0x6B) 同样是 bit7 复位、CLKSEL[2:0]=1 选陀螺 X 轴 PLL
 * ========================================================================== */
#define IMU_REG_SMPLRT_DIV     0x19
#define IMU_REG_CONFIG         0x1A   // 陀螺 DLPF
#define IMU_REG_GYRO_CONFIG    0x1B
#define IMU_REG_ACCEL_CONFIG   0x1C
#define IMU_REG_ACCEL_CONFIG2  0x1D   // 加速度 DLPF（ICM20689 与 MPU6050 的关键区别）
#define IMU_REG_ACCEL_XOUT_H   0x3B
#define IMU_REG_PWR_MGMT_1     0x6B
#define IMU_REG_PWR_MGMT_2     0x6C
#define IMU_REG_WHO_AM_I       0x75

#define IMU_WHO_AM_I_EXPECT    0x98   // ICM20689 固定返回 0x98

typedef struct {
  int16_t ax, ay, az;   // 加速度原始值
  int16_t gx, gy, gz;   // 陀螺仪原始值
} RawImu;

static void imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool imuRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // 保留总线
  if ((int)Wire.requestFrom((uint8_t)IMU_I2C_ADDR, len) != (int)len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

static bool imuInit() {
  // --- 6.1 身份校验：ICM20689 必须是 0x98 ---
  uint8_t id = 0;
  if (!imuRead(IMU_REG_WHO_AM_I, &id, 1)) return false;
  s_imuWhoAmI = id;
  if (id != IMU_WHO_AM_I_EXPECT) return false;   // 具体读到什么，setup() 会打印

  // --- 6.2 软复位：必须等复位完成再配置 ---
  imuWrite(IMU_REG_PWR_MGMT_1, 0x80);            // DEVICE_RESET = 1
  delay(100);                                    // 数据手册要求约 100ms
  imuWrite(IMU_REG_PWR_MGMT_1, 0x01);            // 唤醒 + CLKSEL = 陀螺 X 轴 PLL
  delay(10);

  // --- 6.3 使能与采样率 ---
  imuWrite(IMU_REG_PWR_MGMT_2,   0x00);          // 加速度 + 陀螺六轴全部使能
  imuWrite(IMU_REG_SMPLRT_DIV,   0x04);          // 1kHz/(1+4) = 200Hz

  // --- 6.4 数字低通：陀螺和加速度是两个寄存器，必须都写 ---
  imuWrite(IMU_REG_CONFIG,        0x03);         // 陀螺 DLPF     = 41Hz
  imuWrite(IMU_REG_ACCEL_CONFIG2, 0x03);         // 加速度 DLPF   = 44.8Hz

  // --- 6.5 量程 ---
  imuWrite(IMU_REG_GYRO_CONFIG,  0x08);          // FS_SEL=1  -> ±500 °/s
  imuWrite(IMU_REG_ACCEL_CONFIG, 0x08);          // AFS_SEL=1 -> ±4 g

  // --- 6.6 回读校验：写不进去说明 I2C 有问题，早失败早排查 ---
  uint8_t g = 0, a = 0, f = 0;
  if (!imuRead(IMU_REG_GYRO_CONFIG,    &g, 1)) return false;
  if (!imuRead(IMU_REG_ACCEL_CONFIG,   &a, 1)) return false;
  if (!imuRead(IMU_REG_ACCEL_CONFIG2,  &f, 1)) return false;
  if ((g & 0x18) != 0x08) return false;          // 陀螺量程
  if ((a & 0x18) != 0x08) return false;          // 加速度量程
  if ((f & 0x07) != 0x03) return false;          // 加速度 DLPF（最容易被忽略的一条）
  return true;
}

static bool imuReadRaw(RawImu &r) {
  uint8_t b[14];
  if (!imuRead(IMU_REG_ACCEL_XOUT_H, b, 14)) return false;
  r.ax = (int16_t)((b[0]  << 8) | b[1]);
  r.ay = (int16_t)((b[2]  << 8) | b[3]);
  r.az = (int16_t)((b[4]  << 8) | b[5]);
  // b[6], b[7] 是温度，这里不用
  r.gx = (int16_t)((b[8]  << 8) | b[9]);
  r.gy = (int16_t)((b[10] << 8) | b[11]);
  r.gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

/* 安装方向转换：芯片坐标系 -> 手套坐标系（见 IMU_MOUNT） */
static void remapImu(float &ax, float &ay, float &az,
                     float &gx, float &gy, float &gz) {
  float tx = ax, ty = ay, tz = az;
  float ux = gx, uy = gy, uz = gz;
  switch (IMU_MOUNT) {
    case 1:  tx = -ay; ty =  ax; tz =  az;   // 绕 Z 轴 +90°
             ux = -gy; uy =  gx; uz =  gz;   break;
    case 2:  tx =  ax; ty = -ay; tz = -az;   // 绕 X 轴 180°
             ux =  gx; uy = -gy; uz = -gz;   break;
    case 3:  tx = -ax; ty =  ay; tz = -az;   // 绕 Y 轴 180°
             ux = -gx; uy =  gy; uz = -gz;   break;
    default: break;                          // 0：不旋转
  }
  ax = tx; ay = ty; az = tz;
  gx = ux; gy = uy; gz = uz;
}

/* 加速度滑动平均（步骤 2-2 的可选滤波） */
static void accelFilter(const float in[3], float out[3]) {
  if (!s_accReady) {
    for (uint8_t i = 0; i < 3; i++) {
      for (uint8_t k = 0; k < ACC_WIN; k++) s_accBuf[i][k] = in[i];
      s_accSum[i] = in[i] * ACC_WIN;
    }
    s_accReady = true;
  }
  for (uint8_t i = 0; i < 3; i++) {
    s_accSum[i] -= s_accBuf[i][s_accIdx];
    s_accBuf[i][s_accIdx] = in[i];
    s_accSum[i] += in[i];
    out[i] = s_accSum[i] / (float)ACC_WIN;
  }
  s_accIdx = (uint8_t)((s_accIdx + 1) % ACC_WIN);
}

/* =============================================================================
 * 7. Mahony 六轴姿态解算（步骤 2-3）
 *    输入：角速度 rad/s、加速度 g、dt 秒；输出：四元数 + 欧拉角
 *    说明：6 轴 IMU 的 yaw 会漂移，只做显示，不参与控制
 * ========================================================================== */
#define MAHONY_TWO_KP 1.00f   // 2*Kp，Kp=0.5
#define MAHONY_TWO_KI 0.010f  // 2*Ki，Ki=0.005

static float s_q0 = 1.0f, s_q1 = 0.0f, s_q2 = 0.0f, s_q3 = 0.0f;
static float s_iFB[3] = {0, 0, 0};

static void mahonyReset() {
  s_q0 = 1.0f; s_q1 = s_q2 = s_q3 = 0.0f;
  s_iFB[0] = s_iFB[1] = s_iFB[2] = 0.0f;
}

/* 用第一帧加速度直接把四元数置到当前倾角，避免上电后长时间收敛 */
static void mahonyInitFromAccel(float ax, float ay, float az) {
  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n < 1e-3f) return;
  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  // q = q(roll about X) * q(pitch about Y)，yaw 置 0
  s_q0 = cr * cp;
  s_q1 = sr * cp;
  s_q2 = cr * sp;
  s_q3 = -sr * sp;
  float norm = sqrtf(s_q0 * s_q0 + s_q1 * s_q1 + s_q2 * s_q2 + s_q3 * s_q3);
  if (norm > 1e-6f) {
    float inv = 1.0f / norm;
    s_q0 *= inv; s_q1 *= inv; s_q2 *= inv; s_q3 *= inv;
  }
}

static void mahonyUpdate(float gx, float gy, float gz,
                         float ax, float ay, float az, float dt) {
  // 7.1 加速度归一化（得到重力方向单位向量）
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm > 1e-3f) {
    float inv = 1.0f / norm;
    ax *= inv; ay *= inv; az *= inv;

    // 7.2 估计重力方向（四元数旋转矩阵的第三列）
    float halfvx = s_q1 * s_q3 - s_q0 * s_q2;
    float halfvy = s_q0 * s_q1 + s_q2 * s_q3;
    float halfvz = s_q0 * s_q0 - 0.5f + s_q3 * s_q3;

    // 7.3 误差 = 测量重力方向 × 估计重力方向
    float halfex = (ay * halfvz - az * halfvy);
    float halfey = (az * halfvx - ax * halfvz);
    float halfez = (ax * halfvy - ay * halfvx);

    // 7.4 积分反馈：自动补偿残余陀螺零偏
    s_iFB[0] += MAHONY_TWO_KI * halfex * dt;
    s_iFB[1] += MAHONY_TWO_KI * halfey * dt;
    s_iFB[2] += MAHONY_TWO_KI * halfez * dt;
    gx += s_iFB[0]; gy += s_iFB[1]; gz += s_iFB[2];

    // 7.5 比例反馈：把陀螺积分"拉回"重力方向
    gx += MAHONY_TWO_KP * halfex;
    gy += MAHONY_TWO_KP * halfey;
    gz += MAHONY_TWO_KP * halfez;
  }

  // 7.6 一阶龙格库塔积分四元数
  gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
  float qa = s_q0, qb = s_q1, qc = s_q2;
  s_q0 += (-qb * gx - qc * gy - s_q3 * gz);
  s_q1 += ( qa * gx + qc * gz - s_q3 * gy);
  s_q2 += ( qa * gy - qb * gz + s_q3 * gx);
  s_q3 += ( qa * gz + qb * gy - qc * gx);

  // 7.7 归一化
  norm = sqrtf(s_q0 * s_q0 + s_q1 * s_q1 + s_q2 * s_q2 + s_q3 * s_q3);
  if (norm > 1e-6f) {
    float inv = 1.0f / norm;
    s_q0 *= inv; s_q1 *= inv; s_q2 *= inv; s_q3 *= inv;
  }
}

#define RAD2DEG 57.29577951308232f

static void mahonyGetEuler(float &roll, float &pitch, float &yaw) {
  roll  = atan2f(2.0f * (s_q0 * s_q1 + s_q2 * s_q3),
                 1.0f - 2.0f * (s_q1 * s_q1 + s_q2 * s_q2)) * RAD2DEG;
  float sinp = 2.0f * (s_q0 * s_q2 - s_q3 * s_q1);
  if (sinp >  1.0f) sinp =  1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  pitch = asinf(sinp) * RAD2DEG;
  yaw   = atan2f(2.0f * (s_q0 * s_q3 + s_q1 * s_q2),
                 1.0f - 2.0f * (s_q2 * s_q2 + s_q3 * s_q3)) * RAD2DEG;
}

/* =============================================================================
 * 8. nRF24L01 通信（步骤 2-6）
 *    策略：关闭自动应答/自动重发，一帧只发一次。动作捕捉优先低延迟，
 *          丢一帧远好于卡顿（关闭 ACK 后单帧空中时间 <1ms）。
 * ========================================================================== */
static bool radioInit() {
  SPI.begin(PIN_NRF_SCK, PIN_NRF_MISO, PIN_NRF_MOSI, PIN_NRF_CSN);

  if (!s_radio.begin()) return false;
  if (!s_radio.isChipConnected()) return false;

  s_radio.setChannel(NRF_CHANNEL);
  s_radio.setDataRate(RF24_250KBPS);   // 250kbps：速率低但距离/穿透更好
  s_radio.setPALevel(NRF_PA_LEVEL);    // 无外置 PA 时不要开 MAX
  s_radio.setAutoAck(false);           // 关掉自动应答，追求最低延迟
  s_radio.setRetries(0, 0);
  s_radio.setCRCLength(RF24_CRC_16);
  s_radio.setPayloadSize((uint8_t)sizeof(CtrlPacket));
  s_radio.openWritingPipe(NRF_ADDR);
  s_radio.stopListening();             // 发送模式
  return true;
}

/* 非阻塞发送：返回 true 表示本帧已被射频硬件接受 */
static bool radioSend(const CtrlPacket &pkt) {
  if (!s_radioOk) return false;
  return s_radio.write(&pkt, (uint8_t)sizeof(pkt));
}

/* =============================================================================
 * 9. 标定（步骤 1 中的静态校准 + 水平基准）
 * ========================================================================== */

/* 读一帧 -> 滤波 -> 解算，标定与主循环共用同一条数据通路 */
static bool readAttitude(float dt, float &roll, float &pitch, float &yaw, float &gzDps) {
  RawImu r;
  if (!imuReadRaw(r)) return false;

  float raw[3], acc[3];
  raw[0] = (float)r.ax / ACC_LSB_PER_G;
  raw[1] = (float)r.ay / ACC_LSB_PER_G;
  raw[2] = (float)r.az / ACC_LSB_PER_G;
  accelFilter(raw, acc);

  float ax = acc[0], ay = acc[1], az = acc[2];
  float gx = ((float)r.gx / GYRO_LSB_PER_DPS - s_gyroBias[0]) * (float)DEG_TO_RAD;
  float gy = ((float)r.gy / GYRO_LSB_PER_DPS - s_gyroBias[1]) * (float)DEG_TO_RAD;
  float gz = ((float)r.gz / GYRO_LSB_PER_DPS - s_gyroBias[2]) * (float)DEG_TO_RAD;

  remapImu(ax, ay, az, gx, gy, gz);
  gzDps = gz * RAD2DEG;

  mahonyUpdate(gx, gy, gz, ax, ay, az, dt);
  mahonyGetEuler(roll, pitch, yaw);
  return true;
}

static void calibrateGyro(uint16_t samples) {
  float sum[3] = {0, 0, 0};
  uint16_t got = 0;
  RawImu r;

  for (uint16_t i = 0; i < samples; i++) {
    if (imuReadRaw(r)) {
      sum[0] += (float)r.gx;
      sum[1] += (float)r.gy;
      sum[2] += (float)r.gz;
      got++;
    }
    delay(2);
  }
  if (got == 0) { s_gyroBias[0] = s_gyroBias[1] = s_gyroBias[2] = 0; return; }

  s_gyroBias[0] = (sum[0] / got) / GYRO_LSB_PER_DPS;   // °/s
  s_gyroBias[1] = (sum[1] / got) / GYRO_LSB_PER_DPS;
  s_gyroBias[2] = (sum[2] / got) / GYRO_LSB_PER_DPS;

  Serial.printf("CAL  gyro_bias(dps) = %+.2f %+.2f %+.2f  (n=%u)\n",
                s_gyroBias[0], s_gyroBias[1], s_gyroBias[2], got);
}

/* 取水平基准：要求此刻手套平放（对应文档"上电瞬间手套必须平放不动"） */
static void captureLevel() {
  float roll = 0, pitch = 0, yaw = 0, gz = 0;
  uint8_t okCount = 0;
  float rollSum = 0, pitchSum = 0;

  uint32_t t0 = millis();
  while (millis() - t0 < LEVEL_WARMUP_MS) {
    if (readAttitude((float)FRAME_MS / 1000.0f, roll, pitch, yaw, gz)) {
      rollSum  += roll;
      pitchSum += pitch;
      okCount++;
    }
    delay(FRAME_MS);
  }

  if (okCount > 0) {
    s_levelRoll  = rollSum  / okCount;
    s_levelPitch = pitchSum / okCount;
  }
  Serial.printf("CAL  level_ref = roll %+.2f  pitch %+.2f\n", s_levelRoll, s_levelPitch);
}

/* =============================================================================
 * 10. 按键：短按切模式，长按(>1.5s)重取水平基准
 * ========================================================================== */
static void pollButton() {
  bool pressed = (digitalRead(PIN_BTN) == LOW);
  uint32_t now = millis();

  if (pressed && !s_btnLast) {              // 按下
    s_btnDownMs = now;
    s_btnLong   = false;
  } else if (pressed && !s_btnLong && (now - s_btnDownMs > 1500)) {
    s_btnLong = true;                       // 长按：重取水平基准
    Serial.println("BTN  long press -> re-capture level reference");
    s_calibrating = true;
    captureLevel();
    s_calibrating = false;
    s_lastUs = micros();                    // 吃掉标定耗时，避免 dt 突变
  } else if (!pressed && s_btnLast) {       // 抬起
    if (!s_btnLong && (now - s_btnDownMs > 30)) {   // 30ms 消抖 + 短按
      s_mode = (s_mode == MODE_DRIVE) ? MODE_GIMBAL : MODE_DRIVE;
      Serial.printf("BTN  mode -> %s\n", (s_mode == MODE_DRIVE) ? "DRIVE" : "GIMBAL");
    }
  }
  s_btnLast = pressed;
}

/* =============================================================================
 * 11. setup()：步骤 1 上电初始化
 * ========================================================================== */
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  pinMode(PIN_BTN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=========================================");
  Serial.printf (" IMU Gesture Car - Glove TX  v%s\n", FW_VERSION);
  Serial.println("=========================================");

  // ---- 11.1 I2C + ICM20689 ----
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  s_imuOk = imuInit();
  if (!s_imuOk) {
    Serial.printf("ERR  ICM20689 init failed (WHO_AM_I=0x%02X, expect 0x98)\n",
                  (unsigned)s_imuWhoAmI);
    Serial.println("     check SDA21/SCL22, 3V3, GND, AD0/SDO, and CS tied to 3V3");
    // 不直接死等：继续初始化射频，方便区分是哪个模块坏了
  } else {
    Serial.printf("OK   ICM20689 ready (id=0x%02X, gyro +-500dps, accel +-4g)\n",
                  (unsigned)s_imuWhoAmI);
  }

  // ---- 11.2 陀螺零偏标定（手套必须静止放平）----
  s_calibrating = true;
  if (s_imuOk) {
    Serial.println("CAL  keep the glove STILL and LEVEL for ~2s ...");
    calibrateGyro(GYRO_CAL_SAMPLES);
  }
  s_calibrating = false;

  // ---- 11.3 姿态基准 ----
  if (s_imuOk) {
    RawImu r;
    if (imuReadRaw(r)) {
      float ax = (float)r.ax / ACC_LSB_PER_G;
      float ay = (float)r.ay / ACC_LSB_PER_G;
      float az = (float)r.az / ACC_LSB_PER_G;
      float gx = 0.0f, gy = 0.0f, gz = 0.0f;   // 初值只用加速度，角速度占位
      remapImu(ax, ay, az, gx, gy, gz);
      mahonyReset();
      mahonyInitFromAccel(ax, ay, az);
    }
    captureLevel();
  }

  // ---- 11.4 SPI + nRF24L01 ----
  s_radioOk = radioInit();
  if (s_radioOk) {
    Serial.printf("OK   nRF24L01 ready (ch=%d, 250kbps, addr=GLV01)\n", NRF_CHANNEL);
  } else {
    Serial.println("ERR  nRF24L01 init failed: check CE4/CSN5/SCK18/MISO19/MOSI23,");
    Serial.println("     make sure it is powered by 3V3 (NOT 5V) with a 10-100uF cap.");
    Serial.println("     Running in BENCH mode: attitude still printed on serial.");
  }

  // ---- 11.5 控制量清零（步骤 1 最后一项）----
  s_roll = s_pitch = s_yaw = 0.0f;
  s_frameNo = 0;
  s_lastUs  = micros();
  s_nextMs  = millis();

  Serial.printf(" RUN  frame=%dms  mode=%s\n", FRAME_MS,
                (s_mode == MODE_DRIVE) ? "DRIVE" : "GIMBAL");
  // 协议自检：树莓派端解码前先确认双方 sizeof 一致（应为 14）
  Serial.printf("PKT  sizeof=%u  head=0x%02X\n",
                (unsigned)sizeof(CtrlPacket), (unsigned)PKT_HEAD);
  if (DEBUG_CSV) Serial.println("roll,pitch,vx,vy,pan,tilt");
}

/* =============================================================================
 * 12. loop()：步骤 2-1 ~ 2-8
 * ========================================================================== */
void loop() {
  /* ---- 步骤 2-8 前置：固定帧周期调度，到点才干活 ---- */
  uint32_t nowMs = millis();
  if ((int32_t)(nowMs - s_nextMs) < 0) {
    tickLed();
    pollButton();
    return;
  }
  s_nextMs += FRAME_MS;
  if ((int32_t)(nowMs - s_nextMs) > 200) s_nextMs = nowMs + FRAME_MS;  // 卡顿后重同步

  pollButton();

  /* ---- 步骤 2-1 计算 dt ---- */
  uint32_t nowUs = micros();
  float dt = (float)(nowUs - s_lastUs) / 1000000.0f;
  s_lastUs = nowUs;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.100f) dt = 0.100f;           // 限幅，防止卡一帧把解算踹飞

  /* ---- 步骤 2-2 / 2-3 读传感器 + 姿态解算 ---- */
  float dtF = dt * 1000.0f;
  uint8_t dtMsReport = (uint8_t)((dtF > 255.0f) ? 255 : (int)(dtF + 0.5f));

  float rollRel  = 0.0f;
  float pitchRel = 0.0f;
  int8_t  vx = 0, vy = 0, tilt = 0;
  int16_t panDelta = 0;
  uint8_t flags = 0;
  bool    sampleOk = false;
  float   gzDps = 0.0f;

  if (s_imuOk) {
    RawImu r;
    if (imuReadRaw(r)) {
      float raw[3], acc[3];
      raw[0] = (float)r.ax / ACC_LSB_PER_G;
      raw[1] = (float)r.ay / ACC_LSB_PER_G;
      raw[2] = (float)r.az / ACC_LSB_PER_G;
      accelFilter(raw, acc);

      float ax = acc[0], ay = acc[1], az = acc[2];
      float gx = ((float)r.gx / GYRO_LSB_PER_DPS - s_gyroBias[0]) * (float)DEG_TO_RAD;
      float gy = ((float)r.gy / GYRO_LSB_PER_DPS - s_gyroBias[1]) * (float)DEG_TO_RAD;
      float gz = ((float)r.gz / GYRO_LSB_PER_DPS - s_gyroBias[2]) * (float)DEG_TO_RAD;

      remapImu(ax, ay, az, gx, gy, gz);
      gzDps = gz * RAD2DEG;

      mahonyUpdate(gx, gy, gz, ax, ay, az, dt);
      mahonyGetEuler(s_roll, s_pitch, s_yaw);
      sampleOk = true;

      // 相对上电水平基准的角度（保证"平放 = 0°"）
      rollRel  = s_roll  - s_levelRoll;
      pitchRel = s_pitch - s_levelPitch;
    }
  }

  /* ---- 步骤 2-4 手势映射 ---- */
  if (sampleOk) {
    if (s_mode == MODE_DRIVE) {
      vx = mapAxis(pitchRel, VX_DEAD_DEG, VX_FULL_DEG, VX_INVERT);
      vy = mapAxis(rollRel,  VY_DEAD_DEG, VY_FULL_DEG, VY_INVERT);
      tilt = 0;                              // 行进模式云台俯仰保持不动
    } else {
      vx = 0; vy = 0;                        // 云台模式小车不动，专心瞄准
      tilt = mapAxis(pitchRel, TILT_DEAD_DEG, TILT_FULL_DEG, TILT_INVERT);
    }

    // 云台水平：用陀螺 Z 轴角速度做相对增量（yaw 绝对角会漂移，不能用）
    if (s_mode == MODE_GIMBAL || PAN_IN_DRIVE) {
      if (fabsf(gzDps) > PAN_DEAD_DPS) {
        float step = gzDps * dt * PAN_GAIN * (float)PAN_INVERT;
        if (step >  PAN_MAX_STEP_DEG) step =  PAN_MAX_STEP_DEG;
        if (step < -PAN_MAX_STEP_DEG) step = -PAN_MAX_STEP_DEG;
        panDelta = (int16_t)lroundf(step * 10.0f);   // 0.1°
      }
    }

    if (vx != 0 || vy != 0 || tilt != 0 || panDelta != 0) flags |= FLAG_ARMED;
    flags |= FLAG_IMU_OK;
  } else {
    flags |= FLAG_CALIB;
  }
  if (s_mode == MODE_GIMBAL) flags |= FLAG_MODE_GIMBAL;
  if (s_calibrating)         flags |= FLAG_CALIB;

  /* ---- 步骤 2-5 组包 ---- */
  CtrlPacket pkt;
  pkt.head     = PKT_HEAD;
  pkt.seq      = (uint8_t)(s_frameNo & 0xFF);
  pkt.flags    = flags;
  pkt.vx       = vx;
  pkt.vy       = vy;
  pkt.tilt     = tilt;
  pkt.panDelta = panDelta;
  pkt.roll10   = (int16_t)lroundf(rollRel  * 10.0f);
  pkt.pitch10  = (int16_t)lroundf(pitchRel * 10.0f);
  pkt.dtMs     = dtMsReport;
  pkt.crc      = crc8((const uint8_t *)&pkt, (uint8_t)(sizeof(CtrlPacket) - 1));

  /* ---- 步骤 2-6 发送（失败直接丢帧，绝不阻塞重发）---- */
  bool txOk = radioSend(pkt);
  if (!txOk && s_radioOk) pkt.flags |= FLAG_TXFAIL;   // 仅作调试提示

  /* ---- 步骤 2-7 串口调试输出 ---- */
  s_frameNo++;
#if DEBUG_CSV
  if (sampleOk) {
    Serial.printf("%.2f,%.2f,%d,%d,%d,%d\n",
                  rollRel, pitchRel, vx, vy, (int)panDelta, tilt);
  }
#else
  bool verbose = (DEBUG_LEVEL >= 2) || (DEBUG_LEVEL >= 1 && (s_frameNo % 5 == 0));
  if (verbose && sampleOk) {
    Serial.printf("[%6lu] roll=%7.2f pitch=%7.2f | vx=%4d vy=%4d pan=%4d tilt=%4d | %s%s%s imu=%s\n",
                  (unsigned long)s_frameNo,
                  rollRel, pitchRel, vx, vy, (int)panDelta, tilt,
                  (s_mode == MODE_DRIVE) ? "DRIVE " : "GIMBAL",
                  (flags & FLAG_ARMED) ? " ARMED" : " IDLE ",
                  s_radioOk ? (txOk ? " tx=OK  " : " tx=FAIL") : " tx=NONE",
                  s_imuOk ? "OK" : "FAIL");
  }
#endif

  /* ---- 步骤 2-8 延时回到循环开头（由 s_nextMs 调度器完成，无需 delay）---- */
  tickLed();
}
