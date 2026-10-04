# QMC5883P

QST QMC5883P 三轴磁力计驱动模块 / Driver Module for the QST QMC5883P 3-axis magnetometer

## 1. 模块作用 / Purpose

构造时，QMC5883P 通过 I2C（7 位地址 `0x2C`）检查 `CHIP_ID`（期望 `0x80`），按手册的 Normal Mode 示例写入轴符号配置 `0x06`，打开 SET/RESET 并设量程 ±8 G，配置为 Normal 模式、200 Hz；初始化失败时每 100 ms 重试，直到成功。随后创建线程 `qmc5883p_thread`（`REALTIME` 优先级，栈深 `task_stack_depth`），每 5 ms 查询一次状态寄存器，溢出时输出警告；DRDY 位置位时读取 6 字节原始数据，乘以 1000/3750 mG/LSB，经 `rotation` 旋转后发布。原始值全为 0 的样本被丢弃，此时重新发布上一次的值。

`OnMonitor()` 读取状态寄存器，溢出时输出警告；数据出现 NaN 或 Inf 时同样输出警告。

Upon construction, QMC5883P checks `CHIP_ID` over I2C (7-bit address `0x2C`, expects `0x80`) and, following the datasheet's normal-mode example, writes the axis sign configuration `0x06`, enables SET/RESET with the ±8 G range and selects normal mode at 200 Hz; on failure it retries every 100 ms until it succeeds. It then creates the thread `qmc5883p_thread` (`REALTIME` priority, stack depth `task_stack_depth`), which polls the status register every 5 ms and logs a warning on overflow; when the DRDY bit is set it reads the 6 raw bytes, scales them by 1000/3750 mG/LSB, rotates the vector by `rotation` and publishes it. A sample whose raw values are all zero is discarded and the previous value is published again.

`OnMonitor()` reads the status register and logs a warning on overflow; it also logs a warning when the data contains NaN or Inf.

## 2. RamFS 命令 / RamFS Command

模块向 `ramfs` 添加命令 `qmc5883p`：

- `qmc5883p`：打印用法。
- `qmc5883p show <time_ms> <interval_ms>`：在 `time_ms` 内每隔 `interval_ms` 打印一次磁场（mG）。

The Module adds the command `qmc5883p` to `ramfs`:

- `qmc5883p`: print the usage.
- `qmc5883p show <time_ms> <interval_ms>`: print the magnetic field (mG) every `interval_ms` for `time_ms`.

## 3. 构造接口 / Constructor

```cpp
QMC5883P(LibXR::I2C& i2c, LibXR::RamFS& ramfs,
         LibXR::Quaternion<float>&& rotation = {1.0f, 0.0f, 0.0f, 0.0f},
         const char* topic_name = "qmc5883p_mag",
         size_t task_stack_depth = 1536);
```

依赖：

- `i2c`：芯片所在的 I2C 总线。
- `ramfs`：接收 `qmc5883p` 命令的 RamFS。

配置参数：

- `rotation`：传感器坐标系到应用坐标系的四元数 `{w, x, y, z}`，默认单位四元数。
- `topic_name`：发布磁场数据的 Topic 名称，默认 `qmc5883p_mag`。
- `task_stack_depth`：采集线程栈深，单位字节，默认 1536。

Dependencies:

- `i2c`: the I2C bus the chip is on.
- `ramfs`: the RamFS that receives the `qmc5883p` command.

Configuration parameters:

- `rotation`: quaternion `{w, x, y, z}` from the sensor frame to the application frame, default identity.
- `topic_name`: name of the magnetic-field Topic, default `qmc5883p_mag`.
- `task_stack_depth`: stack depth of the acquisition thread in bytes, default 1536.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `topic_name`（默认 `qmc5883p_mag`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 磁场 x、y、z，单位 mG，已按 `rotation` 旋转 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `topic_name` (default `qmc5883p_mag`) | Publish | `Eigen::Matrix<float, 3, 1>` | Magnetic field x, y, z in mG, rotated by `rotation` |

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/QMC5883P` 写入的实例，`i2c` 与 `ramfs` 填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/QMC5883P`, with `i2c` and `ramfs` set to names registered by the BSP with `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/QMC5883P
    id: qmc5883p_0
    args:
      - i2c: i2c1
      - ramfs: ramfs
      - rotation: '{1.0f, 0.0f, 0.0f, 0.0f}'
      - topic_name: "qmc5883p_mag"
      - task_stack_depth: 1536
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一片 QMC5883P 磁力计，通过 I2C 连接，7 位地址 `0x2C`。

Dependencies: LibXR.

Hardware: one QMC5883P magnetometer on I2C with the 7-bit address `0x2C`.
