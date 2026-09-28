# QMC5883P

QST QMC5883P 三轴磁力计驱动模块。
Driver module for the QST QMC5883P 3-axis magnetometer.

构造时模块通过 I2C（7 位地址 `0x2C`）检查 `CHIP_ID`（期望 `0x80`），按手册的
Normal Mode 示例写入轴符号配置 `0x06`、打开 SET/RESET 并设量程 ±8 G、配置为 Normal 模式、
200 Hz；初始化失败时每 100 ms 重试，直到成功。模块不使用 DRDY 引脚：
`qmc5883p_thread` 线程（`REALTIME` 优先级）每 5 ms 查询一次状态寄存器，溢出时告警；
DRDY 位置位时读取 6 字节原始数据，乘以 1000/3750 mG/LSB，经 `rotation` 旋转后发布。
原始值全为 0 的样本被丢弃，此时重新发布上一次的值。

During construction the module checks `CHIP_ID` over I2C (7-bit address
`0x2C`, expects `0x80`) and, following the datasheet's normal-mode example,
writes the axis sign configuration `0x06`, enables SET/RESET with the ±8 G
range and selects normal mode at 200 Hz; on failure it retries every 100 ms
until it succeeds. No DRDY pin is used: the `qmc5883p_thread` thread
(`REALTIME` priority) polls the status register every 5 ms and logs a warning
on overflow; when the DRDY bit is set it reads the 6 raw bytes, scales them by
1000/3750 mG/LSB, rotates the vector by `rotation` and publishes it. A sample
whose raw values are all zero is discarded and the previous value is published
again.

- Topic：`topic_name`（默认 `qmc5883p_mag`），类型 `Eigen::Matrix<float, 3, 1>`（x, y, z，mG）。
  / Topic `topic_name` (default `qmc5883p_mag`), type `Eigen::Matrix<float, 3, 1>` (x, y, z in mG).
- `OnMonitor()`：读取状态寄存器，溢出时告警；数据出现 NaN 或 Inf 时告警。
  / Reads the status register and warns on overflow; warns when the data contains NaN or Inf.

### RamFS 命令 / RamFS command

模块在 `ramfs` 中注册命令 `qmc5883p`。/ The module adds the command `qmc5883p` to `ramfs`.

```sh
qmc5883p show <time_ms> <interval_ms>   # 每 interval_ms 打印一次磁场（mG），持续 time_ms / print the field (mG) every interval_ms for time_ms
```

## 依赖 / Dependencies

无其他模块依赖，仅使用 LibXR。
No other Modules; LibXR only.

## 构造接口 / Constructor

```cpp
QMC5883P(LibXR::I2C& i2c, LibXR::RamFS& ramfs,
         LibXR::Quaternion<float>&& rotation = {1.0f, 0.0f, 0.0f, 0.0f},
         const char* topic_name = "qmc5883p_mag",
         size_t task_stack_depth = 1536);
```

依赖 / Dependencies:

- `i2c`：芯片所在的 I2C 总线。/ The I2C bus the chip is on.
- `ramfs`：注册 `qmc5883p` 命令的 RamFS。/ RamFS that receives the `qmc5883p` command.

配置 / Configuration:

- `rotation`：安装姿态四元数 (w, x, y, z)，默认单位四元数。/ Mounting rotation quaternion (w, x, y, z), identity by default.
- `topic_name`：发布磁场数据的 Topic 名，默认 `qmc5883p_mag`。/ Name of the magnetic-field topic, default `qmc5883p_mag`.
- `task_stack_depth`：采集线程栈大小，默认 1536。/ Stack size of the acquisition thread, default 1536.

## 使用 / Use

```sh
xrobot module add xrobot-org/QMC5883P
xrobot setup
xrobot instance add xrobot-org/QMC5883P
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖项填为 BSP 中用 `XR_REGISTER` 注册的对象名：
`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of
objects the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/QMC5883P
    id: qmc5883p_0
    args:
      - i2c: i2c1
      - ramfs: ramfs
      - rotation: '{1.0f, 0.0f, 0.0f, 0.0f}'
      - topic_name: '"qmc5883p_mag"'
      - task_stack_depth: '1536'
```

BSP 侧 / BSP side:

```cpp
XR_REGISTER(i2c1, LibXR::I2C);
XR_REGISTER(ramfs, LibXR::RamFS);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。
Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/xrobot-org/QMC5883P`
（在 BSP 中）打印当前的构造函数。
`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/QMC5883P` in a BSP, prints the current
constructor.
