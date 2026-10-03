#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: QST QMC5883P 三轴磁力计驱动模块 / Driver module for the QST QMC5883P 3-axis magnetometer
depends: []
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>
#include <memory>

#include "i2c.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "transform.hpp"

// ---------------- QMC5883P Registers ----------------
// 根据 QMC5883P 手册的寄存器表：CHIPID / 数据 / 状态 / 控制寄存器
#define QMC5883P_REG_CHIP_ID 0x00

#define QMC5883P_REG_X_LSB 0x01
#define QMC5883P_REG_X_MSB 0x02
#define QMC5883P_REG_Y_LSB 0x03
#define QMC5883P_REG_Y_MSB 0x04
#define QMC5883P_REG_Z_LSB 0x05
#define QMC5883P_REG_Z_MSB 0x06

#define QMC5883P_REG_STATUS 0x09  // bit1:OVFL bit0:DRDY
#define QMC5883P_REG_CTRL1 0x0A   // OSR2[7:6] OSR1[5:4] ODR[3:2] MODE[1:0]
#define QMC5883P_REG_CTRL2 0x0B   // [7]SOFT_RST [6]SELF_TEST [3:2]RNG [1:0]SET/RESET_MODE
#define QMC5883P_REG_AXIS_CFG 0x29  // 轴符号配置，datasheet 示例写 0x06

// ----- I2C address (7-bit) -----
// QMC5883P 的 7 位地址为 0x2C
#define QMC5883P_I2C_ADDR (0x2C)  // raw 7-bit address

// ----- Status bits -----
#define QMC5883P_STATUS_DRDY (1u << 0)
#define QMC5883P_STATUS_OVFL (1u << 1)

// ----- Driver configuration -----
// 按手册 “Normal Mode Setup Example”：
//   0x29 = 0x06 （XYZ 符号定义）
//   0x0B = 0x08 （Set/Reset ON, 量程 8G）
//   0x0A = 0xCD （Normal mode, ODR=200Hz, 高 OSR）
//
// 0xCD = 1100_1101b -> OSR2=3, OSR1=0, ODR=3(200Hz), MODE=1(Normal)
#define QMC5883P_CTRL1_NORMAL_8G_200HZ 0xCD
#define QMC5883P_CTRL2_SETRESET_ON_8G 0x08
#define QMC5883P_AXIS_CFG_DEFAULT 0x06

// 量程 ±8G 时灵敏度 3750 LSB/G（datasheet 表 2）
// 1G = 1000 mG -> 1LSB = 1000/3750 mG ≈ 0.2667 mG/LSB
#define QMC5883P_SCALE_MG_PER_LSB (1000.0f / 3750.0f)

#define QMC5883P_MAG_RX_LEN 6

/**
 * @brief QMC5883P 三轴磁力计驱动，通过 I2C 轮询并发布磁场数据，单位 mG。
 *        Driver for the QMC5883P 3-axis magnetometer; polls it over I2C and publishes
 *        the magnetic field in mG.
 */
class QMC5883P
{
 public:
  /**
   * @brief 构造 QMC5883P，初始化芯片并创建采集线程，初始化失败时每 100 ms 重试。
   *        Construct QMC5883P, initialize the chip and create the acquisition thread;
   *        a failed initialization is retried every 100 ms.
   *
   * @param i2c 芯片所在的 I2C 总线。
   *            I2C bus of the chip.
   * @param ramfs 接收 `qmc5883p` 命令的 RamFS。
   *              RamFS that receives the `qmc5883p` command.
   * @param rotation 安装姿态四元数，分量顺序为 w、x、y、z。
   *                 Mounting rotation quaternion with components in the order w, x, y, z.
   * @param topic_name 磁场 Topic 名称。
   *                   Name of the magnetic-field Topic.
   * @param task_stack_depth 采集线程栈深。
   *                         Stack depth of the acquisition thread.
   */
  QMC5883P(
      LibXR::I2C& i2c,
      LibXR::RamFS& ramfs,
      LibXR::Quaternion<float>&& rotation = {1.0f, 0.0f, 0.0f, 0.0f},
      const char* topic_name = "qmc5883p_mag",
      size_t task_stack_depth = 1536)
      : rotation_(std::move(rotation)),
        topic_mag_(LibXR::Topic::CreateTopic<decltype(mag_data_)>(topic_name)),
        i2c_(std::addressof(i2c)),
        op_i2c_read_(sem_i2c_),
        op_i2c_write_(sem_i2c_),
        cmd_file_(LibXR::RamFS::CreateFile("qmc5883p", CommandFunc, this))
  {
    ramfs.Add(cmd_file_);

    while (!Init())
    {
      XR_LOG_ERROR("QMC5883P: Init failed, retrying...\r\n");
      LibXR::Thread::Sleep(100);
    }

    XR_LOG_PASS("QMC5883P: Init succeeded.\r\n");
    thread_.Create(this, ThreadFunc, "qmc5883p_thread", task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);
  }

  /**
   * @brief 检查 CHIP_ID 并写入配置：Normal 模式、±8 G、200 Hz。
   *        Check CHIP_ID and write the configuration: normal mode, ±8 G, 200 Hz.
   *
   * @return CHIP_ID 正确时为 true，否则为 false。
   *         true when CHIP_ID matches, false otherwise.
   */
  bool Init()
  {
    // 等待上电复位完成，datasheet 要求 PORT >= 250 us，取 1 ms
    LibXR::Thread::Sleep(1);

    // 检查 CHIPID，QMC5883P 在 0x00 处默认值 0x80
    uint8_t chip_id = ReadReg(QMC5883P_REG_CHIP_ID);
    if (chip_id != 0x80)
    {
      XR_LOG_ERROR("QMC5883P: wrong CHIP_ID (0x%02X).\r\n", chip_id);
      return false;
    }

    // 轴符号配置：datasheet 示例 0x06
    WriteReg(QMC5883P_REG_AXIS_CFG, QMC5883P_AXIS_CFG_DEFAULT);

    // Set/Reset 打开，量程设为 ±8G（CTRL2 = 0x08）
    WriteReg(QMC5883P_REG_CTRL2, QMC5883P_CTRL2_SETRESET_ON_8G);

    // Normal mode, ±8G, ODR=200Hz，高 OSR（CTRL1 = 0xCD）
    WriteReg(QMC5883P_REG_CTRL1, QMC5883P_CTRL1_NORMAL_8G_200HZ);

    // 等待内部完成第一次转换
    LibXR::Thread::Sleep(5);

    return true;
  }

  /**
   * @brief 采集线程：每 5 ms 查询状态寄存器，数据就绪时读取并发布磁场数据。
   *        Acquisition thread: poll the status register every 5 ms, then read and
   *        publish the magnetic field when data is ready.
   *
   * @param sensor QMC5883P 实例。
   *               QMC5883P instance.
   */
  static void ThreadFunc(QMC5883P* sensor)
  {
    // 轮询状态寄存器获取数据就绪状态
    while (true)
    {
      // 200Hz ODR -> 5ms 一次新数据，这里用 5ms 周期轮询
      LibXR::Thread::Sleep(5);

      uint8_t st = sensor->ReadReg(QMC5883P_REG_STATUS);

      if (st & QMC5883P_STATUS_OVFL)
      {
        XR_LOG_WARN("QMC5883P: data overflow.\r\n");
      }

      if (st & QMC5883P_STATUS_DRDY)
      {
        // DRDY 置 1，数据就绪
        sensor->ReadMagnetometer();
        sensor->ParseMagData();
        sensor->topic_mag_.Publish(sensor->mag_data_);
      }
    }
  }

  /**
   * @brief 从 X_LSB 起读取 6 字节原始磁场数据到内部缓冲区。
   *        Read the 6 raw magnetic-field bytes starting at X_LSB into the internal
   *        buffer.
   */
  void ReadMagnetometer()
  {
    // 从 X LSB(0x01) 连读 6 字节到 Z MSB(0x06)
    i2c_->MemRead(QMC5883P_I2C_ADDR, QMC5883P_REG_X_LSB, read_buffer_, op_i2c_read_,
                  LibXR::I2C::MemAddrLength::BYTE_8);
  }

  /**
   * @brief 把缓冲区中的原始数据换算为 mG 并按 rotation 旋转，
   *        原始值全为 0 时保持上一次的值。
   *        Convert the buffered raw data to mG and rotate it by rotation;
   *        the previous value is kept when all raw values are 0.
   */
  void ParseMagData()
  {
    std::array<int16_t, 3> raw;
    raw[0] = static_cast<int16_t>((read_buffer_[1] << 8) | read_buffer_[0]);
    raw[1] = static_cast<int16_t>((read_buffer_[3] << 8) | read_buffer_[2]);
    raw[2] = static_cast<int16_t>((read_buffer_[5] << 8) | read_buffer_[4]);

    if (raw[0] == 0 && raw[1] == 0 && raw[2] == 0)
    {
      return;
    }

    // 单位：mG
    Eigen::Matrix<float, 3, 1> vec;
    vec[0] = raw[0] * QMC5883P_SCALE_MG_PER_LSB;
    vec[1] = raw[1] * QMC5883P_SCALE_MG_PER_LSB;
    vec[2] = raw[2] * QMC5883P_SCALE_MG_PER_LSB;

    mag_data_ = rotation_ * vec;
  }

  /**
   * @brief 读取一个寄存器。
   *        Read one register.
   *
   * @param reg 寄存器地址。
   *            Register address.
   * @return 寄存器值。
   *         Register value.
   */
  uint8_t ReadReg(uint8_t reg)
  {
    uint8_t data = 0;
    i2c_->MemRead(QMC5883P_I2C_ADDR, reg, data, op_i2c_read_);
    return data;
  }

  /**
   * @brief 写入一个寄存器。
   *        Write one register.
   *
   * @param reg 寄存器地址。
   *            Register address.
   * @param val 写入值。
   *            Value to write.
   */
  void WriteReg(uint8_t reg, uint8_t val)
  {
    i2c_->MemWrite(QMC5883P_I2C_ADDR, reg, val, op_i2c_write_);
  }

  /**
   * @brief 监控回调：数据溢出或出现 NaN、Inf 时输出告警。
   *        Monitor callback: log a warning on data overflow or when the data contains
   *        NaN or Inf.
   */
  void OnMonitor(void)
  {
    uint8_t st = ReadReg(QMC5883P_REG_STATUS);
    if (st & QMC5883P_STATUS_OVFL)
    {
      XR_LOG_WARN("QMC5883P: data overflow.\r\n");
    }
    if (std::isnan(mag_data_.x()) || std::isnan(mag_data_.y()) ||
        std::isnan(mag_data_.z()) || std::isinf(mag_data_.x()) ||
        std::isinf(mag_data_.y()) || std::isinf(mag_data_.z()))
    {
      XR_LOG_WARN("QMC5883P: NaN or Inf detected.\r\n");
    }
  }

  /**
   * @brief `qmc5883p` 命令入口，`show <time_ms> <interval_ms>` 周期打印磁场。
   *        Entry of the `qmc5883p` command; `show <time_ms> <interval_ms>` prints the
   *        magnetic field periodically.
   *
   * @param sensor QMC5883P 实例。
   *               QMC5883P instance.
   * @param argc 参数个数。
   *             Argument count.
   * @param argv 参数列表。
   *             Argument list.
   * @return 命令返回值，恒为 0。
   *         Command return value, always 0.
   */
  static int CommandFunc(QMC5883P* sensor, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  show [time_ms] [interval_ms] - Print sensor data "
          "periodically.\r\n">();
    }
    else if (argc == 4 && strcmp(argv[1], "show") == 0)
    {
      int time_ms = atoi(argv[2]);
      int interval_ms = atoi(argv[3]);
      for (int i = 0; i < time_ms / interval_ms; i++)
      {
        LibXR::Thread::Sleep(interval_ms);
        LibXR::STDIO::Printf<"Mag(mG): x=%f, y=%f, z=%f\r\n">(
            sensor->mag_data_.x(), sensor->mag_data_.y(), sensor->mag_data_.z());
      }
    }
    return 0;
  }

 private:
  LibXR::Quaternion<float> rotation_;
  Eigen::Matrix<float, 3, 1> mag_data_{0, 0, 0};
  LibXR::Topic topic_mag_;

  LibXR::I2C* i2c_;

  LibXR::Semaphore sem_i2c_;
  LibXR::ReadOperation op_i2c_read_;
  LibXR::WriteOperation op_i2c_write_;

  uint8_t read_buffer_[QMC5883P_MAG_RX_LEN] = {};

  LibXR::RamFS::File cmd_file_;
  LibXR::Thread thread_;
};
