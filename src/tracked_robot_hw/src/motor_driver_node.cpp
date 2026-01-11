#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

using namespace std::chrono_literals;

// ===== Motor driver register addresses (from your map + notes) =====
static constexpr uint8_t REG_ADC_BATTERY_VOLTAGE     = 0x00;
static constexpr uint8_t REG_MOTOR_TYPE              = 0x14;
static constexpr uint8_t REG_ENCODER_POLARITY        = 0x15;  // MUST be 0 per your note
static constexpr uint8_t REG_MOTOR_FIXED_PWM         = 0x1F;  // open-loop PWM, -100..100
static constexpr uint8_t REG_MOTOR_FIXED_SPEED       = 0x33;  // closed-loop speed (not used yet)
static constexpr uint8_t REG_ENCODER_TOTAL           = 0x3C;  // 4x int32 totals; reset requires 16 bytes of 0x00

// ===== Low-level I2C helpers =====
static bool i2c_write_bytes(int fd, const uint8_t* data, size_t len) {
  if (write(fd, data, len) != (ssize_t)len) {
    perror("I2C write failed");
    return false;
  }
  return true;
}

static bool i2c_write_reg_block(int fd, uint8_t reg, const uint8_t* data, size_t len) {
  // Write: [reg][data...]
  if (len > 64) {
    fprintf(stderr, "i2c_write_reg_block: len too large\n");
    return false;
  }

  uint8_t buf[1 + 64];
  buf[0] = reg;
  if (len > 0) std::memcpy(buf + 1, data, len);

  return i2c_write_bytes(fd, buf, 1 + len);
}

static bool i2c_read_reg_block(int fd, uint8_t reg, uint8_t* out, size_t len) {
  // Typical pattern: write register address, then read N bytes
  if (!i2c_write_bytes(fd, &reg, 1)) return false;

  if (read(fd, out, len) != (ssize_t)len) {
    perror("I2C read failed");
    return false;
  }
  return true;
}

static std::array<int32_t, 4> decode_4x_i32_le(const uint8_t* buf16) {
  // Decode 4 signed int32 from 16 bytes, little-endian, order M1..M4
  std::array<int32_t, 4> out{};
  for (int i = 0; i < 4; ++i) {
    int idx = i * 4;
    uint32_t v =
      (uint32_t)buf16[idx] |
      ((uint32_t)buf16[idx + 1] << 8) |
      ((uint32_t)buf16[idx + 2] << 16) |
      ((uint32_t)buf16[idx + 3] << 24);
    out[i] = (int32_t)v;
  }
  return out;
}

// ===== ROS2 Node =====
class MotorDriverNode : public rclcpp::Node {
public:
  MotorDriverNode() : Node("motor_driver_node") {
    // ---- Parameters (tunable without recompiling) ----
    i2c_bus_ = declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
    i2c_addr_ = declare_parameter<int>("i2c_addr", 0x34);

    // Poll encoder totals at this rate (Hz)
    encoder_poll_hz_ = declare_parameter<int>("encoder_poll_hz", 10);

    // Optional open-loop motion test (forward 1s, stop, backward 1s, stop)
    do_pwm_test_ = declare_parameter<bool>("do_pwm_test", false);
    test_pwm_ = declare_parameter<int>("test_pwm", 70);  // 0..100

    // Motor type setting (0..3 from your table). If unsure, leave 0 for now.
    // NOTE: Some boards expect 1 byte, some expect 4 bytes (M1..M4).
    // We'll write 4 bytes, which is safe for the common "4 motors" layout.
    motor_type_ = declare_parameter<int>("motor_type", 0);

    RCLCPP_INFO(get_logger(), "Starting motor driver node");
    RCLCPP_INFO(get_logger(), "I2C bus: %s  addr: 0x%02X", i2c_bus_.c_str(), i2c_addr_);

    // ---- Open I2C bus ----
    fd_ = open(i2c_bus_.c_str(), O_RDWR);
    if (fd_ < 0) {
      perror("Failed to open I2C bus");
      throw std::runtime_error("open(i2c_bus) failed");
    }

    // ---- Select device address ----
    if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) {
      perror("Failed to select I2C device (I2C_SLAVE)");
      throw std::runtime_error("ioctl(I2C_SLAVE) failed");
    }

    // ---- Board-specific init sequence (based on your notes) ----

    // 1) Encoder polarity MUST be set to 0, otherwise direction changes may fail.
    {
      uint8_t pol[4] = {0, 0, 0, 0};
      if (!i2c_write_reg_block(fd_, REG_ENCODER_POLARITY, pol, sizeof(pol))) {
        RCLCPP_WARN(get_logger(), "Failed to set encoder polarity to 0");
      } else {
        RCLCPP_INFO(get_logger(), "Encoder polarity set to 0");
      }
    }

    // 2) Set motor type (safe default is 0) for all four motors.
    {
      int mt = motor_type_;
      if (mt < 0) mt = 0;
      if (mt > 3) mt = 3;
      uint8_t types[4] = {(uint8_t)mt, (uint8_t)mt, (uint8_t)mt, (uint8_t)mt};

      if (!i2c_write_reg_block(fd_, REG_MOTOR_TYPE, types, sizeof(types))) {
        RCLCPP_WARN(get_logger(), "Failed to set motor type (0x14)");
      } else {
        RCLCPP_INFO(get_logger(), "Motor type set to %d (written to M1..M4)", mt);
      }
    }

    // 3) Reset encoder totals: MUST write 16 bytes of 0x00 to 0x3C.
    {
      uint8_t zeros16[16] = {0};
      if (!i2c_write_reg_block(fd_, REG_ENCODER_TOTAL, zeros16, sizeof(zeros16))) {
        RCLCPP_WARN(get_logger(), "Failed to reset encoders (16-byte write to 0x3C)");
      } else {
        RCLCPP_INFO(get_logger(), "Encoders reset (16 bytes to 0x3C)");
      }
    }

    // Ensure motors are stopped at startup
    stop_motors();

    // ---- Encoder polling timer ----
    if (encoder_poll_hz_ <= 0) encoder_poll_hz_ = 10;
    auto period = std::chrono::milliseconds(1000 / encoder_poll_hz_);
    encoder_timer_ = create_wall_timer(period, std::bind(&MotorDriverNode::poll_encoders, this));

    // ---- Optional PWM test ----
    if (do_pwm_test_) {
      RCLCPP_WARN(get_logger(), "do_pwm_test=true: robot may move! Lift it first.");
      test_timer_ = create_wall_timer(500ms, std::bind(&MotorDriverNode::run_pwm_test_once, this));
    }
  }

  ~MotorDriverNode() override {
    stop_motors();
    if (fd_ >= 0) close(fd_);
  }

private:
  // Parameters
  std::string i2c_bus_;
  int i2c_addr_{0x34};
  int encoder_poll_hz_{10};
  bool do_pwm_test_{false};
  int test_pwm_{70};
  int motor_type_{0};

  // I2C file descriptor
  int fd_{-1};

  // Timers
  rclcpp::TimerBase::SharedPtr encoder_timer_;
  rclcpp::TimerBase::SharedPtr test_timer_;
  bool pwm_test_ran_{false};

  void poll_encoders() {
    uint8_t buf[16]{};

    if (!i2c_read_reg_block(fd_, REG_ENCODER_TOTAL, buf, sizeof(buf))) {
      RCLCPP_ERROR(get_logger(), "Failed to read encoder totals (0x3C)");
      return;
    }

    auto counts = decode_4x_i32_le(buf);

    RCLCPP_INFO(
      get_logger(),
      "Encoders: M1=%d  M2=%d  M3=%d  M4=%d",
      counts[0], counts[1], counts[2], counts[3]
    );
  }

  void set_pwm_all(int8_t m1, int8_t m2, int8_t m3, int8_t m4) {
    // Assumption: REG_MOTOR_FIXED_PWM expects 4 signed bytes [M1,M2,M3,M4]
    int8_t vals[4] = {m1, m2, m3, m4};
    if (!i2c_write_reg_block(fd_, REG_MOTOR_FIXED_PWM,
                             reinterpret_cast<uint8_t*>(vals), 4)) {
      RCLCPP_ERROR(get_logger(), "PWM write failed (0x1F)");
    }
  }

  void stop_motors() {
    set_pwm_all(0, 0, 0, 0);
  }

  void run_pwm_test_once() {
    if (pwm_test_ran_) return;
    pwm_test_ran_ = true;

    if (test_timer_) test_timer_->cancel();

    // Clamp test PWM
    if (test_pwm_ < 0) test_pwm_ = 0;
    if (test_pwm_ > 100) test_pwm_ = 100;

    // Forward
    RCLCPP_INFO(get_logger(), "PWM test: forward for 1s (PWM=%d)", test_pwm_);
    set_pwm_all((int8_t)test_pwm_, (int8_t)test_pwm_, (int8_t)test_pwm_, (int8_t)test_pwm_);
    sleep(1);

    // Stop
    RCLCPP_INFO(get_logger(), "PWM test: stop for 1s");
    stop_motors();
    sleep(1);

    // Backward
    RCLCPP_INFO(get_logger(), "PWM test: backward for 1s (PWM=%d)", test_pwm_);
    set_pwm_all((int8_t)-test_pwm_, (int8_t)-test_pwm_, (int8_t)-test_pwm_, (int8_t)-test_pwm_);
    sleep(1);

    // Stop
    RCLCPP_INFO(get_logger(), "PWM test: stop");
    stop_motors();

    RCLCPP_INFO(get_logger(), "PWM test complete");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotorDriverNode>());
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
