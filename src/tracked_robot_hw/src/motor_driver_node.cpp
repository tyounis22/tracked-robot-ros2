/*
================================================================================
tracked_robot_hw :: motor_driver_node.cpp  (Phase 1 bring-up / hardware driver)

WHAT THIS FILE IS 
- I wrote this ROS 2 node to talk directly to my motor driver board over Linux I²C.
- I use it to do the board-required initialization sequence (encoder polarity + encoder reset),
  then I:
  (A) poll and print encoder totals periodically so I can validate encoders and mapping, and
  (B) optionally run a simple open-loop PWM motion test (forward/stop/back/stop) to prove that
      software → electronics → motion works end-to-end.

WHY I MADE IT
- I needed a “hardware truth layer” for motion:
  - This is the only place I let myself deal with the I²C bus path, slave address, and raw registers.
  - Everything higher level I’ll build later (cmd_vel control, PID, odometry, navigation) depends on this
    node being correct and predictable.

KEY IDEA: Linux I²C access
- My motor driver is an I²C slave at address 0x34 on bus /dev/i2c-1.
- Linux exposes I²C buses as device files (e.g., /dev/i2c-1).
- The register pattern I’m using is:
    1) I select a register by writing 1 byte (the register address)
    2) I read or write the associated bytes

BOARD-SPECIFIC RULES I LEARNED (from the docs you provided)
- When I use register 0x15 (encoder polarity), I must set it to 0 or direction changes may not work.
- When I reset encoder pulse totals, I must write sixteen bytes of 0x00 to register 0x3C.
  (This matches 4 motors × 4 bytes each = 16 bytes.)

DATA ASSUMPTIONS (validated by “it moved”)
- REG_ENCODER_TOTAL (0x3C) returns 16 bytes interpreted as:
    M1_count(int32 little-endian), M2_count(int32 LE), M3_count(int32 LE), M4_count(int32 LE)
- REG_MOTOR_FIXED_PWM (0x1F) accepts 4 signed bytes:
    [M1_pwm, M2_pwm, M3_pwm, M4_pwm] each in range -100..100
  This is open-loop power (no regulation).

THE DIRECTION FIX I ADDED
- My robot drove backward when I told it to go forward, so I added a parameter called direction_sign.
- direction_sign can be +1 (normal) or -1 (inverted).
- I apply direction_sign to every PWM value before writing to the motor driver.
- This lets me fix forward/backward without rewiring and without editing code again:
    ros2 run tracked_robot_hw motor_driver_node --ros-args -p direction_sign:=-1

ROS BEHAVIOR
- On startup I:
    - Open the I²C bus file
    - Select the I²C slave device address
    - Initialize the board (polarity, motor type, encoder reset)
    - Stop motors (safety)
    - Start a timer that polls encoder totals at encoder_poll_hz
    - Optionally schedule a one-time PWM test
- On shutdown I:
    - Stop motors (safety)
    - Close the I²C file descriptor

FUNCTION-BY-FUNCTION WALKTHROUGH (what each function does)

(1) i2c_write_bytes(fd, data, len)
    - I write raw bytes to the currently selected I²C slave.
    - If this fails, it’s usually the wrong bus, permissions, missing hardware, or unstable wiring/power.

(2) i2c_write_reg_block(fd, reg, data, len)
    - I write a register plus its payload in one transaction:
        [reg][payload...]
    - I use this for: polarity, motor type, encoder reset, and PWM commands.

(3) i2c_read_reg_block(fd, reg, out, len)
    - I read bytes from a register by:
        a) writing the register address (1 byte)
        b) reading len bytes into out

(4) decode_4x_i32_le(buf16)
    - I decode 16 raw bytes into 4 signed 32-bit integers, little-endian.
    - This converts “wire bytes” into meaningful encoder totals.

(5) MotorDriverNode constructor
    - I declare ROS parameters so I can change behavior without recompiling:
        i2c_bus, i2c_addr, encoder_poll_hz
        do_pwm_test, test_pwm
        motor_type
        direction_sign  (+1 normal, -1 invert direction)
    - I open the I²C bus and select the slave address.
    - I run my board-required init sequence:
        a) set encoder polarity (0x15) to 0
        b) set motor type (0x14) for M1..M4
        c) reset encoder totals (0x3C) by writing 16 zeros
    - I stop motors by default for safety.
    - I start:
        - encoder_timer_ → poll_encoders()
        - test_timer_ → run_pwm_test_once() (only if do_pwm_test=true)

(6) MotorDriverNode destructor
    - I stop motors so the robot won’t keep driving if the process exits.
    - I close the I²C file descriptor.

(7) poll_encoders()
    - I read encoder totals from 0x3C, decode them, and print M1..M4.
    - This helps me confirm encoder direction and motor-to-track mapping.

(8) set_pwm_all(m1,m2,m3,m4)
    - I apply direction_sign_ to each requested PWM value.
    - Then I write the 4 signed PWM bytes to 0x1F.
    - This is the direct actuation path.

(9) stop_motors()
    - I set all PWM outputs to 0.

(10) run_pwm_test_once()
    - If enabled, I run: forward 1s → stop 1s → backward 1s → stop.
    - I cancel my own timer so it doesn’t repeat.

(11) main()
    - Standard ROS 2 program entry: init → spin node → shutdown.

NEXT EVOLUTION (Phase 1 proper)
- I will replace the PWM test with:
    - /cmd_vel subscriber
    - encoder delta → wheel velocity estimation
    - PID closed-loop velocity control
    - /odom publishing + TF (odom → base_link)

================================================================================
*/

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

// ===== Motor driver register addresses =====
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
  // Pattern: write register address, then read N bytes
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

    encoder_poll_hz_ = declare_parameter<int>("encoder_poll_hz", 10);

    do_pwm_test_ = declare_parameter<bool>("do_pwm_test", false);
    test_pwm_ = declare_parameter<int>("test_pwm", 70);  // 0..100

    motor_type_ = declare_parameter<int>("motor_type", 0);

    // Direction fix: +1 = normal, -1 = invert forward/backward
    direction_sign_ = declare_parameter<int>("direction_sign", 1);
    if (direction_sign_ != 1 && direction_sign_ != -1) {
      RCLCPP_WARN(get_logger(), "direction_sign must be +1 or -1. Using +1.");
      direction_sign_ = 1;
    }

    RCLCPP_INFO(get_logger(), "Starting motor driver node");
    RCLCPP_INFO(get_logger(), "I2C bus: %s  addr: 0x%02X", i2c_bus_.c_str(), i2c_addr_);
    RCLCPP_INFO(get_logger(), "direction_sign=%d (forward/backward invert if -1)", direction_sign_);

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

    // ---- Board-specific init sequence ----

    // 1) Encoder polarity MUST be set to 0.
    {
      uint8_t pol[4] = {0, 0, 0, 0};
      if (!i2c_write_reg_block(fd_, REG_ENCODER_POLARITY, pol, sizeof(pol))) {
        RCLCPP_WARN(get_logger(), "Failed to set encoder polarity to 0");
      } else {
        RCLCPP_INFO(get_logger(), "Encoder polarity set to 0");
      }
    }

    // 2) Set motor type for all four motors (safe default is 0).
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

    // Safety: stop motors at startup
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
  int direction_sign_{1};

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
    // Apply direction_sign_ so I can invert forward/backward globally without rewiring.
    int8_t vals[4] = {
      (int8_t)(m1 * direction_sign_),
      (int8_t)(m2 * direction_sign_),
      (int8_t)(m3 * direction_sign_),
      (int8_t)(m4 * direction_sign_)
    };

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

    // Forward (sign handled by direction_sign_)
    RCLCPP_INFO(get_logger(), "PWM test: forward for 1s (PWM=%d)", test_pwm_);
    set_pwm_all((int8_t)test_pwm_, (int8_t)test_pwm_, (int8_t)test_pwm_, (int8_t)test_pwm_);
    sleep(1);

    // Stop
    RCLCPP_INFO(get_logger(), "PWM test: stop for 1s");
    stop_motors();
    sleep(1);

    // Backward (sign handled by direction_sign_)
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
