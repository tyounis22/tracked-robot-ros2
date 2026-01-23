/*
================================================================================
tracked_robot_hw :: motor_driver_node.cpp  (Phase 1.3 - encoder odometry + cmd_vel)

Big picture
- I wrote this node because I want ONE place that touches the raw motor driver registers.
- I also want the rest of ROS2 to see my robot as "normal":
    - it should accept /cmd_vel
    - it should publish /odom
    - it should publish TF odom -> base_link (so Nav2/SLAM/etc can plug in later)

My current physical setup (important)
- I have ONLY two motors wired:
    M1 = left track motor
    M2 = right track motor
    M3 and M4 are not wired (unused)
- So I always force M3/M4 PWM to 0 when two_motor_mode=true.

What I calibrated already (measured constants)
- Track circumference: 0.607 m for one full track revolution
- Encoder counts for one revolution at that test:
    M1 = -7194
    M2 = -6694
  => avg ticks per rev ~ 6944
- meters_per_tick = 0.607 / 6944 ≈ 0.0000874 m per tick
- I put meters_per_tick as a ROS parameter, defaulting to that value.

What odometry means (what I’m computing)
- Encoders give me ticks. I convert ticks -> meters using meters_per_tick.
- Every update cycle I compute:
    dl = (left_ticks_delta)  * meters_per_tick
    dr = (right_ticks_delta) * meters_per_tick

- Differential drive kinematics (tracked robot behaves like diff drive, just slips more):
    ds     = (dr + dl) / 2
    dtheta = (dr - dl) / track_width

- Then I integrate pose in the odom frame:
    x     += ds * cos(theta + dtheta/2)
    y     += ds * sin(theta + dtheta/2)
    theta += dtheta

What track_width means here
- track_width is the distance between left and right track centerlines in meters.
- You measured 157mm, so default track_width=0.157.
- Later I can refine track_width to an "effective" value by calibration, but this is a good start.

How /cmd_vel works in this node (for now)
- /cmd_vel gives me a desired linear velocity v (m/s) and yaw rate w (rad/s).
- Eventually I’ll do closed-loop velocity control (PID) using encoders.
- For now I do a simple open-loop mapping:
    left_cmd  = v - w*(track_width/2)
    right_cmd = v + w*(track_width/2)
  then convert m/s -> PWM using a gain (mps_to_pwm).
- It’s crude, but it lets me drive from teleop and still get odom.

Safety / sanity things I do
- I set encoder polarity register 0x15 to 0 (per your docs) so direction changes work properly.
- I reset encoder totals by writing 16 bytes of 0x00 to 0x3C.
- I stop motors at startup and shutdown.
- I implement a cmd_vel timeout: if commands stop arriving, I stop.

Motor driver registers I’m using
- 0x15: encoder polarity (must be 0)
- 0x14: motor type (I write same type to all channels)
- 0x3C: encoder total counts (read 16 bytes; write 16 zeros to reset)
- 0x1F: fixed PWM open-loop command (-100..100)

Assumptions (callout)
- Encoder totals at 0x3C are 4x int32 little-endian: M1..M4.
- Fixed PWM at 0x1F is 4 signed bytes: M1..M4.

================================================================================
*/

#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

// ===== Motor driver register addresses =====
static constexpr uint8_t REG_ADC_BATTERY_VOLTAGE     = 0x00;
static constexpr uint8_t REG_MOTOR_TYPE              = 0x14;
static constexpr uint8_t REG_ENCODER_POLARITY        = 0x15;  // must be 0 per your note
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

static int clamp_int(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static double wrap_pi(double a) {
  // wrap angle to [-pi, pi]
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ===== ROS2 Node =====
class MotorDriverNode : public rclcpp::Node {
public:
  MotorDriverNode() : Node("motor_driver_node") {
    // -------- Parameters (all tweakable without recompiling) --------
    i2c_bus_ = declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
    i2c_addr_ = declare_parameter<int>("i2c_addr", 0x34);

    // Your wiring
    two_motor_mode_ = declare_parameter<bool>("two_motor_mode", true); // M1 left, M2 right
    direction_sign_ = declare_parameter<int>("direction_sign", -1);     // you were using -1 earlier
    if (direction_sign_ != 1 && direction_sign_ != -1) direction_sign_ = 1;

    // Driver config
    motor_type_ = declare_parameter<int>("motor_type", 0);

    // Odometry / kinematics
    meters_per_tick_ = declare_parameter<double>("meters_per_tick", 0.0000874); // from your calibration
    track_width_ = declare_parameter<double>("track_width", 0.157);             // your ruler measurement (m)

    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    // Update rate for encoder->odom (Hz)
    odom_hz_ = declare_parameter<int>("odom_hz", 30);
    if (odom_hz_ < 5) odom_hz_ = 5;

    // /cmd_vel handling
    cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 0.25);

    // Simple open-loop mapping from m/s to PWM (temporary until PID)
    // If it's too weak/strong, tweak this param.
    mps_to_pwm_ = declare_parameter<double>("mps_to_pwm", 250.0); // (m/s) * gain -> PWM
    max_pwm_ = declare_parameter<int>("max_pwm", 60);             // keep it conservative

    // Optional: print encoders each cycle (can get spammy)
    verbose_encoders_ = declare_parameter<bool>("verbose_encoders", false);

    // -------- ROS interfaces --------
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&MotorDriverNode::on_cmd_vel, this, std::placeholders::_1)
    );

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // -------- Open I2C and init board --------
    RCLCPP_INFO(get_logger(), "Starting motor driver (I2C %s addr 0x%02X)", i2c_bus_.c_str(), i2c_addr_);
    RCLCPP_INFO(get_logger(), "two_motor_mode=%s (M1 left, M2 right), direction_sign=%d",
                two_motor_mode_ ? "true" : "false", direction_sign_);
    RCLCPP_INFO(get_logger(), "meters_per_tick=%.10f, track_width=%.4f", meters_per_tick_, track_width_);
    RCLCPP_INFO(get_logger(), "odom_hz=%d, publish_tf=%s, cmd_vel_timeout_s=%.2f",
                odom_hz_, publish_tf_ ? "true" : "false", cmd_vel_timeout_s_);
    RCLCPP_INFO(get_logger(), "mps_to_pwm=%.1f, max_pwm=%d", mps_to_pwm_, max_pwm_);

    fd_ = open(i2c_bus_.c_str(), O_RDWR);
    if (fd_ < 0) {
      perror("Failed to open I2C bus");
      throw std::runtime_error("open(i2c_bus) failed");
    }
    if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) {
      perror("Failed to select I2C device (I2C_SLAVE)");
      throw std::runtime_error("ioctl(I2C_SLAVE) failed");
    }

    // 1) Encoder polarity MUST be 0
    {
      uint8_t pol[4] = {0, 0, 0, 0};
      if (!i2c_write_reg_block(fd_, REG_ENCODER_POLARITY, pol, sizeof(pol))) {
        RCLCPP_WARN(get_logger(), "Failed to set encoder polarity to 0 (0x15)");
      }
    }

    // 2) Motor type (write to all channels)
    {
      int mt = motor_type_;
      if (mt < 0) mt = 0;
      if (mt > 3) mt = 3;
      uint8_t types[4] = {(uint8_t)mt, (uint8_t)mt, (uint8_t)mt, (uint8_t)mt};
      if (!i2c_write_reg_block(fd_, REG_MOTOR_TYPE, types, sizeof(types))) {
        RCLCPP_WARN(get_logger(), "Failed to set motor type (0x14)");
      }
    }

    // 3) Reset encoder totals: write 16 bytes of 0x00 to 0x3C
    reset_encoders();

    // Always stop motors on startup (safety)
    stop_motors();

    // Grab initial encoder snapshot so first delta isn't garbage
    if (!read_encoders(last_enc_)) {
      RCLCPP_WARN(get_logger(), "Initial encoder read failed; odometry may be weird at first");
      last_enc_ = {0, 0, 0, 0};
    }

    last_time_ = now();

    // Odom update timer
    auto period = std::chrono::milliseconds(1000 / odom_hz_);
    odom_timer_ = create_wall_timer(period, std::bind(&MotorDriverNode::update, this));
  }

  ~MotorDriverNode() override {
    stop_motors();
    if (fd_ >= 0) close(fd_);
  }

private:
  // -------- Parameters / config --------
  std::string i2c_bus_;
  int i2c_addr_{0x34};

  bool two_motor_mode_{true};
  int direction_sign_{-1};
  int motor_type_{0};

  double meters_per_tick_{0.0000874};
  double track_width_{0.157};

  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};
  bool publish_tf_{true};
  int odom_hz_{30};

  double cmd_vel_timeout_s_{0.25};

  double mps_to_pwm_{250.0};
  int max_pwm_{60};

  bool verbose_encoders_{false};

  // -------- State --------
  int fd_{-1};

  // Pose in odom frame
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0}; // yaw

  // Encoder snapshots
  std::array<int32_t, 4> last_enc_{0, 0, 0, 0};

  rclcpp::Time last_time_;

  // cmd_vel target (stored)
  double target_v_{0.0}; // m/s
  double target_w_{0.0}; // rad/s
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  // -------- ROS stuff --------
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr odom_timer_;

  // -------- I2C motor helpers --------
  bool read_encoders(std::array<int32_t, 4>& out) {
    uint8_t buf[16]{};
    if (!i2c_read_reg_block(fd_, REG_ENCODER_TOTAL, buf, sizeof(buf))) return false;
    out = decode_4x_i32_le(buf);
    return true;
  }

  void reset_encoders() {
    uint8_t zeros16[16] = {0};
    if (!i2c_write_reg_block(fd_, REG_ENCODER_TOTAL, zeros16, sizeof(zeros16))) {
      RCLCPP_WARN(get_logger(), "Failed to reset encoders (16-byte write to 0x3C)");
    }
  }

  void set_pwm_raw(int8_t m1, int8_t m2, int8_t m3, int8_t m4) {
    // If I'm only wired for 2 motors, force unused channels to 0.
    if (two_motor_mode_) {
      m3 = 0;
      m4 = 0;
    }

    // Apply direction sign globally so forward/backward is consistent.
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
    set_pwm_raw(0, 0, 0, 0);
  }

  // -------- ROS callbacks / update loop --------
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    target_v_ = msg->linear.x;
    target_w_ = msg->angular.z;
    last_cmd_time_ = now();
  }

  void update() {
    // --- Time delta ---
    rclcpp::Time t = now();
    double dt = (t - last_time_).seconds();
    if (dt <= 0.0) dt = 1.0 / (double)odom_hz_;
    last_time_ = t;

    // --- Read encoders ---
    std::array<int32_t, 4> enc{};
    if (!read_encoders(enc)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Encoder read failed");
      // If encoder read fails, safest is to stop motors (optional).
      stop_motors();
      return;
    }

    if (verbose_encoders_) {
      RCLCPP_INFO(get_logger(), "Encoders: M1=%d M2=%d M3=%d M4=%d", enc[0], enc[1], enc[2], enc[3]);
    }

    // --- Compute deltas (ticks) ---
    int32_t dL_ticks = enc[0] - last_enc_[0]; // M1 = left
    int32_t dR_ticks = enc[1] - last_enc_[1]; // M2 = right
    last_enc_ = enc;

    // Use magnitudes as physical distance; sign comes from tick deltas naturally.
    double dL = (double)dL_ticks * meters_per_tick_;
    double dR = (double)dR_ticks * meters_per_tick_;

    // --- Differential-drive integration ---
    // ds = forward distance, dtheta = yaw change
    double ds = (dR + dL) * 0.5;

    // Guard against nonsense track width
    double W = (track_width_ > 1e-6) ? track_width_ : 0.157;
    double dtheta = (dR - dL) / W;

    // "midpoint" integration helps a bit vs naive Euler
    double theta_mid = theta_ + dtheta * 0.5;
    x_ += ds * std::cos(theta_mid);
    y_ += ds * std::sin(theta_mid);
    theta_ = wrap_pi(theta_ + dtheta);

    // --- Compute velocities for message (approx) ---
    double vx = ds / dt;
    double wz = dtheta / dt;

    // --- Publish odometry ---
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = wz;

    // (Covariances left as zeros for now; later we’ll set something reasonable or fuse with IMU)
    odom_pub_->publish(odom);

    // --- Publish TF odom -> base_link (optional) ---
    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tfmsg;
      tfmsg.header.stamp = t;
      tfmsg.header.frame_id = odom_frame_id_;
      tfmsg.child_frame_id = base_frame_id_;
      tfmsg.transform.translation.x = x_;
      tfmsg.transform.translation.y = y_;
      tfmsg.transform.translation.z = 0.0;
      tfmsg.transform.rotation.x = q.x();
      tfmsg.transform.rotation.y = q.y();
      tfmsg.transform.rotation.z = q.z();
      tfmsg.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tfmsg);
    }

    // --- Simple open-loop /cmd_vel -> PWM (temporary) ---
    // If cmd_vel is stale, stop.
    double cmd_age = (t - last_cmd_time_).seconds();
    if (cmd_age > cmd_vel_timeout_s_) {
      set_pwm_raw(0, 0, 0, 0);
      return;
    }

    // Convert base command to left/right linear velocities (m/s)
    // left = v - w*(W/2), right = v + w*(W/2)
    double vL = target_v_ - target_w_ * (W * 0.5);
    double vR = target_v_ + target_w_ * (W * 0.5);

    // Convert desired m/s to PWM (super rough mapping for now)
    int pwmL = (int)std::lround(vL * mps_to_pwm_);
    int pwmR = (int)std::lround(vR * mps_to_pwm_);

    pwmL = clamp_int(pwmL, -max_pwm_, max_pwm_);
    pwmR = clamp_int(pwmR, -max_pwm_, max_pwm_);

    // Apply to M1/M2 (M3/M4 forced to 0 by two_motor_mode)
    set_pwm_raw((int8_t)pwmL, (int8_t)pwmR, 0, 0);
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
