#include "k9_drive_pkg/k9_roboclaw_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include <cctype>
#include <functional>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "k9_drive_pkg/drive_math.hpp"

namespace k9_drive_pkg
{
using namespace drive_math;
namespace
{
// RoboClaw GETERROR/status word (command 90) as implemented by K9's proven
// later RoboClaw driver. K9's USB RoboClaw 2x15A v4.1.34 returns FOUR data
// bytes plus CRC for this command. E-stop is bit 0.
constexpr uint32_t kStatusEStop = 0x00000001;
constexpr uint8_t kS3ModeEStop = 0x01;  // RoboClaw 2x15A firmware v4.1.34+
constexpr uint32_t kStatusTemperature1 = 0x00000002;
constexpr uint32_t kStatusTemperature2 = 0x00000004;
constexpr uint32_t kStatusLogicBatteryHigh = 0x00000010;
constexpr uint32_t kStatusLogicBatteryLow = 0x00000020;
constexpr uint32_t kStatusM1DriverFault = 0x00000040;
constexpr uint32_t kStatusM2DriverFault = 0x00000080;
constexpr uint32_t kStatusM1Speed = 0x00000100;
constexpr uint32_t kStatusM2Speed = 0x00000200;
constexpr uint32_t kStatusM1Position = 0x00000400;
constexpr uint32_t kStatusM2Position = 0x00000800;
constexpr uint32_t kStatusM1Current = 0x00001000;
constexpr uint32_t kStatusM2Current = 0x00002000;
constexpr uint32_t kStatusM1OverCurrentWarning = 0x00010000;
constexpr uint32_t kStatusM2OverCurrentWarning = 0x00020000;
constexpr uint32_t kStatusMainBatteryHighWarning = 0x00040000;
constexpr uint32_t kStatusMainBatteryLowWarning = 0x00080000;
constexpr uint32_t kStatusTemperatureWarning = 0x00100000;
constexpr uint32_t kStatusTemperature2Warning = 0x00200000;
constexpr uint32_t kStatusS4Triggered = 0x00400000;
constexpr uint32_t kStatusS5Triggered = 0x00800000;
constexpr uint32_t kStatusCanWarning = 0x10000000;
constexpr uint32_t kStatusBootWarning = 0x20000000;
constexpr uint32_t kStatusM1OverRegenWarning = 0x40000000;
constexpr uint32_t kStatusM2OverRegenWarning = 0x80000000;

constexpr uint32_t kFaultMask =
  kStatusEStop | kStatusTemperature1 | kStatusTemperature2 |
  kStatusLogicBatteryHigh | kStatusLogicBatteryLow |
  kStatusM1DriverFault | kStatusM2DriverFault |
  kStatusM1Speed | kStatusM2Speed | kStatusM1Position | kStatusM2Position |
  kStatusM1Current | kStatusM2Current;

constexpr uint32_t kWarningMask =
  kStatusM1OverCurrentWarning | kStatusM2OverCurrentWarning |
  kStatusMainBatteryHighWarning | kStatusMainBatteryLowWarning |
  kStatusTemperatureWarning | kStatusTemperature2Warning |
  kStatusS4Triggered | kStatusS5Triggered | kStatusCanWarning |
  kStatusBootWarning | kStatusM1OverRegenWarning | kStatusM2OverRegenWarning;
constexpr double kNearZero = 1.0e-9;

std::string bool_text(bool value) { return value ? "true" : "false"; }
std::string hex32(uint32_t value)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::setw(8) << std::setfill('0') << value;
  return stream.str();
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

template<typename T>
std::string number_text(T value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

}  // namespace

K9RoboClawHardware::~K9RoboClawHardware()
{
  safe_stop_noexcept("hardware interface destruction");
  if (roboclaw_) {
    roboclaw_->close_port();
  }
}

template<typename T>
T K9RoboClawHardware::parameter_or(const std::string & name, const T & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }

  if constexpr (std::is_same_v<T, std::string>) {
    return it->second;
  } else if constexpr (std::is_integral_v<T>) {
    std::size_t consumed = 0;
    const auto raw = std::stoll(it->second, &consumed, 0);
    if (consumed != it->second.size()) {
      throw std::invalid_argument("Invalid integer hardware parameter '" + name + "': " + it->second);
    }
    return static_cast<T>(raw);
  } else {
    std::size_t consumed = 0;
    const auto raw = std::stod(it->second, &consumed);
    if (consumed != it->second.size()) {
      throw std::invalid_argument("Invalid numeric hardware parameter '" + name + "': " + it->second);
    }
    return static_cast<T>(raw);
  }
}

bool K9RoboClawHardware::parameter_bool(const std::string & name, bool default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  std::string value = it->second;
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (value == "true" || value == "1" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no" || value == "off") {
    return false;
  }
  throw std::invalid_argument("Invalid boolean hardware parameter '" + name + "': " + it->second);
}

K9RoboClawHardware::CallbackReturn K9RoboClawHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    left_joint_name_ = parameter_or<std::string>("left_joint", left_joint_name_);
    right_joint_name_ = parameter_or<std::string>("right_joint", right_joint_name_);
    device_ = parameter_or<std::string>("device", device_);
    baud_rate_ = parameter_or<uint32_t>("baud_rate", baud_rate_);
    address_ = parameter_or<uint8_t>("address", address_);
    io_timeout_ms_ = parameter_or<int>("io_timeout_ms", io_timeout_ms_);
    command_retries_ = parameter_or<int>("command_retries", command_retries_);
    debug_serial_ = parameter_bool("debug_serial", debug_serial_);
    dry_run_ = parameter_bool("dry_run", dry_run_);
    start_inhibited_ = parameter_bool("start_inhibited", start_inhibited_);
    reset_encoders_on_configure_ = parameter_bool(
      "reset_encoders_on_configure", reset_encoders_on_configure_);
    configure_main_voltage_limits_ = parameter_bool(
      "configure_main_voltage_limits", configure_main_voltage_limits_);
    configure_s3_estop_ = parameter_bool("configure_s3_estop", configure_s3_estop_);
    read_temperature_2_ = parameter_bool("read_temperature_2", read_temperature_2_);

    encoder_counts_per_revolution_ = parameter_or<double>(
      "encoder_counts_per_revolution", encoder_counts_per_revolution_);
    metres_per_encoder_count_ = parameter_or<double>(
      "metres_per_encoder_count", metres_per_encoder_count_);
    wheel_separation_ = parameter_or<double>("wheel_separation", wheel_separation_);
    operational_max_qpps_ = parameter_or<int32_t>("operational_max_qpps", operational_max_qpps_);

    acceleration_qpps_per_second_ = parameter_or<uint32_t>(
      "acceleration_qpps_per_second", acceleration_qpps_per_second_);
    deceleration_qpps_per_second_ = parameter_or<uint32_t>(
      "deceleration_qpps_per_second", deceleration_qpps_per_second_);
    emergency_deceleration_qpps_per_second_ = parameter_or<uint32_t>(
      "emergency_deceleration_qpps_per_second", emergency_deceleration_qpps_per_second_);
    max_seconds_uncommanded_travel_ = parameter_or<double>(
      "max_seconds_uncommanded_travel", max_seconds_uncommanded_travel_);
    serial_timeout_deciseconds_ = parameter_or<uint8_t>(
      "serial_timeout_deciseconds", serial_timeout_deciseconds_);

    m1_p_ = parameter_or<double>("m1_p", m1_p_);
    m1_i_ = parameter_or<double>("m1_i", m1_i_);
    m1_d_ = parameter_or<double>("m1_d", m1_d_);
    m1_pid_qpps_ = parameter_or<uint32_t>("m1_pid_qpps", m1_pid_qpps_);
    m2_p_ = parameter_or<double>("m2_p", m2_p_);
    m2_i_ = parameter_or<double>("m2_i", m2_i_);
    m2_d_ = parameter_or<double>("m2_d", m2_d_);
    m2_pid_qpps_ = parameter_or<uint32_t>("m2_pid_qpps", m2_pid_qpps_);

    main_voltage_min_tenths_ = parameter_or<uint16_t>(
      "main_voltage_min_tenths", main_voltage_min_tenths_);
    main_voltage_max_tenths_ = parameter_or<uint16_t>(
      "main_voltage_max_tenths", main_voltage_max_tenths_);
    s3_mode_ = parameter_or<uint8_t>("s3_mode", s3_mode_);
    s4_mode_ = parameter_or<uint8_t>("s4_mode", s4_mode_);
    s5_mode_ = parameter_or<uint8_t>("s5_mode", s5_mode_);
    status_poll_hz_ = parameter_or<double>("status_poll_hz", status_poll_hz_);
    diagnostics_publish_hz_ = parameter_or<double>(
      "diagnostics_publish_hz", diagnostics_publish_hz_);
    dry_run_main_voltage_ = parameter_or<double>("dry_run_main_voltage", dry_run_main_voltage_);
    dry_run_logic_voltage_ = parameter_or<double>("dry_run_logic_voltage", dry_run_logic_voltage_);

    // Preserve the proven 2021 distance calibration exactly. The nominal 139 mm
    // wheel diameter is documentation; metres/count is the authoritative scale.
    const double effective_circumference =
      encoder_counts_per_revolution_ * metres_per_encoder_count_;
    effective_wheel_radius_ = effective_circumference / kTwoPi;

    left_position_interface_ = left_joint_name_ + "/" + hardware_interface::HW_IF_POSITION;
    right_position_interface_ = right_joint_name_ + "/" + hardware_interface::HW_IF_POSITION;
    left_velocity_state_interface_ = left_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
    right_velocity_state_interface_ = right_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
    left_velocity_command_interface_ = left_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
    right_velocity_command_interface_ = right_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY;

    validate_configuration();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "K9 RoboClaw configuration invalid: %s", ex.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(),
    "K9 RoboClaw: %s, addr=0x%02x, dry_run=%s, wheel_radius=%.8f m, separation=%.4f m, "
    "200-count calibration=%.6f m/rev, operational limit=%d qpps",
    device_.c_str(), static_cast<unsigned>(address_), dry_run_ ? "true" : "false",
    effective_wheel_radius_, wheel_separation_,
    encoder_counts_per_revolution_ * metres_per_encoder_count_, operational_max_qpps_);

  return CallbackReturn::SUCCESS;
}

void K9RoboClawHardware::validate_configuration()
{
  if (left_joint_name_ == right_joint_name_) {
    throw std::invalid_argument("left_joint and right_joint must differ");
  }
  if (encoder_counts_per_revolution_ <= 0.0 || metres_per_encoder_count_ <= 0.0) {
    throw std::invalid_argument("encoder scale must be positive");
  }
  if (wheel_separation_ <= 0.0 || operational_max_qpps_ <= 0) {
    throw std::invalid_argument("wheel separation and operational speed limit must be positive");
  }
  if (acceleration_qpps_per_second_ == 0 ||
      deceleration_qpps_per_second_ < acceleration_qpps_per_second_ ||
      emergency_deceleration_qpps_per_second_ < deceleration_qpps_per_second_)
  {
    throw std::invalid_argument("expected acceleration <= deceleration <= emergency deceleration");
  }
  if (static_cast<uint32_t>(operational_max_qpps_) > std::min(m1_pid_qpps_, m2_pid_qpps_)) {
    throw std::invalid_argument("operational speed limit exceeds a RoboClaw PID QPPS calibration");
  }
  if (max_seconds_uncommanded_travel_ <= 0.0 || serial_timeout_deciseconds_ == 0) {
    throw std::invalid_argument("both hardware watchdog bounds must be non-zero");
  }
  if (status_poll_hz_ <= 0.0 || diagnostics_publish_hz_ <= 0.0) {
    throw std::invalid_argument("status/diagnostics rates must be positive");
  }
  if (configure_s3_estop_ && s3_mode_ != kS3ModeEStop) {
    throw std::invalid_argument(
      "K9 requires RoboClaw S3 mode 0x01 (non-latching E-Stop) on firmware v4.1.34");
  }

  const auto validate_wheel_joint = [&](const std::string & expected_name) {
      const auto it = std::find_if(
        info_.joints.begin(), info_.joints.end(),
        [&](const auto & joint) {return joint.name == expected_name;});
      if (it == info_.joints.end()) {
        throw std::invalid_argument(
                "wheel joint '" + expected_name + "' not found in ros2_control description");
      }
      if (it->command_interfaces.size() != 1 ||
        it->command_interfaces.front().name != hardware_interface::HW_IF_VELOCITY)
      {
        throw std::invalid_argument(
                "wheel joint '" + expected_name + "' must expose exactly one velocity command interface");
      }
      if (it->state_interfaces.size() != 2) {
        throw std::invalid_argument(
                "wheel joint '" + expected_name + "' must expose position and velocity state interfaces");
      }
      const bool has_position = std::any_of(
        it->state_interfaces.begin(), it->state_interfaces.end(),
        [](const auto & interface) {return interface.name == hardware_interface::HW_IF_POSITION;});
      const bool has_velocity = std::any_of(
        it->state_interfaces.begin(), it->state_interfaces.end(),
        [](const auto & interface) {return interface.name == hardware_interface::HW_IF_VELOCITY;});
      if (!has_position || !has_velocity) {
        throw std::invalid_argument(
                "wheel joint '" + expected_name + "' must expose position and velocity state interfaces");
      }
    };

  validate_wheel_joint(left_joint_name_);
  validate_wheel_joint(right_joint_name_);
}

void K9RoboClawHardware::create_ros_interfaces()
{
  if (motor_battery_pub_) {
    return;
  }
  auto node = get_node();
  if (!node) {
    throw std::runtime_error("ros2_control did not provide a framework-managed hardware node");
  }

  motor_battery_pub_ = node->create_publisher<sensor_msgs::msg::BatteryState>(
    "/k9/battery/motor", rclcpp::SensorDataQoS());
  logic_battery_pub_ = node->create_publisher<sensor_msgs::msg::BatteryState>(
    "/k9/battery/logic", rclcpp::SensorDataQoS());
  diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  estop_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "/k9/drive/estop", rclcpp::SystemDefaultsQoS());
  estop_latched_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "/k9/drive/estop_latched", rclcpp::SystemDefaultsQoS());
  enabled_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "/k9/drive/enabled", rclcpp::SystemDefaultsQoS());

  clear_estop_service_ = node->create_service<std_srvs::srv::Trigger>(
    "/k9/drive/clear_estop_latch",
    std::bind(
      &K9RoboClawHardware::clear_estop_callback, this,
      std::placeholders::_1, std::placeholders::_2));
  set_inhibit_service_ = node->create_service<std_srvs::srv::SetBool>(
    "/k9/drive/set_inhibit",
    std::bind(
      &K9RoboClawHardware::set_inhibit_callback, this,
      std::placeholders::_1, std::placeholders::_2));

}

K9RoboClawHardware::CallbackReturn K9RoboClawHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    create_ros_interfaces();
    active_.store(false);
    software_inhibit_.store(true);
    raw_estop_.store(false);
    estop_latched_.store(false);
    motion_rearm_required_.store(true);
    connection_fault_.store(false);
    invalid_command_fault_.store(false);
    hardware_fault_.store(false);
    accumulated_left_counts_.store(0);
    accumulated_right_counts_.store(0);
    previous_encoder_valid_ = false;
    have_last_command_ = false;
    last_left_qpps_ = 0;
    last_right_qpps_ = 0;
    commanded_m1_qpps_.store(0);
    commanded_m2_qpps_.store(0);

    set_state(left_position_interface_, 0.0);
    set_state(right_position_interface_, 0.0);
    set_state(left_velocity_state_interface_, 0.0);
    set_state(right_velocity_state_interface_, 0.0);
    set_command(left_velocity_command_interface_, 0.0);
    set_command(right_velocity_command_interface_, 0.0);

    if (dry_run_) {
      firmware_version_ = "K9 RoboClaw dry-run";
      main_battery_voltage_.store(dry_run_main_voltage_);
      logic_battery_voltage_.store(dry_run_logic_voltage_);
      roboclaw_status_.store(0);
      dry_run_left_velocity_rad_s_ = 0.0;
      dry_run_right_velocity_rad_s_ = 0.0;
      RCLCPP_WARN(get_logger(), "K9 RoboClaw configured in DRY-RUN mode: no serial device will be opened");
    } else {
      configure_real_hardware();
    }

    // Start telemetry publishing only after all cached configuration/firmware
    // fields are initialised, so the timer never races configuration writes.
    if (!diagnostics_timer_) {
      const auto timer_period = std::chrono::duration<double>(1.0 / diagnostics_publish_hz_);
      diagnostics_timer_ = get_node()->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&K9RoboClawHardware::publish_cached_status, this));
    }
  } catch (const std::exception & ex) {
    connection_fault_.store(true);
    safe_stop_noexcept("configure failure");
    RCLCPP_ERROR(get_logger(), "Failed to configure K9 RoboClaw: %s", ex.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

void K9RoboClawHardware::configure_real_hardware()
{
  roboclaw_ = std::make_unique<RoboClawTransport>(
    device_, baud_rate_, address_, io_timeout_ms_, command_retries_, debug_serial_);
  roboclaw_->open_port();

  // The first transmitted drive command after opening the link is always zero.
  roboclaw_->stop(emergency_deceleration_qpps_per_second_);
  firmware_version_ = roboclaw_->firmware_version();
  if (firmware_version_.find("v4.1.") == std::string::npos) {
    throw std::runtime_error(
      "This K9 RoboClaw driver is validated for the legacy firmware 4.1.x packet protocol; "
      "detected firmware: " + firmware_version_);
  }
  RCLCPP_INFO(get_logger(), "RoboClaw firmware 4.1.x detected; using K9-verified 32-bit command-90 status framing");

  // Hardware-level watchdog: independent of diff_drive_controller's ROS timeout.
  roboclaw_->set_serial_timeout(serial_timeout_deciseconds_);

  roboclaw_->set_velocity_pid(RoboClawTransport::Motor::M1, m1_p_, m1_i_, m1_d_, m1_pid_qpps_);
  roboclaw_->set_velocity_pid(RoboClawTransport::Motor::M2, m2_p_, m2_i_, m2_d_, m2_pid_qpps_);

  if (configure_main_voltage_limits_) {
    roboclaw_->set_main_voltages(main_voltage_min_tenths_, main_voltage_max_tenths_);
  }
  if (configure_s3_estop_) {
    // RoboClaw 2x15A firmware v4.1.34 uses 0x01 for the non-firmware-latching
    // E-stop (0x81 is firmware-latching). K9's key is mechanically latching;
    // the ROS safety latch additionally prevents stale motion resuming.
    roboclaw_->set_pin_functions(kS3ModeEStop, s4_mode_, s5_mode_);
    const auto pin_modes = roboclaw_->read_pin_functions();
    if (pin_modes[0] != kS3ModeEStop || pin_modes[1] != s4_mode_ || pin_modes[2] != s5_mode_) {
      throw std::runtime_error(
        "RoboClaw auxiliary pin configuration readback mismatch: requested S3/S4/S5=" +
        std::to_string(kS3ModeEStop) + "/" + std::to_string(s4_mode_) + "/" +
        std::to_string(s5_mode_) + ", read back " + std::to_string(pin_modes[0]) + "/" +
        std::to_string(pin_modes[1]) + "/" + std::to_string(pin_modes[2]));
    }
    RCLCPP_INFO(
      get_logger(), "Verified RoboClaw pin modes: S3=0x%02X (E-Stop), S4=0x%02X, S5=0x%02X",
      static_cast<unsigned>(pin_modes[0]), static_cast<unsigned>(pin_modes[1]),
      static_cast<unsigned>(pin_modes[2]));
  }

  if (reset_encoders_on_configure_) {
    // Match the proven 2021 controller exactly: reset both RoboClaw encoder
    // registers on genuine hardware startup/configuration. Runtime rollover is
    // still handled independently by the 64-bit software accumulator.
    roboclaw_->reset_encoders();
  }
  initialise_encoder_tracking();

  roboclaw_status_.store(roboclaw_->read_status());
  const bool estop = (roboclaw_status_.load() & kStatusEStop) != 0;
  raw_estop_.store(estop);
  estop_latched_.store(estop);
  if (estop) {
    motion_rearm_required_.store(true);
  }
  const bool initial_hardware_fault =
    (roboclaw_status_.load() & (kFaultMask & ~kStatusEStop)) != 0;
  hardware_fault_.store(initial_hardware_fault);
  if (initial_hardware_fault) {
    software_inhibit_.store(true);
    motion_rearm_required_.store(true);
  }
  main_battery_voltage_.store(roboclaw_->read_main_battery_voltage());
  logic_battery_voltage_.store(roboclaw_->read_logic_battery_voltage());
  roboclaw_->stop(emergency_deceleration_qpps_per_second_);

  RCLCPP_INFO(
    get_logger(), "Connected to RoboClaw 2x15A: %s; main=%.1f V, logic=%.1f V, status=%s",
    firmware_version_.c_str(), main_battery_voltage_.load(), logic_battery_voltage_.load(),
    hex32(roboclaw_status_.load()).c_str());
}

void K9RoboClawHardware::initialise_encoder_tracking()
{
  const auto left = roboclaw_->read_encoder(RoboClawTransport::Motor::M1);
  const auto right = roboclaw_->read_encoder(RoboClawTransport::Motor::M2);
  previous_left_raw_ = left.raw;
  previous_right_raw_ = right.raw;
  previous_encoder_valid_ = true;
  accumulated_left_counts_.store(0);
  accumulated_right_counts_.store(0);
}

K9RoboClawHardware::CallbackReturn K9RoboClawHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  safe_stop_noexcept("activation");
  set_command(left_velocity_command_interface_, 0.0);
  set_command(right_velocity_command_interface_, 0.0);
  motion_rearm_required_.store(true);
  software_inhibit_.store(start_inhibited_);
  active_.store(true);

  // Communication and non-E-stop hardware faults prevent a usable control loop,
  // so activation must fail for those conditions.  A physical E-stop is
  // different: K9 should be able to boot, publish diagnostics/encoders/battery
  // state, and run controller_manager while motion remains positively blocked.
  if (connection_fault_.load() || hardware_fault_.load()) {
    RCLCPP_ERROR(
      get_logger(),
      "Drive activation refused: connection_fault=%s hardware_fault=%s",
      bool_text(connection_fault_.load()).c_str(), bool_text(hardware_fault_.load()).c_str());
    active_.store(false);
    return CallbackReturn::FAILURE;
  }

  if (raw_estop_.load() || estop_latched_.load()) {
    RCLCPP_WARN(
      get_logger(),
      "Drive active with E-STOP ASSERTED/LATCHED: motion is inhibited. "
      "Release the physical key, then clear /k9/drive/clear_estop_latch; "
      "an explicit zero wheel command is required before motion can re-arm.");
  }

  if (start_inhibited_) {
    RCLCPP_WARN(
      get_logger(),
      "Drive active but SOFTWARE INHIBITED. Clear with: ros2 service call /k9/drive/set_inhibit "
      "std_srvs/srv/SetBool \"{data: false}\"");
  }
  return CallbackReturn::SUCCESS;
}

K9RoboClawHardware::CallbackReturn K9RoboClawHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  software_inhibit_.store(true);
  motion_rearm_required_.store(true);
  set_command(left_velocity_command_interface_, 0.0);
  set_command(right_velocity_command_interface_, 0.0);
  safe_stop_noexcept("deactivate");
  return CallbackReturn::SUCCESS;
}

K9RoboClawHardware::CallbackReturn K9RoboClawHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  software_inhibit_.store(true);
  safe_stop_noexcept("cleanup");
  if (diagnostics_timer_) {
    diagnostics_timer_->cancel();
    diagnostics_timer_.reset();
  }
  if (roboclaw_) {
    roboclaw_->close_port();
    roboclaw_.reset();
  }
  return CallbackReturn::SUCCESS;
}

K9RoboClawHardware::CallbackReturn K9RoboClawHardware::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  software_inhibit_.store(true);
  safe_stop_noexcept("shutdown");
  if (diagnostics_timer_) {
    diagnostics_timer_->cancel();
    diagnostics_timer_.reset();
  }
  if (roboclaw_) {
    roboclaw_->close_port();
  }
  return CallbackReturn::SUCCESS;
}

K9RoboClawHardware::CallbackReturn K9RoboClawHardware::on_error(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  software_inhibit_.store(true);
  motion_rearm_required_.store(true);
  safe_stop_noexcept("hardware error");
  if (roboclaw_) {
    // Closing the port ensures the RoboClaw serial watchdog is no longer being
    // refreshed after a ros2_control hardware error. The bounded-distance
    // command remains an independent final-motion limit.
    roboclaw_->close_port();
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type K9RoboClawHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  try {
    const double dt = period.seconds();
    if (!(dt > 0.0) || !std::isfinite(dt)) {
      return hardware_interface::return_type::OK;
    }

    if (dry_run_) {
      const double old_left = get_state<double>(left_position_interface_);
      const double old_right = get_state<double>(right_position_interface_);
      set_state(left_position_interface_, old_left + dry_run_left_velocity_rad_s_ * dt);
      set_state(right_position_interface_, old_right + dry_run_right_velocity_rad_s_ * dt);
      set_state(left_velocity_state_interface_, dry_run_left_velocity_rad_s_);
      set_state(right_velocity_state_interface_, dry_run_right_velocity_rad_s_);
      measured_m1_qpps_.store(radians_per_second_to_qpps(
        dry_run_left_velocity_rad_s_, encoder_counts_per_revolution_));
      measured_m2_qpps_.store(radians_per_second_to_qpps(
        dry_run_right_velocity_rad_s_, encoder_counts_per_revolution_));
    } else {
      const auto left = roboclaw_->read_encoder(RoboClawTransport::Motor::M1);
      const auto right = roboclaw_->read_encoder(RoboClawTransport::Motor::M2);

      if (!previous_encoder_valid_) {
        previous_left_raw_ = left.raw;
        previous_right_raw_ = right.raw;
        previous_encoder_valid_ = true;
      } else {
        const int32_t left_delta = rollover_safe_delta(left.raw, previous_left_raw_);
        const int32_t right_delta = rollover_safe_delta(right.raw, previous_right_raw_);
        previous_left_raw_ = left.raw;
        previous_right_raw_ = right.raw;
        accumulated_left_counts_.fetch_add(static_cast<int64_t>(left_delta));
        accumulated_right_counts_.fetch_add(static_cast<int64_t>(right_delta));
        left_velocity_rad_s_ = counts_to_radians(
          static_cast<int64_t>(left_delta), encoder_counts_per_revolution_) / dt;
        right_velocity_rad_s_ = counts_to_radians(
          static_cast<int64_t>(right_delta), encoder_counts_per_revolution_) / dt;
      }

      set_state(
        left_position_interface_,
        counts_to_radians(accumulated_left_counts_.load(), encoder_counts_per_revolution_));
      set_state(
        right_position_interface_,
        counts_to_radians(accumulated_right_counts_.load(), encoder_counts_per_revolution_));
      set_state(left_velocity_state_interface_, left_velocity_rad_s_);
      set_state(right_velocity_state_interface_, right_velocity_rad_s_);

      const auto now = std::chrono::steady_clock::now();
      const double poll_period = 1.0 / status_poll_hz_;
      if (last_status_poll_.time_since_epoch().count() == 0 ||
          std::chrono::duration<double>(now - last_status_poll_).count() >= poll_period)
      {
        update_status_cache();
        last_status_poll_ = now;
      }
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception & ex) {
    connection_fault_.store(true);
    software_inhibit_.store(true);
    motion_rearm_required_.store(true);
    safe_stop_noexcept("read failure");
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "RoboClaw read failure: %s", ex.what());
    return hardware_interface::return_type::ERROR;
  }
}

void K9RoboClawHardware::update_status_cache()
{
  const uint32_t status = roboclaw_->read_status();
  roboclaw_status_.store(status);
  const bool estop = (status & kStatusEStop) != 0;
  raw_estop_.store(estop);
  if (estop) {
    estop_latched_.store(true);
    motion_rearm_required_.store(true);
  }
  const bool non_estop_fault = (status & (kFaultMask & ~kStatusEStop)) != 0;
  hardware_fault_.store(non_estop_fault);
  if (non_estop_fault) {
    software_inhibit_.store(true);
    motion_rearm_required_.store(true);
  }

  // Intentionally spread slower RoboClaw queries over several status ticks.
  switch (secondary_status_index_++ % 4) {
    case 0:
      main_battery_voltage_.store(roboclaw_->read_main_battery_voltage());
      logic_battery_voltage_.store(roboclaw_->read_logic_battery_voltage());
      break;
    case 1: {
      const auto current = roboclaw_->read_motor_currents();
      m1_current_amp_.store(current.m1_amp);
      m2_current_amp_.store(current.m2_amp);
      break;
    }
    case 2:
      temperature_1_c_.store(roboclaw_->read_temperature(false));
      if (read_temperature_2_) {
        temperature_2_c_.store(roboclaw_->read_temperature(true));
      }
      break;
    case 3:
      measured_m1_qpps_.store(roboclaw_->read_speed(RoboClawTransport::Motor::M1));
      measured_m2_qpps_.store(roboclaw_->read_speed(RoboClawTransport::Motor::M2));
      break;
    default:
      break;
  }
}

hardware_interface::return_type K9RoboClawHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    const double left_command = get_command<double>(left_velocity_command_interface_);
    const double right_command = get_command<double>(right_velocity_command_interface_);

    if (!std::isfinite(left_command) || !std::isfinite(right_command)) {
      invalid_command_fault_.store(true);
      software_inhibit_.store(true);
      motion_rearm_required_.store(true);
      safe_stop_noexcept("non-finite wheel command");
      RCLCPP_ERROR(get_logger(), "Rejected non-finite wheel velocity command");
      return hardware_interface::return_type::ERROR;
    }

    // After activation, clearing an inhibit, or releasing/clearing an E-stop,
    // at least one explicit zero command must be observed before motion resumes.
    bool blocked = !active_.load() || software_inhibit_.load() || raw_estop_.load() ||
      estop_latched_.load() || connection_fault_.load() || invalid_command_fault_.load() ||
      hardware_fault_.load();
    bool just_rearmed = false;
    if (!blocked && motion_rearm_required_.load()) {
      if (std::abs(left_command) <= kNearZero && std::abs(right_command) <= kNearZero) {
        motion_rearm_required_.store(false);
        just_rearmed = true;
      } else {
        blocked = true;
      }
    }

    double requested_left_qpps = blocked || just_rearmed ? 0.0 :
      static_cast<double>(radians_per_second_to_qpps(left_command, encoder_counts_per_revolution_));
    double requested_right_qpps = blocked || just_rearmed ? 0.0 :
      static_cast<double>(radians_per_second_to_qpps(right_command, encoder_counts_per_revolution_));

    // Preserve the original K9 turn-speed policy. The 2021 controller reduced
    // maximum wheel speed as the centreline turning radius tightened, reaching
    // 10% of walking speed for an on-the-spot turn. Infer the same radius from
    // the wheel commands supplied by diff_drive_controller.
    double turn_modifier = 1.0;
    if (!blocked && !just_rearmed) {
      const double left_linear = left_command * effective_wheel_radius_;
      const double right_linear = right_command * effective_wheel_radius_;
      const double body_linear = 0.5 * (left_linear + right_linear);
      const double body_angular = (right_linear - left_linear) / wheel_separation_;
      if (std::abs(body_angular) > kNearZero) {
        const double centreline_radius = std::abs(body_linear / body_angular);
        turn_modifier = original_k9_turn_modifier(centreline_radius);
      }
    }
    applied_turn_modifier_.store(turn_modifier);

    // Hard physical ceiling inside the hardware layer. Scale the pair together
    // so a bad upstream command cannot exceed K9's radius-dependent limit and
    // the requested curvature is preserved.
    const double wheel_ceiling = static_cast<double>(operational_max_qpps_) * turn_modifier;
    const double largest = std::max(std::abs(requested_left_qpps), std::abs(requested_right_qpps));
    if (largest > wheel_ceiling && largest > 0.0) {
      const double scale = wheel_ceiling / largest;
      requested_left_qpps *= scale;
      requested_right_qpps *= scale;
    }

    const int32_t left_qpps = static_cast<int32_t>(std::lround(requested_left_qpps));
    const int32_t right_qpps = static_cast<int32_t>(std::lround(requested_right_qpps));

    uint32_t selected_accel = acceleration_qpps_per_second_;
    if (left_qpps == 0 && right_qpps == 0) {
      selected_accel = emergency_deceleration_qpps_per_second_;
    } else if (have_last_command_ &&
      (is_slowing(last_left_qpps_, left_qpps) || is_slowing(last_right_qpps_, right_qpps)))
    {
      selected_accel = deceleration_qpps_per_second_;
    }
    selected_acceleration_.store(selected_accel);

    const uint32_t left_distance = bounded_travel_counts(left_qpps, max_seconds_uncommanded_travel_);
    const uint32_t right_distance = bounded_travel_counts(right_qpps, max_seconds_uncommanded_travel_);

    if (dry_run_) {
      dry_run_left_velocity_rad_s_ = qpps_to_radians_per_second(
        left_qpps, encoder_counts_per_revolution_);
      dry_run_right_velocity_rad_s_ = qpps_to_radians_per_second(
        right_qpps, encoder_counts_per_revolution_);
    } else if (left_qpps == 0 && right_qpps == 0) {
      // Use RoboClaw's speed+acceleration command for the explicit zero path.
      // This makes stopping independent of distance-command zero semantics.
      roboclaw_->drive_speed_accel(selected_accel, 0, 0);
    } else {
      roboclaw_->drive_speed_accel_distance(
        selected_accel, left_qpps, left_distance, right_qpps, right_distance, true);
    }

    commanded_m1_qpps_.store(left_qpps);
    commanded_m2_qpps_.store(right_qpps);
    last_left_qpps_ = left_qpps;
    last_right_qpps_ = right_qpps;
    have_last_command_ = true;
    return hardware_interface::return_type::OK;
  } catch (const std::exception & ex) {
    connection_fault_.store(true);
    software_inhibit_.store(true);
    motion_rearm_required_.store(true);
    safe_stop_noexcept("write failure");
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "RoboClaw write failure: %s", ex.what());
    return hardware_interface::return_type::ERROR;
  }
}

void K9RoboClawHardware::safe_stop_noexcept(const char *) noexcept
{
  commanded_m1_qpps_.store(0);
  commanded_m2_qpps_.store(0);
  dry_run_left_velocity_rad_s_ = 0.0;
  dry_run_right_velocity_rad_s_ = 0.0;
  last_left_qpps_ = 0;
  last_right_qpps_ = 0;
  have_last_command_ = true;
  if (dry_run_ || !roboclaw_ || !roboclaw_->is_open()) {
    return;
  }
  try {
    roboclaw_->stop(emergency_deceleration_qpps_per_second_);
  } catch (...) {
    // Safety cleanup must not throw from lifecycle/destructor paths.
  }
}

void K9RoboClawHardware::clear_estop_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (raw_estop_.load()) {
    response->success = false;
    response->message = "Physical RoboClaw E-stop is still asserted; release the latching key first";
    return;
  }
  if (connection_fault_.load()) {
    response->success = false;
    response->message = "Cannot clear E-stop latch while a RoboClaw communication fault is active";
    return;
  }
  estop_latched_.store(false);
  motion_rearm_required_.store(true);
  // Do not access the serial port from a ROS service callback; read()/write()
  // are its sole runtime owners. The next 30 Hz write cycle explicitly sends zero.
  response->success = true;
  response->message = "Software E-stop latch cleared; zero wheel command required before motion can re-arm";
}

void K9RoboClawHardware::set_inhibit_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!request->data && (raw_estop_.load() || estop_latched_.load() || connection_fault_.load() || hardware_fault_.load())) {
    response->success = false;
    response->message = "Cannot release software inhibit while E-stop/fault is active";
    return;
  }
  software_inhibit_.store(request->data);
  motion_rearm_required_.store(true);
  // The control-loop write() observes the atomic inhibit on its next cycle and
  // sends the emergency-decelerated zero command. Keep serial I/O out of this
  // executor callback to prevent concurrent packet transactions.
  response->success = true;
  response->message = request->data ?
    "Drive software inhibit asserted" :
    "Drive software inhibit released; zero wheel command required before motion can re-arm";
}

int K9RoboClawHardware::diagnostic_level(uint32_t status) const
{
  if (connection_fault_.load() || invalid_command_fault_.load() || hardware_fault_.load() ||
      estop_latched_.load() || (status & kFaultMask) != 0)
  {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  if ((status & kWarningMask) != 0 || software_inhibit_.load() || motion_rearm_required_.load()) {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }
  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

std::string K9RoboClawHardware::decode_status(uint32_t status) const
{
  if (status == 0) {
    return "normal";
  }
  std::vector<std::string> labels;
  const auto add = [&](uint32_t mask, const char * text) {
    if ((status & mask) != 0) labels.emplace_back(text);
  };
  add(kStatusEStop, "E-stop");
  add(kStatusTemperature1, "temperature-error");
  add(kStatusTemperature2, "temperature-2-error");
  add(kStatusLogicBatteryHigh, "logic-battery-high-error");
  add(kStatusLogicBatteryLow, "logic-battery-low-error");
  add(kStatusM1DriverFault, "M1-driver-fault");
  add(kStatusM2DriverFault, "M2-driver-fault");
  add(kStatusM1Speed, "M1-speed-error");
  add(kStatusM2Speed, "M2-speed-error");
  add(kStatusM1Position, "M1-position-error");
  add(kStatusM2Position, "M2-position-error");
  add(kStatusM1Current, "M1-current-error");
  add(kStatusM2Current, "M2-current-error");
  add(kStatusM1OverCurrentWarning, "M1-over-current-warning");
  add(kStatusM2OverCurrentWarning, "M2-over-current-warning");
  add(kStatusMainBatteryHighWarning, "main-battery-high-warning");
  add(kStatusMainBatteryLowWarning, "main-battery-low-warning");
  add(kStatusTemperatureWarning, "temperature-warning");
  add(kStatusTemperature2Warning, "temperature-2-warning");
  add(kStatusS4Triggered, "S4-warning");
  add(kStatusS5Triggered, "S5-warning");
  add(kStatusCanWarning, "CAN-warning");
  add(kStatusBootWarning, "boot-warning");
  add(kStatusM1OverRegenWarning, "M1-over-regen-warning");
  add(kStatusM2OverRegenWarning, "M2-over-regen-warning");

  std::ostringstream stream;
  for (std::size_t i = 0; i < labels.size(); ++i) {
    if (i) stream << ", ";
    stream << labels[i];
  }
  return stream.str();
}

void K9RoboClawHardware::publish_cached_status()
{
  if (!motor_battery_pub_) {
    return;
  }
  const auto stamp = get_clock()->now();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  sensor_msgs::msg::BatteryState motor;
  motor.header.stamp = stamp;
  motor.header.frame_id = "base_link";
  motor.voltage = static_cast<float>(main_battery_voltage_.load());
  motor.temperature = static_cast<float>(nan);
  motor.current = static_cast<float>(nan);
  motor.charge = static_cast<float>(nan);
  motor.capacity = static_cast<float>(nan);
  motor.design_capacity = static_cast<float>(nan);
  motor.percentage = static_cast<float>(nan);
  motor.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  motor.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  motor.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  motor.present = true;
  motor.location = "K9 24 V motor battery / RoboClaw main supply";
  motor.serial_number = "";
  motor_battery_pub_->publish(motor);

  sensor_msgs::msg::BatteryState logic = motor;
  logic.voltage = static_cast<float>(logic_battery_voltage_.load());
  logic.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
  logic.location = "RELiON 12.8 V LiFePO4 / RoboClaw logic supply";
  logic_battery_pub_->publish(logic);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus drive;
  const uint32_t status = roboclaw_status_.load();
  drive.level = static_cast<uint8_t>(diagnostic_level(status));
  drive.name = "K9/RoboClaw 2x15A";
  drive.hardware_id = dry_run_ ? "dry-run" : device_;
  drive.message = connection_fault_.load() ? "RoboClaw communication fault" :
    (estop_latched_.load() ? "E-stop latched" :
    (hardware_fault_.load() ? decode_status(status) :
    (invalid_command_fault_.load() ? "invalid wheel command latched" :
    (software_inhibit_.load() ? "software inhibited" : decode_status(status)))));
  drive.values.push_back(kv("firmware", firmware_version_));
  drive.values.push_back(kv("dry_run", bool_text(dry_run_)));
  drive.values.push_back(kv("active", bool_text(active_.load())));
  drive.values.push_back(kv("software_inhibit", bool_text(software_inhibit_.load())));
  drive.values.push_back(kv("raw_estop", bool_text(raw_estop_.load())));
  drive.values.push_back(kv("estop_latched", bool_text(estop_latched_.load())));
  drive.values.push_back(kv("motion_rearm_required", bool_text(motion_rearm_required_.load())));
  drive.values.push_back(kv("connection_fault", bool_text(connection_fault_.load())));
  drive.values.push_back(kv("hardware_fault", bool_text(hardware_fault_.load())));
  drive.values.push_back(kv("status_register", hex32(status)));
  drive.values.push_back(kv("status_text", decode_status(status)));
  drive.values.push_back(kv("motor_battery_V", number_text(main_battery_voltage_.load(), 2)));
  drive.values.push_back(kv("logic_battery_V", number_text(logic_battery_voltage_.load(), 2)));
  drive.values.push_back(kv("M1_current_A", number_text(m1_current_amp_.load(), 2)));
  drive.values.push_back(kv("M2_current_A", number_text(m2_current_amp_.load(), 2)));
  drive.values.push_back(kv("temperature_1_C", number_text(temperature_1_c_.load(), 1)));
  if (read_temperature_2_) {
    drive.values.push_back(kv("temperature_2_C", number_text(temperature_2_c_.load(), 1)));
  }
  drive.values.push_back(kv("M1_measured_qpps", std::to_string(measured_m1_qpps_.load())));
  drive.values.push_back(kv("M2_measured_qpps", std::to_string(measured_m2_qpps_.load())));
  drive.values.push_back(kv("M1_commanded_qpps", std::to_string(commanded_m1_qpps_.load())));
  drive.values.push_back(kv("M2_commanded_qpps", std::to_string(commanded_m2_qpps_.load())));
  drive.values.push_back(kv("selected_accel_qpps_s", std::to_string(selected_acceleration_.load())));
  drive.values.push_back(kv("original_turn_modifier", number_text(applied_turn_modifier_.load(), 4)));
  drive.values.push_back(kv("left_accumulated_counts", std::to_string(accumulated_left_counts_.load())));
  drive.values.push_back(kv("right_accumulated_counts", std::to_string(accumulated_right_counts_.load())));
  drive.values.push_back(kv("encoder_counts_per_revolution", number_text(encoder_counts_per_revolution_, 0)));
  drive.values.push_back(kv("metres_per_encoder_count", number_text(metres_per_encoder_count_, 6)));
  drive.values.push_back(kv("operational_max_qpps", std::to_string(operational_max_qpps_)));
  array.status.push_back(drive);
  diagnostics_pub_->publish(array);

  std_msgs::msg::Bool value;
  value.data = raw_estop_.load();
  estop_pub_->publish(value);
  value.data = estop_latched_.load();
  estop_latched_pub_->publish(value);
  value.data = active_.load() && !software_inhibit_.load() && !raw_estop_.load() &&
    !estop_latched_.load() && !motion_rearm_required_.load() && !connection_fault_.load() &&
    !hardware_fault_.load();
  enabled_pub_->publish(value);
}

}  // namespace k9_drive_pkg

PLUGINLIB_EXPORT_CLASS(k9_drive_pkg::K9RoboClawHardware, hardware_interface::SystemInterface)
