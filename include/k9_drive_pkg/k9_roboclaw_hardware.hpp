#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "k9_drive_pkg/roboclaw_transport.hpp"

namespace k9_drive_pkg
{

class K9RoboClawHardware : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = hardware_interface::CallbackReturn;

  K9RoboClawHardware() = default;
  ~K9RoboClawHardware() override;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  template<typename T>
  T parameter_or(const std::string & name, const T & default_value) const;

  bool parameter_bool(const std::string & name, bool default_value) const;
  void validate_configuration();
  void create_ros_interfaces();
  void configure_real_hardware();
  void initialise_encoder_tracking();
  void safe_stop_noexcept(const char * reason) noexcept;
  void update_status_cache();
  void publish_cached_status();
  std::string decode_status(uint32_t status) const;
  int diagnostic_level(uint32_t status) const;

  void clear_estop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void set_inhibit_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // ROS2 control interface names.
  std::string left_joint_name_{"left_wheel_joint"};
  std::string right_joint_name_{"right_wheel_joint"};
  std::string left_position_interface_;
  std::string right_position_interface_;
  std::string left_velocity_state_interface_;
  std::string right_velocity_state_interface_;
  std::string left_velocity_command_interface_;
  std::string right_velocity_command_interface_;

  // RoboClaw transport/configuration.
  std::string device_{"/dev/roboclaw"};
  uint32_t baud_rate_{115200};
  uint8_t address_{0x80};
  int io_timeout_ms_{50};
  int command_retries_{3};
  bool debug_serial_{false};
  bool dry_run_{false};
  bool start_inhibited_{true};
  bool reset_encoders_on_configure_{true};
  bool configure_main_voltage_limits_{true};
  bool configure_s3_estop_{true};
  bool read_temperature_2_{false};

  // Source-of-truth K9 geometry/calibration from the proven 2021 controller.
  double encoder_counts_per_revolution_{200.0};
  double metres_per_encoder_count_{0.002179};
  double effective_wheel_radius_{0.0693597242};
  double wheel_separation_{0.2022};
  int32_t operational_max_qpps_{642};

  // Original acceleration with later ROS2 deceleration ratios.
  uint32_t acceleration_qpps_per_second_{128};
  uint32_t deceleration_qpps_per_second_{256};
  uint32_t emergency_deceleration_qpps_per_second_{512};
  double max_seconds_uncommanded_travel_{0.1};
  uint8_t serial_timeout_deciseconds_{2};

  // Proven RoboClaw velocity-loop values.
  double m1_p_{10.644};
  double m1_i_{2.206};
  double m1_d_{0.0};
  uint32_t m1_pid_qpps_{1987};
  double m2_p_{9.768};
  double m2_i_{2.294};
  double m2_d_{0.0};
  uint32_t m2_pid_qpps_{1837};

  // Known-working 24 V main-battery protection from the 2021 controller.
  uint16_t main_voltage_min_tenths_{240};
  uint16_t main_voltage_max_tenths_{292};
  uint8_t s3_mode_{0x01};  // RoboClaw 2x15A v4.1.34: E-Stop, non-firmware-latching.
  uint8_t s4_mode_{0};
  uint8_t s5_mode_{0};

  double status_poll_hz_{10.0};
  double diagnostics_publish_hz_{2.0};
  double dry_run_main_voltage_{25.6};
  double dry_run_logic_voltage_{13.2};

  std::unique_ptr<RoboClawTransport> roboclaw_;
  std::string firmware_version_{"dry-run"};

  // Continuous wheel position. RoboClaw counters are 32-bit and inevitably wrap;
  // delta calculation is modulo-safe while the accumulated count is 64-bit.
  uint32_t previous_left_raw_{0};
  uint32_t previous_right_raw_{0};
  bool previous_encoder_valid_{false};
  std::atomic<int64_t> accumulated_left_counts_{0};
  std::atomic<int64_t> accumulated_right_counts_{0};
  double left_velocity_rad_s_{0.0};
  double right_velocity_rad_s_{0.0};

  // Last command is retained only to choose accel vs decel. Commands are still
  // sent every control cycle so both RoboClaw safety mechanisms are renewed.
  int32_t last_left_qpps_{0};
  int32_t last_right_qpps_{0};
  bool have_last_command_{false};

  std::atomic<bool> active_{false};
  std::atomic<bool> software_inhibit_{true};
  std::atomic<bool> raw_estop_{false};
  std::atomic<bool> estop_latched_{false};
  std::atomic<bool> motion_rearm_required_{true};
  std::atomic<bool> connection_fault_{false};
  std::atomic<bool> invalid_command_fault_{false};
  std::atomic<bool> hardware_fault_{false};

  // Dry-run effective command after all hardware-layer safety processing.
  double dry_run_left_velocity_rad_s_{0.0};
  double dry_run_right_velocity_rad_s_{0.0};

  std::chrono::steady_clock::time_point last_status_poll_{};
  uint8_t secondary_status_index_{0};

  // Cached diagnostic values. Timer callbacks never access the serial port.
  std::atomic<double> main_battery_voltage_{0.0};
  std::atomic<double> logic_battery_voltage_{0.0};
  std::atomic<double> m1_current_amp_{0.0};
  std::atomic<double> m2_current_amp_{0.0};
  std::atomic<double> temperature_1_c_{0.0};
  std::atomic<double> temperature_2_c_{0.0};
  std::atomic<uint32_t> roboclaw_status_{0};
  std::atomic<int32_t> measured_m1_qpps_{0};
  std::atomic<int32_t> measured_m2_qpps_{0};
  std::atomic<int32_t> commanded_m1_qpps_{0};
  std::atomic<int32_t> commanded_m2_qpps_{0};
  std::atomic<uint32_t> selected_acceleration_{0};
  std::atomic<double> applied_turn_modifier_{1.0};

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr motor_battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr logic_battery_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_latched_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_estop_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_inhibit_service_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace k9_drive_pkg
