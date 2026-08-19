#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace k9_drive_pkg
{

class RoboClawTransport
{
public:
  enum class Motor : uint8_t { M1 = 0, M2 = 1 };

  struct EncoderReading
  {
    uint32_t raw{0};
    uint8_t status{0};
  };

  struct MotorCurrents
  {
    double m1_amp{0.0};
    double m2_amp{0.0};
  };

  RoboClawTransport(
    std::string device, uint32_t baud_rate, uint8_t address,
    int io_timeout_ms = 50, int retries = 3, bool debug = false);
  ~RoboClawTransport();

  RoboClawTransport(const RoboClawTransport &) = delete;
  RoboClawTransport & operator=(const RoboClawTransport &) = delete;

  void open_port();
  void close_port() noexcept;
  bool is_open() const noexcept {return fd_ >= 0;}

  std::string firmware_version();
  void set_velocity_pid(Motor motor, double p, double i, double d, uint32_t qpps);
  void reset_encoders();
  void set_encoder(Motor motor, int32_t value);
  EncoderReading read_encoder(Motor motor);
  int32_t read_speed(Motor motor);

  void set_serial_timeout(uint8_t deciseconds);
  void set_main_voltages(uint16_t minimum_tenths, uint16_t maximum_tenths);
  void set_pin_functions(uint8_t s3_mode, uint8_t s4_mode, uint8_t s5_mode);
  std::array<uint8_t, 3> read_pin_functions();

  double read_main_battery_voltage();
  double read_logic_battery_voltage();
  MotorCurrents read_motor_currents();
  double read_temperature(bool second_sensor = false);
  uint32_t read_status();

  void drive_speed_accel(
    uint32_t acceleration_qpps_per_second,
    int32_t m1_speed_qpps, int32_t m2_speed_qpps);
  void drive_speed_accel_distance(
    uint32_t acceleration_qpps_per_second,
    int32_t m1_speed_qpps, uint32_t m1_max_distance_counts,
    int32_t m2_speed_qpps, uint32_t m2_max_distance_counts,
    bool cancel_previous = true);
  void stop(uint32_t emergency_deceleration_qpps_per_second);

private:
  static constexpr uint8_t CMD_SET_SERIAL_TIMEOUT = 14;
  static constexpr uint8_t CMD_GET_M1_ENCODER = 16;
  static constexpr uint8_t CMD_GET_M2_ENCODER = 17;
  static constexpr uint8_t CMD_GET_M1_SPEED = 18;
  static constexpr uint8_t CMD_GET_M2_SPEED = 19;
  static constexpr uint8_t CMD_RESET_ENCODERS = 20;
  static constexpr uint8_t CMD_GET_VERSION = 21;
  static constexpr uint8_t CMD_SET_M1_ENCODER = 22;
  static constexpr uint8_t CMD_SET_M2_ENCODER = 23;
  static constexpr uint8_t CMD_GET_MAIN_BATTERY = 24;
  static constexpr uint8_t CMD_GET_LOGIC_BATTERY = 25;
  static constexpr uint8_t CMD_SET_M1_PID = 28;
  static constexpr uint8_t CMD_SET_M2_PID = 29;
  static constexpr uint8_t CMD_MIXED_SPEED_ACCEL = 40;
  static constexpr uint8_t CMD_MIXED_SPEED_ACCEL_DISTANCE = 46;
  static constexpr uint8_t CMD_GET_CURRENTS = 49;
  static constexpr uint8_t CMD_SET_MAIN_VOLTAGES = 57;
  static constexpr uint8_t CMD_SET_PIN_FUNCTIONS = 74;
  static constexpr uint8_t CMD_GET_PIN_FUNCTIONS = 75;
  static constexpr uint8_t CMD_GET_TEMPERATURE = 82;
  static constexpr uint8_t CMD_GET_TEMPERATURE_2 = 83;
  static constexpr uint8_t CMD_GET_STATUS = 90;

  static uint16_t crc_update(uint16_t crc, uint8_t data);
  static void append_u16(std::vector<uint8_t> & bytes, uint16_t value);
  static void append_u32(std::vector<uint8_t> & bytes, uint32_t value);
  static uint16_t decode_u16(const std::vector<uint8_t> & bytes, size_t offset);
  static uint32_t decode_u32(const std::vector<uint8_t> & bytes, size_t offset);

  void configure_termios();
  void flush_input() noexcept;
  void write_all(const uint8_t * data, size_t length);
  uint8_t read_byte();
  std::vector<uint8_t> read_exact(size_t length);

  void write_command_ack(uint8_t command, const std::vector<uint8_t> & payload);
  std::vector<uint8_t> read_command(uint8_t command, size_t data_length);
  std::vector<uint8_t> transact_read_once(uint8_t command, size_t data_length);
  void transact_write_once(uint8_t command, const std::vector<uint8_t> & payload);

  std::string device_;
  uint32_t baud_rate_;
  uint8_t address_;
  int io_timeout_ms_;
  int retries_;
  bool debug_;
  int fd_{-1};
};

}  // namespace k9_drive_pkg
