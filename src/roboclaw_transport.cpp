#include "k9_drive_pkg/roboclaw_transport.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace k9_drive_pkg
{
namespace
{
std::runtime_error io_error(const std::string & where)
{
  return std::runtime_error(where + ": " + std::strerror(errno));
}
}  // namespace

RoboClawTransport::RoboClawTransport(
  std::string device, uint32_t baud_rate, uint8_t address,
  int io_timeout_ms, int retries, bool debug)
: device_(std::move(device)),
  baud_rate_(baud_rate),
  address_(address),
  io_timeout_ms_(io_timeout_ms),
  retries_(retries),
  debug_(debug)
{
}

RoboClawTransport::~RoboClawTransport()
{
  close_port();
}

void RoboClawTransport::open_port()
{
  if (is_open()) {
    return;
  }

  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    throw io_error("Unable to open RoboClaw device " + device_);
  }

  try {
    configure_termios();
    flush_input();
  } catch (...) {
    close_port();
    throw;
  }
}

void RoboClawTransport::close_port() noexcept
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void RoboClawTransport::configure_termios()
{
  termios options{};
  if (tcgetattr(fd_, &options) < 0) {
    throw io_error("tcgetattr failed");
  }

  cfmakeraw(&options);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  speed_t baud{};
  switch (baud_rate_) {
    case 9600: baud = B9600; break;
    case 19200: baud = B19200; break;
    case 38400: baud = B38400; break;
    case 57600: baud = B57600; break;
    case 115200: baud = B115200; break;
#ifdef B230400
    case 230400: baud = B230400; break;
#endif
    default:
      throw std::runtime_error("Unsupported RoboClaw baud rate: " + std::to_string(baud_rate_));
  }

  if (cfsetispeed(&options, baud) < 0 || cfsetospeed(&options, baud) < 0) {
    throw io_error("Unable to set serial baud rate");
  }
  if (tcsetattr(fd_, TCSANOW, &options) < 0) {
    throw io_error("tcsetattr failed");
  }
}

void RoboClawTransport::flush_input() noexcept
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIFLUSH);
  }
}

uint16_t RoboClawTransport::crc_update(uint16_t crc, uint8_t data)
{
  crc ^= static_cast<uint16_t>(data) << 8;
  for (int i = 0; i < 8; ++i) {
    crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                          : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

void RoboClawTransport::append_u16(std::vector<uint8_t> & bytes, uint16_t value)
{
  bytes.push_back(static_cast<uint8_t>((value >> 8) & 0xffU));
  bytes.push_back(static_cast<uint8_t>(value & 0xffU));
}

void RoboClawTransport::append_u32(std::vector<uint8_t> & bytes, uint32_t value)
{
  bytes.push_back(static_cast<uint8_t>((value >> 24) & 0xffU));
  bytes.push_back(static_cast<uint8_t>((value >> 16) & 0xffU));
  bytes.push_back(static_cast<uint8_t>((value >> 8) & 0xffU));
  bytes.push_back(static_cast<uint8_t>(value & 0xffU));
}

uint16_t RoboClawTransport::decode_u16(const std::vector<uint8_t> & bytes, size_t offset)
{
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(bytes.at(offset)) << 8) |
    static_cast<uint16_t>(bytes.at(offset + 1)));
}

uint32_t RoboClawTransport::decode_u32(const std::vector<uint8_t> & bytes, size_t offset)
{
  return
    (static_cast<uint32_t>(bytes.at(offset)) << 24) |
    (static_cast<uint32_t>(bytes.at(offset + 1)) << 16) |
    (static_cast<uint32_t>(bytes.at(offset + 2)) << 8) |
    static_cast<uint32_t>(bytes.at(offset + 3));
}

void RoboClawTransport::write_all(const uint8_t * data, size_t length)
{
  size_t done = 0;
  while (done < length) {
    const ssize_t written = ::write(fd_, data + done, length - done);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw io_error("RoboClaw serial write failed");
    }
    if (written == 0) {
      throw std::runtime_error("RoboClaw serial write returned zero bytes");
    }
    done += static_cast<size_t>(written);
  }

  if (debug_) {
    std::cerr << "RoboClaw TX:";
    for (size_t i = 0; i < length; ++i) {
      std::cerr << ' ' << std::hex << static_cast<int>(data[i]);
    }
    std::cerr << std::dec << '\n';
  }
}

uint8_t RoboClawTransport::read_byte()
{
  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;

  while (true) {
    const int result = ::poll(&pfd, 1, io_timeout_ms_);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw io_error("RoboClaw serial poll failed");
    }
    if (result == 0) {
      throw std::runtime_error("RoboClaw serial read timeout");
    }
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw std::runtime_error("RoboClaw serial device reported an I/O error");
    }
    if ((pfd.revents & POLLIN) != 0) {
      uint8_t value = 0;
      const ssize_t n = ::read(fd_, &value, 1);
      if (n == 1) {
        if (debug_) {
          std::cerr << "RoboClaw RX: " << std::hex << static_cast<int>(value) << std::dec << '\n';
        }
        return value;
      }
      if (n < 0 && errno == EINTR) {
        continue;
      }
      throw io_error("RoboClaw serial read failed");
    }
  }
}

std::vector<uint8_t> RoboClawTransport::read_exact(size_t length)
{
  std::vector<uint8_t> result;
  result.reserve(length);
  while (result.size() < length) {
    result.push_back(read_byte());
  }
  return result;
}

void RoboClawTransport::transact_write_once(
  uint8_t command, const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> packet;
  packet.reserve(2 + payload.size() + 2);
  packet.push_back(address_);
  packet.push_back(command);
  packet.insert(packet.end(), payload.begin(), payload.end());

  uint16_t crc = 0;
  for (const uint8_t b : packet) {
    crc = crc_update(crc, b);
  }
  append_u16(packet, crc);

  flush_input();
  write_all(packet.data(), packet.size());
  if (tcdrain(fd_) < 0) {
    throw io_error("tcdrain failed");
  }

  const uint8_t ack = read_byte();
  if (ack != 0xffU) {
    throw std::runtime_error(
      "Invalid RoboClaw ACK: expected 0xFF, received " + std::to_string(ack));
  }
}

void RoboClawTransport::write_command_ack(
  uint8_t command, const std::vector<uint8_t> & payload)
{
  std::string last_error;
  for (int attempt = 0; attempt < retries_; ++attempt) {
    try {
      transact_write_once(command, payload);
      return;
    } catch (const std::exception & e) {
      last_error = e.what();
      flush_input();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  throw std::runtime_error(
    "RoboClaw write command " + std::to_string(command) +
    " failed after retries: " + last_error);
}

std::vector<uint8_t> RoboClawTransport::transact_read_once(
  uint8_t command, size_t data_length)
{
  flush_input();
  const uint8_t request[2] = {address_, command};
  write_all(request, sizeof(request));
  if (tcdrain(fd_) < 0) {
    throw io_error("tcdrain failed");
  }

  // Retains the short pacing used by the proven later RoboClaw driver.
  std::this_thread::sleep_for(std::chrono::microseconds(300));

  const auto response = read_exact(data_length + 2);
  uint16_t expected_crc = 0;
  expected_crc = crc_update(expected_crc, address_);
  expected_crc = crc_update(expected_crc, command);
  for (size_t i = 0; i < data_length; ++i) {
    expected_crc = crc_update(expected_crc, response[i]);
  }
  const uint16_t received_crc = decode_u16(response, data_length);
  if (expected_crc != received_crc) {
    throw std::runtime_error(
      "RoboClaw CRC mismatch on command " + std::to_string(command));
  }

  return std::vector<uint8_t>(response.begin(), response.begin() + data_length);
}

std::vector<uint8_t> RoboClawTransport::read_command(uint8_t command, size_t data_length)
{
  std::string last_error;
  for (int attempt = 0; attempt < retries_; ++attempt) {
    try {
      return transact_read_once(command, data_length);
    } catch (const std::exception & e) {
      last_error = e.what();
      flush_input();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  throw std::runtime_error(
    "RoboClaw read command " + std::to_string(command) +
    " failed after retries: " + last_error);
}

std::string RoboClawTransport::firmware_version()
{
  std::string last_error;
  for (int attempt = 0; attempt < retries_; ++attempt) {
    try {
      flush_input();
      const uint8_t request[2] = {address_, CMD_GET_VERSION};
      write_all(request, sizeof(request));
      if (tcdrain(fd_) < 0) {
        throw io_error("tcdrain failed");
      }

      uint16_t crc = 0;
      crc = crc_update(crc, address_);
      crc = crc_update(crc, CMD_GET_VERSION);

      std::string version;
      bool terminated = false;
      for (size_t i = 0; i < 48; ++i) {
        const uint8_t b = read_byte();
        crc = crc_update(crc, b);
        if (b == 0) {
          terminated = true;
          break;
        }
        version.push_back(static_cast<char>(b));
      }
      if (!terminated) {
        throw std::runtime_error("RoboClaw firmware version string exceeded 48 bytes");
      }

      const uint16_t received_crc = static_cast<uint16_t>(read_byte()) << 8 | read_byte();
      if (received_crc != crc) {
        throw std::runtime_error("RoboClaw firmware version CRC mismatch");
      }
      return version;
    } catch (const std::exception & e) {
      last_error = e.what();
      flush_input();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  throw std::runtime_error("Unable to read RoboClaw firmware version: " + last_error);
}

void RoboClawTransport::set_velocity_pid(
  Motor motor, double p, double i, double d, uint32_t qpps)
{
  const auto to_fixed = [](double value) -> uint32_t {
      return static_cast<uint32_t>(value * 65536.0);
    };

  std::vector<uint8_t> payload;
  payload.reserve(16);
  // RoboClaw packet order is D, P, I, QPPS (same order used by the later driver).
  append_u32(payload, to_fixed(d));
  append_u32(payload, to_fixed(p));
  append_u32(payload, to_fixed(i));
  append_u32(payload, qpps);
  write_command_ack(motor == Motor::M1 ? CMD_SET_M1_PID : CMD_SET_M2_PID, payload);
}

void RoboClawTransport::reset_encoders()
{
  write_command_ack(CMD_RESET_ENCODERS, {});
}

void RoboClawTransport::set_encoder(Motor motor, int32_t value)
{
  std::vector<uint8_t> payload;
  append_u32(payload, static_cast<uint32_t>(value));
  write_command_ack(motor == Motor::M1 ? CMD_SET_M1_ENCODER : CMD_SET_M2_ENCODER, payload);
}

RoboClawTransport::EncoderReading RoboClawTransport::read_encoder(Motor motor)
{
  const auto data = read_command(
    motor == Motor::M1 ? CMD_GET_M1_ENCODER : CMD_GET_M2_ENCODER, 5);
  return EncoderReading{decode_u32(data, 0), data[4]};
}

int32_t RoboClawTransport::read_speed(Motor motor)
{
  // RoboClaw returns a signed 32-bit speed followed by a status byte. This
  // mirrors Basicmicro's Python _read4_1(), which calls _readslong() first.
  const auto data = read_command(motor == Motor::M1 ? CMD_GET_M1_SPEED : CMD_GET_M2_SPEED, 5);
  return static_cast<int32_t>(decode_u32(data, 0));
}

void RoboClawTransport::set_serial_timeout(uint8_t deciseconds)
{
  write_command_ack(CMD_SET_SERIAL_TIMEOUT, {deciseconds});
}

void RoboClawTransport::set_main_voltages(uint16_t minimum_tenths, uint16_t maximum_tenths)
{
  std::vector<uint8_t> payload;
  append_u16(payload, minimum_tenths);
  append_u16(payload, maximum_tenths);
  write_command_ack(CMD_SET_MAIN_VOLTAGES, payload);
}

void RoboClawTransport::set_pin_functions(uint8_t s3_mode, uint8_t s4_mode, uint8_t s5_mode)
{
  write_command_ack(CMD_SET_PIN_FUNCTIONS, {s3_mode, s4_mode, s5_mode});
}

double RoboClawTransport::read_main_battery_voltage()
{
  return static_cast<double>(decode_u16(read_command(CMD_GET_MAIN_BATTERY, 2), 0)) / 10.0;
}

double RoboClawTransport::read_logic_battery_voltage()
{
  return static_cast<double>(decode_u16(read_command(CMD_GET_LOGIC_BATTERY, 2), 0)) / 10.0;
}

RoboClawTransport::MotorCurrents RoboClawTransport::read_motor_currents()
{
  const auto data = read_command(CMD_GET_CURRENTS, 4);
  const int16_t m1 = static_cast<int16_t>(decode_u16(data, 0));
  const int16_t m2 = static_cast<int16_t>(decode_u16(data, 2));
  return MotorCurrents{static_cast<double>(m1) / 100.0, static_cast<double>(m2) / 100.0};
}

double RoboClawTransport::read_temperature(bool second_sensor)
{
  const auto data = read_command(second_sensor ? CMD_GET_TEMPERATURE_2 : CMD_GET_TEMPERATURE, 2);
  return static_cast<double>(decode_u16(data, 0)) / 10.0;
}

uint32_t RoboClawTransport::read_status()
{
  return decode_u32(read_command(CMD_GET_STATUS, 4), 0);
}

void RoboClawTransport::drive_speed_accel(
  uint32_t acceleration_qpps_per_second,
  int32_t m1_speed_qpps, int32_t m2_speed_qpps)
{
  std::vector<uint8_t> payload;
  payload.reserve(12);
  append_u32(payload, acceleration_qpps_per_second);
  append_u32(payload, static_cast<uint32_t>(m1_speed_qpps));
  append_u32(payload, static_cast<uint32_t>(m2_speed_qpps));
  write_command_ack(CMD_MIXED_SPEED_ACCEL, payload);
}

void RoboClawTransport::drive_speed_accel_distance(
  uint32_t acceleration_qpps_per_second,
  int32_t m1_speed_qpps, uint32_t m1_max_distance_counts,
  int32_t m2_speed_qpps, uint32_t m2_max_distance_counts,
  bool cancel_previous)
{
  std::vector<uint8_t> payload;
  payload.reserve(21);
  append_u32(payload, acceleration_qpps_per_second);
  append_u32(payload, static_cast<uint32_t>(m1_speed_qpps));
  append_u32(payload, m1_max_distance_counts);
  append_u32(payload, static_cast<uint32_t>(m2_speed_qpps));
  append_u32(payload, m2_max_distance_counts);
  payload.push_back(cancel_previous ? 1U : 0U);
  write_command_ack(CMD_MIXED_SPEED_ACCEL_DISTANCE, payload);
}

void RoboClawTransport::stop(uint32_t emergency_deceleration_qpps_per_second)
{
  // Command 40 is an unbounded speed/acceleration command. Using it for the
  // explicit zero-speed path avoids relying on distance-command semantics when
  // the requested distance is zero, while retaining the emergency deceleration.
  drive_speed_accel(emergency_deceleration_qpps_per_second, 0, 0);
}

}  // namespace k9_drive_pkg
