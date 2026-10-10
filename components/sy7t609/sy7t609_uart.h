#pragma once

#include <array>
#include <vector>

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/datatypes.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "sy7t609_def.h"

namespace esphome {
namespace sy7t609 {

template<typename... Ts> class ResetEnergyAction;
template<typename... Ts> class ResetCalibrationAction;

class SY7T609_UART;
typedef void (SY7T609_UART::*ActionCallbackFuncPtr)(void);
typedef float (*MeasurementParser)(uint32_t raw);

class SY7T609_UART : public PollingComponent, public uart::UARTDevice {
 public:
  struct Measurement {
    uint16_t addr;
    sensor::Sensor *SY7T609_UART::*sensor_ptr;
    MeasurementParser parse;
  };

  void set_power_factor_sensor(sensor::Sensor *s) { power_factor_sensor_ = s; }
  void set_voltage_sensor(sensor::Sensor *s) { voltage_sensor_ = s; }
  void set_current_sensor(sensor::Sensor *s) { current_sensor_ = s; }
  void set_power_sensor(sensor::Sensor *s) { power_sensor_ = s; }
  void set_power_reactive_sensor(sensor::Sensor *s) { power_reactive_sensor_ = s; }
  void set_apparent_power_sensor(sensor::Sensor *s) { apparent_power_sensor_ = s; }
  void set_frequency_sensor(sensor::Sensor *s) { frequency_sensor_ = s; }
  void set_energy_sensor(sensor::Sensor *s) { energy_sensor_ = s; }
  void set_reverse_energy_sensor(sensor::Sensor *s) { reverse_energy_sensor_ = s; }
  void set_temperature_sensor(sensor::Sensor *s) { temperature_sensor_ = s; }

  // 死区阈值：绝对值小于该值的近零读数按 0 处理，
  // 超过阈值的读数保留原始符号，因此仍可反映反向功率（倒送）。
  void set_power_deadband(float v) { power_deadband_ = v; }
  void set_reactive_power_deadband(float v) { reactive_power_deadband_ = v; }
  void set_power_factor_deadband(float v) { power_factor_deadband_ = v; }

  void setup() override;
  void loop() override;
  void update() override;

 protected:
  const std::array<Measurement, 10> &get_measurements_();
  float apply_deadband_(uint16_t addr, float value) const;
  bool seek_next_active_(std::size_t from);
  void start_current_read_();
  void advance_read_();
  void retry_or_advance_(const char *reason);

  void uartSendReadCmd(uint16_t addr);
  void uartSendWriteCmd(uint16_t addr, uint32_t value);
  bool send_write_and_wait_ack_(uint16_t addr, uint32_t value, uint32_t timeout_ms = 150);

  void collect_uart_();
  bool extract_frame_(std::vector<uint8_t> &frame);
  void handle_read_frame_(const std::vector<uint8_t> &frame);
  static bool is_reply_code_(uint8_t code);
  static bool frame_checksum_valid_(const std::vector<uint8_t> &frame);

  void reset_energy_();
  void reset_calibration_();

  static float parse_milli_(uint32_t raw);
  static float parse_signed_milli_(uint32_t raw);
  static float parse_raw_(uint32_t raw);
  static uint32_t extract_data_(const std::vector<uint8_t> &frame);

  int addActionCallBack(ActionCallbackFuncPtr ptrFunc);
  void handleActionCallback();
  bool isNeedHandleActionCallback() { return !m_vecActionCallback.empty(); }

  template<std::size_t SIZE> uint8_t checksum(const std::array<uint8_t, SIZE> &array) {
    uint8_t sum = 0;
    for (std::size_t i = 0; i < array.size() - 1; i++) sum += array[i];
    return static_cast<uint8_t>(~sum + 1);
  }

  template<typename... Ts> friend class ResetEnergyAction;
  template<typename... Ts> friend class ResetCalibrationAction;

  ProcessState m_state{ProcessState::IDLE};
  std::size_t read_index_{0};
  uint8_t retry_count_{0};
  uint32_t last_read_{0};
  std::vector<uint8_t> rx_buffer_{};

  sensor::Sensor *power_factor_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *power_reactive_sensor_{nullptr};
  sensor::Sensor *apparent_power_sensor_{nullptr};
  sensor::Sensor *frequency_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  sensor::Sensor *reverse_energy_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  float power_deadband_{0.5f};
  float reactive_power_deadband_{0.5f};
  float power_factor_deadband_{0.05f};

 private:
  static constexpr uint8_t MAX_READ_RETRIES = 2;
  static constexpr std::size_t MAX_FRAME_SIZE = SSI_DEFAULT_FRAME_SIZE + SSI_MAX_PAYLOAD_SIZE;
  static constexpr std::size_t MAX_RX_BUFFER_SIZE = 64;
  static constexpr uint32_t READ_TIMEOUT_MS = 500;
  std::vector<ActionCallbackFuncPtr> m_vecActionCallback{};
};

template<typename... Ts> class ResetEnergyAction : public Action<Ts...> {
 public:
  explicit ResetEnergyAction(SY7T609_UART *parent) : parent_(parent) {}
  void play(Ts... x) override { this->parent_->addActionCallBack(&SY7T609_UART::reset_energy_); }
 protected:
  SY7T609_UART *parent_;
};

template<typename... Ts> class ResetCalibrationAction : public Action<Ts...> {
 public:
  explicit ResetCalibrationAction(SY7T609_UART *parent) : parent_(parent) {}
  void play(Ts... x) override { this->parent_->addActionCallBack(&SY7T609_UART::reset_calibration_); }
 protected:
  SY7T609_UART *parent_;
};

}  // namespace sy7t609
}  // namespace esphome
