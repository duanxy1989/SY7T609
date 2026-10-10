#include "sy7t609_uart.h"

#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sy7t609 {

static const char *const TAG = "sy7t609";

float SY7T609_UART::parse_milli_(uint32_t raw) { return static_cast<float>(raw) / 1000.0f; }

float SY7T609_UART::parse_signed_milli_(uint32_t raw) {
  raw &= 0x00FFFFFFu;
  const int32_t value = (raw & 0x00800000u) != 0 ? static_cast<int32_t>(raw | 0xFF000000u)
                                                   : static_cast<int32_t>(raw);
  return static_cast<float>(value) / 1000.0f;
}

float SY7T609_UART::parse_raw_(uint32_t raw) {
  // 数据手册：EPPCNT 每增加 1，表示累计一个 BUCKET。当前固定 BUCKET 校准值对应 1 Wh。
  return static_cast<float>(raw);
}

uint32_t SY7T609_UART::extract_data_(const std::vector<uint8_t> &frame) {
  return (static_cast<uint32_t>(frame[4]) << 16) | (static_cast<uint32_t>(frame[3]) << 8) |
         static_cast<uint32_t>(frame[2]);
}

float SY7T609_UART::apply_deadband_(uint16_t addr, float value) const {
  // 仅对功率类读数施加死区：绝对值低于阈值时视为空载噪声并归零，
  // 高于阈值的读数原样返回（含负号），因此真正的反向功率不受影响。
  float threshold = 0.0f;
  switch (addr) {
    case ADDR_POWER:
      threshold = power_deadband_;
      break;
    case ADDR_VAR:
      threshold = reactive_power_deadband_;
      break;
    case ADDR_PF:
      threshold = power_factor_deadband_;
      break;
    default:
      return value;
  }
  return (std::fabs(value) < threshold) ? 0.0f : value;
}

const std::array<SY7T609_UART::Measurement, 10> &SY7T609_UART::get_measurements_() {
  static const std::array<Measurement, 10> table = {{
      {ADDR_PF, &SY7T609_UART::power_factor_sensor_, &SY7T609_UART::parse_signed_milli_},
      {ADDR_VRMS, &SY7T609_UART::voltage_sensor_, &SY7T609_UART::parse_milli_},
      {ADDR_IRMS, &SY7T609_UART::current_sensor_, &SY7T609_UART::parse_milli_},
      {ADDR_POWER, &SY7T609_UART::power_sensor_, &SY7T609_UART::parse_signed_milli_},
      {ADDR_VAR, &SY7T609_UART::power_reactive_sensor_, &SY7T609_UART::parse_signed_milli_},
      {ADDR_VA, &SY7T609_UART::apparent_power_sensor_, &SY7T609_UART::parse_signed_milli_},
      {ADDR_FREQUENCY, &SY7T609_UART::frequency_sensor_, &SY7T609_UART::parse_milli_},
      {ADDR_EPPCNT, &SY7T609_UART::energy_sensor_, &SY7T609_UART::parse_raw_},
      {ADDR_EPMCNT, &SY7T609_UART::reverse_energy_sensor_, &SY7T609_UART::parse_raw_},
      {ADDR_CTEMP, &SY7T609_UART::temperature_sensor_, &SY7T609_UART::parse_signed_milli_},
  }};
  return table;
}

void SY7T609_UART::setup() {
  this->flush();
  rx_buffer_.reserve(MAX_RX_BUFFER_SIZE);
  // 按用户要求保留：每次启动均软复位计量芯片。
  if (!send_write_and_wait_ack_(ADDR_COMMAND, CMD_REG_SOFT_RESET, 250)) {
    ESP_LOGW(TAG, "Soft reset was sent but no valid ACK was received");
  }
}

void SY7T609_UART::loop() {
  collect_uart_();

  std::vector<uint8_t> frame;
  while (extract_frame_(frame)) {
    if (m_state == ProcessState::READING) {
      handle_read_frame_(frame);
    } else {
      ESP_LOGV(TAG, "Ignoring unsolicited reply 0x%02X", frame[0]);
    }
    frame.clear();
  }

  if (m_state == ProcessState::READING && millis() - last_read_ >= READ_TIMEOUT_MS) {
    retry_or_advance_("timeout");
  }

  if (m_state == ProcessState::IDLE) handleActionCallback();
}

void SY7T609_UART::update() {
  if (m_state == ProcessState::IDLE && !isNeedHandleActionCallback() && seek_next_active_(0)) {
    retry_count_ = 0;
    start_current_read_();
  } else if (m_state != ProcessState::IDLE) {
    ESP_LOGD(TAG, "Skipping update while a read cycle is active");
  }
}

void SY7T609_UART::collect_uart_() {
  while (this->available()) {
    const int value = this->read();
    if (value < 0) break;
    if (rx_buffer_.size() >= MAX_RX_BUFFER_SIZE) {
      ESP_LOGW(TAG, "RX buffer overflow; discarding oldest byte");
      rx_buffer_.erase(rx_buffer_.begin());
    }
    rx_buffer_.push_back(static_cast<uint8_t>(value));
  }
}

bool SY7T609_UART::is_reply_code_(uint8_t code) {
  switch (code) {
    case REPLY_ACK_WITH_DATA:
    case REPLY_AUTO_REPORTING_HEADER:
    case REPLY_ACK_WITHOUT_DATA:
    case REPLY_NEGATIVE_ACK:
    case REPLY_COMMAND_NOT_IMPLEMENTED:
    case REPLY_CHECKSUM_FAILED:
    case REPLY_BUFFER_OVERFLOW:
      return true;
    default:
      return false;
  }
}

bool SY7T609_UART::frame_checksum_valid_(const std::vector<uint8_t> &frame) {
  uint8_t sum = 0;
  for (uint8_t byte : frame) sum = static_cast<uint8_t>(sum + byte);
  return sum == 0;
}

bool SY7T609_UART::extract_frame_(std::vector<uint8_t> &frame) {
  while (!rx_buffer_.empty() && !is_reply_code_(rx_buffer_[0])) rx_buffer_.erase(rx_buffer_.begin());
  if (rx_buffer_.size() < SSI_DEFAULT_FRAME_SIZE) return false;

  const std::size_t frame_size = rx_buffer_[1];
  if (frame_size < SSI_DEFAULT_FRAME_SIZE || frame_size > MAX_FRAME_SIZE) {
    ESP_LOGW(TAG, "Invalid SSI frame length %u", static_cast<unsigned>(frame_size));
    rx_buffer_.erase(rx_buffer_.begin());
    return extract_frame_(frame);
  }
  if (rx_buffer_.size() < frame_size) return false;

  frame.assign(rx_buffer_.begin(), rx_buffer_.begin() + frame_size);
  rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_size);
  if (!frame_checksum_valid_(frame)) {
    ESP_LOGW(TAG, "Discarding SSI frame with invalid checksum");
    if (m_state == ProcessState::READING) retry_or_advance_("checksum error");
    frame.clear();
    return extract_frame_(frame);
  }
  return true;
}

void SY7T609_UART::handle_read_frame_(const std::vector<uint8_t> &frame) {
  if (frame[0] != REPLY_ACK_WITH_DATA || frame.size() != SSI_UART_READ_RECV_PKG_SIZE) {
    ESP_LOGW(TAG, "Read failed: reply=0x%02X length=%u", frame[0], static_cast<unsigned>(frame.size()));
    retry_or_advance_("negative or malformed reply");
    return;
  }

  const auto &measurement = get_measurements_()[read_index_];
  sensor::Sensor *target = this->*(measurement.sensor_ptr);
  if (target != nullptr) {
    const float value = apply_deadband_(measurement.addr, measurement.parse(extract_data_(frame)));
    target->publish_state(value);
  }
  retry_count_ = 0;
  advance_read_();
}

bool SY7T609_UART::seek_next_active_(std::size_t from) {
  const auto &table = get_measurements_();
  for (std::size_t i = from; i < table.size(); i++) {
    if (this->*(table[i].sensor_ptr) != nullptr) {
      read_index_ = i;
      return true;
    }
  }
  return false;
}

void SY7T609_UART::start_current_read_() {
  rx_buffer_.clear();
  while (this->available()) this->read();
  uartSendReadCmd(get_measurements_()[read_index_].addr);
  m_state = ProcessState::READING;
  last_read_ = millis();
}

void SY7T609_UART::advance_read_() {
  if (seek_next_active_(read_index_ + 1)) {
    start_current_read_();
  } else {
    m_state = ProcessState::IDLE;
    read_index_ = 0;
    retry_count_ = 0;
  }
}

void SY7T609_UART::retry_or_advance_(const char *reason) {
  if (retry_count_ < MAX_READ_RETRIES) {
    retry_count_++;
    ESP_LOGW(TAG, "Retrying register 0x%04X (%u/%u): %s", get_measurements_()[read_index_].addr,
             retry_count_, MAX_READ_RETRIES, reason);
    start_current_read_();
    return;
  }
  ESP_LOGE(TAG, "Skipping register 0x%04X after retries: %s", get_measurements_()[read_index_].addr, reason);
  retry_count_ = 0;
  advance_read_();
}

void SY7T609_UART::uartSendReadCmd(uint16_t addr) {
  std::array<uint8_t, SSI_UART_READ_SEND_PKG_SIZE> data{};
  data[0] = SSI_HEADER;
  data[1] = data.size();
  data[2] = CMD_SELECT_REGISTER_ADDRESS;
  data[3] = addr & 0xFF;
  data[4] = (addr >> 8) & 0xFF;
  data[5] = CMD_READ_REGITSTER_3BYTES;
  data[6] = checksum(data);
  this->write_array(data);
  this->flush();
}

void SY7T609_UART::uartSendWriteCmd(uint16_t addr, uint32_t value) {
  std::array<uint8_t, SSI_UART_WRITE_SEND_PKG_SIZE> data{};
  data[0] = SSI_HEADER;
  data[1] = data.size();
  data[2] = CMD_SELECT_REGISTER_ADDRESS;
  data[3] = addr & 0xFF;
  data[4] = (addr >> 8) & 0xFF;
  data[5] = CMD_WRITE_RETISTER_3BYTES;
  data[6] = value & 0xFF;
  data[7] = (value >> 8) & 0xFF;
  data[8] = (value >> 16) & 0xFF;
  data[9] = checksum(data);
  this->write_array(data);
  this->flush();
}

bool SY7T609_UART::send_write_and_wait_ack_(uint16_t addr, uint32_t value, uint32_t timeout_ms) {
  rx_buffer_.clear();
  while (this->available()) this->read();
  uartSendWriteCmd(addr, value);

  const uint32_t started = millis();
  while (millis() - started < timeout_ms) {
    collect_uart_();
    std::vector<uint8_t> frame;
    if (extract_frame_(frame)) {
      if (frame[0] == REPLY_ACK_WITHOUT_DATA && frame.size() == SSI_DEFAULT_FRAME_SIZE) return true;
      ESP_LOGW(TAG, "Write rejected: reply=0x%02X", frame[0]);
      return false;
    }
    delay(1);
    yield();
  }
  ESP_LOGW(TAG, "Write ACK timeout for register 0x%04X", addr);
  return false;
}

int SY7T609_UART::addActionCallBack(ActionCallbackFuncPtr ptrFunc) {
  m_vecActionCallback.push_back(ptrFunc);
  return static_cast<int>(m_vecActionCallback.size());
}

void SY7T609_UART::handleActionCallback() {
  if (m_vecActionCallback.empty()) return;
  const auto callbacks = m_vecActionCallback;
  m_vecActionCallback.clear();
  for (auto callback : callbacks) {
    if (callback != nullptr) (this->*callback)();
  }
  m_state = ProcessState::IDLE;
  read_index_ = 0;
  retry_count_ = 0;
}

void SY7T609_UART::reset_energy_() {
  ESP_LOGI(TAG, "Clearing energy counters");
  if (send_write_and_wait_ack_(ADDR_COMMAND, CMD_REG_CLEAR_ENGERGY_COUNTERS)) {
    if (energy_sensor_ != nullptr) energy_sensor_->publish_state(0.0f);
    ESP_LOGI(TAG, "Energy counters cleared");
  } else {
    ESP_LOGE(TAG, "Energy counter clear failed; state was not changed");
  }
}

void SY7T609_UART::reset_calibration_() {
  struct RegisterValue { uint16_t addr; uint32_t value; };
  static const RegisterValue values[] = {
      {ADDR_IGAIN, DEFAULT_IGAIN}, {ADDR_VGAIN, DEFAULT_VGAIN}, {ADDR_ISCALE, DEFAULT_ISCALE},
      {ADDR_VSCALE, DEFAULT_VSCALE}, {ADDR_PSCALE, DEFAULT_PSCALE}, {ADDR_ACCUM, DEFAULT_ACCUM},
      {ADDR_IRMS_TARGET, DEFAULT_IRMS_TARGET}, {ADDR_VRMS_TARGET, DEFAULT_VRMS_TARGET},
      {ADDR_POWER_TARGET, DEFAULT_POWER_TARGET}, {ADDR_BUCKETL, DEFAULT_BUCKETL},
      {ADDR_BUCKETH, DEFAULT_BUCKETH}, {ADDR_CONTROL, DEFAULT_CONTROL},
  };

  ESP_LOGI(TAG, "Applying fixed calibration");
  for (const auto &item : values) {
    if (!send_write_and_wait_ack_(item.addr, item.value)) {
      ESP_LOGE(TAG, "Calibration aborted at register 0x%04X", item.addr);
      return;
    }
  }
  if (!send_write_and_wait_ack_(ADDR_COMMAND, CMD_REG_SAVE_TO_FLASH, 300)) {
    ESP_LOGE(TAG, "Calibration applied but saving to flash failed");
    return;
  }
  ESP_LOGI(TAG, "Calibration saved to flash");
}

}  // namespace sy7t609
}  // namespace esphome
