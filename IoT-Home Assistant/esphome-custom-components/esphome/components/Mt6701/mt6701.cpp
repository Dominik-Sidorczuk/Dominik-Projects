#include "mt6701.h" // Assume this file exists and contains the class definition and enums
#include "esphome/core/log.h"
#include "esphome/core/helpers.h" // For YESNO, str_equals_case_insensitive, etc.
#include "esphome/core/hal.h"     // For PI

namespace esphome {
namespace mt6701 {

static const char *const TAG = "mt6701";

// --- MT6701 Chip-Specific Constants ---
// MT6701 resolution is 14 bits
static constexpr uint16_t MT6701_RESOLUTION_COUNTS = 1 << 14; // 16384
static constexpr float COUNTS_PER_REVOLUTION = static_cast<float>(MT6701_RESOLUTION_COUNTS);

static constexpr float COUNTS_TO_DEGREES_FACTOR = 360.0f / COUNTS_PER_REVOLUTION;
static constexpr float COUNTS_TO_RADIANS_FACTOR = (2.0f * PI) / COUNTS_PER_REVOLUTION; // PI from esphome/core/hal.h
// (degrees / us) * (1 revolution / 360 degrees) * (60 s / 1 min) * (10^6 us / 1 s) = revolutions / min
static constexpr float DEGREES_PER_US_TO_RPM_FACTOR = (1.0f / 360.0f) * 60.0f * 1000000.0f;

// Constants for SSI Interface
static constexpr uint8_t SSI_ANGLE_DATA_BITS = 14;
static constexpr uint8_t SSI_STATUS_DATA_BITS = 4;
static constexpr uint8_t SSI_CRC_BITS = 6;
static constexpr uint8_t SSI_TOTAL_FRAME_BITS = SSI_ANGLE_DATA_BITS + SSI_STATUS_DATA_BITS + SSI_CRC_BITS; // 14+4+6 = 24 bits
static constexpr uint8_t SSI_FRAME_SIZE_BYTES = (SSI_TOTAL_FRAME_BITS + 7) / 8; // Should be 3 bytes
static constexpr uint8_t SSI_CRC6_ITU_POLY_REDUCED = 0x03; // Polynomial G(x)=x^6+x^1.
                                                          // For CRC implementations where x^n is implicit and XOR is used, 0x03 is correct for G(x)=x^6+x^1.
static constexpr uint16_t SSI_RAW_ANGLE_MASK = (1 << SSI_ANGLE_DATA_BITS) - 1; // 0x3FFF
static constexpr uint8_t SSI_STATUS_MASK = (1 << SSI_STATUS_DATA_BITS) - 1;    // 0x0F
static constexpr uint8_t SSI_CRC_MASK = (1 << SSI_CRC_BITS) - 1;               // 0x3F


// --- Methods for setting configuration from YAML ---
void MT6701SensorComponent::set_velocity_filter_type_str(const std::string &type_str) {
  if (str_equals_case_insensitive(type_str, "NONE")) {
    this->velocity_filter_type_ = VelocityFilterType::NONE;
  } else if (str_equals_case_insensitive(type_str, "EMA")) {
    this->velocity_filter_type_ = VelocityFilterType::EMA;
  } else if (str_equals_case_insensitive(type_str, "BUTTERWORTH_2ND_ORDER")) {
    this->velocity_filter_type_ = VelocityFilterType::BUTTERWORTH_2ND_ORDER;
  } else {
    ESP_LOGW(TAG, "Unknown velocity filter type: '%s'. Defaulting to EMA.", type_str.c_str());
    this->velocity_filter_type_ = VelocityFilterType::EMA; // Default
  }
}

void MT6701SensorComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MT6701 Sensor Component...");

  // Validate interface type
  if (this->interface_type_ == MT6701InterfaceType::NONE) {
    ESP_LOGE(TAG, "Interface type not set! Aborting setup.");
    this->mark_failed();
    return;
  }

  // Interface-specific initialization and communication test
  if (this->interface_type_ == MT6701InterfaceType::I2C) {
    ESP_LOGCONFIG(TAG, "Interface: I2C (Address: 0x%02X)", this->address_);
    // I2CDevice::setup() is usually called by ESPHome framework
    // Additional read test to confirm communication
    uint8_t temp_val;
    if (this->read_register(static_cast<uint8_t>(MT6701Register::ANGLE_DATA_H), &temp_val, 1, true) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "I2C communication test failed. Check address and wiring.");
      this->mark_failed();
      return;
    }
    ESP_LOGD(TAG, "I2C communication test successful.");
  } else if (this->interface_type_ == MT6701InterfaceType::SSI) {
    ESP_LOGCONFIG(TAG, "Interface: SSI (SPI)");
    // SPIDeviceComponent::setup() is usually called by ESPHome framework
    // Can log SPI parameters like this->cs_pin_->get_pin() if needed
  }

  // Initialize variables for velocity and filters
  this->last_velocity_update_time_us_ = micros();
  this->first_velocity_sample_ = true;
  this->filtered_velocity_rpm_ = 0.0f;
  this->x_n1_ = 0.0f; this->x_n2_ = 0.0f; // Input history for Butterworth
  this->y_n1_ = 0.0f; this->y_n2_ = 0.0f; // Output history for Butterworth
  this->ssi_crc_error_count_ = 0; // Explicitly zero CRC error counter

  ESP_LOGCONFIG(TAG, "Velocity filter type: %d, Cutoff: %.2f Hz",
                static_cast<int>(this->velocity_filter_type_), this->velocity_filter_cutoff_hz_);
  ESP_LOGCONFIG(TAG, "Min velocity update period: %u us", this->min_velocity_update_period_us_);

  // Calculate Butterworth coefficients if that filter is selected
  if (this->velocity_filter_type_ == VelocityFilterType::BUTTERWORTH_2ND_ORDER && this->velocity_filter_cutoff_hz_ > 0.0f) {
    this->calculate_butterworth_coeffs_();
  }

  // Initial sensor reading
  // this->current_raw_sensor_value_ is set by read_sensor_data_()
  if (this->read_sensor_data_()) {
    this->internal_accumulated_count_ = 0; // Zero the accumulator
    this->previous_raw_sensor_value_for_velocity_ = this->current_raw_sensor_value_; // Initialize previous value for velocity calculation
    ESP_LOGD(TAG, "Initial raw sensor value: %u. Accumulator zeroed.", this->current_raw_sensor_value_);
  } else {
    ESP_LOGE(TAG, "Failed to read initial sensor value during setup. Component marked as failed.");
    this->mark_failed();
    return; // Do not proceed if initial read fails
  }

  // Register services
  this->register_service(&MT6701SensorComponent::on_set_zero_offset, "set_zero_offset");
  if (this->interface_type_ == MT6701InterfaceType::SSI) {
    this->register_service(&MT6701SensorComponent::on_reset_crc_error_count, "reset_ssi_crc_errors");
  }

  ESP_LOGCONFIG(TAG, "MT6701 setup complete.");
}

void MT6701SensorComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "MT6701 Sensor Component:");
  LOG_UPDATE_INTERVAL(this);

  if (this->interface_type_ == MT6701InterfaceType::I2C) {
    ESP_LOGCONFIG(TAG, "  Interface: I2C");
    LOG_I2C_DEVICE(this);
  } else if (this->interface_type_ == MT6701InterfaceType::SSI) {
    ESP_LOGCONFIG(TAG, "  Interface: SSI (SPI)");
    // Assuming it inherits from SPIDevice, this->spi_settings_.mode and .data_rate might be available
    // ESP_LOGCONFIG(TAG, "    SPI Mode: %d", static_cast<int>(this->spi_settings_.mode));
    // ESP_LOGCONFIG(TAG, "    SPI Clock Speed: %s Hz", format_freq(this->spi_settings_.data_rate).c_str());
    // If MT6701SensorComponent is an SPIDevice, `this->cs_pin_` should be available
    // if (this->cs_pin_) { ESP_LOGCONFIG(TAG, "    CS Pin: %s", this->cs_pin_->get_pin_pcf_desc().c_str()); }
    // Access to `parent_` (SPI bus component) would be needed for MISO/MOSI/CLK pins.
  }

  ESP_LOGCONFIG(TAG, "  Zero Offset: %.2f degrees", this->zero_offset_degrees_);
  ESP_LOGCONFIG(TAG, "  Direction Inverted: %s", YESNO(this->direction_inverted_));

  const char *filter_type_str = "UNKNOWN";
  switch (this->velocity_filter_type_) {
    case VelocityFilterType::NONE: filter_type_str = "None"; break;
    case VelocityFilterType::EMA: filter_type_str = "EMA (1st Order IIR)"; break;
    case VelocityFilterType::BUTTERWORTH_2ND_ORDER: filter_type_str = "Butterworth (2nd Order IIR)"; break;
  }
  ESP_LOGCONFIG(TAG, "  Velocity Filter Type: %s", filter_type_str);
  ESP_LOGCONFIG(TAG, "  Velocity Filter Cutoff: %.2f Hz (Filter: %s)",
                this->velocity_filter_cutoff_hz_,
                (this->velocity_filter_cutoff_hz_ > 0.0f && this->velocity_filter_type_ != VelocityFilterType::NONE ? "Enabled" : "Disabled"));
  ESP_LOGCONFIG(TAG, "  Min Velocity Update Period: %u us", this->min_velocity_update_period_us_);

  LOG_SENSOR("  ", "Angle Sensor (Main)", this->angle_sensor_);
  LOG_SENSOR("  ", "Accumulated Angle Sensor", this->accumulated_angle_sensor_);
  LOG_SENSOR("  ", "Velocity RPM Sensor", this->velocity_rpm_sensor_);
  LOG_SENSOR("  ", "Raw Count Sensor", this->raw_count_sensor_);
  LOG_SENSOR("  ", "Raw Radians Sensor", this->raw_radians_sensor_);
  LOG_SENSOR("  ", "Accumulated Count Sensor", this->accumulated_count_sensor_);
  LOG_SENSOR("  ", "Accumulated Radians Sensor", this->accumulated_radians_sensor_);

  if (this->interface_type_ == MT6701InterfaceType::SSI) {
    LOG_SENSOR("  ", "Magnetic Field Status Sensor", this->magnetic_field_status_sensor_);
    LOG_SENSOR("  ", "Loss of Track Status Sensor", this->loss_of_track_status_sensor_);
    LOG_BINARY_SENSOR("  ", "Push Button SSI Binary Sensor", this->push_button_ssi_binary_sensor_);
    LOG_SENSOR("  ", "SSI CRC Error Count Sensor", this->ssi_crc_error_sensor_);
  }
}

// Helper template to publish sensor state if sensor object exists
template <typename T>
void MT6701SensorComponent::publish_sensor_state_(esphome::sensor::Sensor *sensor, T state) {
  if (sensor != nullptr) {
    sensor->publish_state(state);
  }
}

// Overload for binary sensors
void MT6701SensorComponent::publish_sensor_state_(esphome::binary_sensor::BinarySensor *sensor, bool state) {
  if (sensor != nullptr) {
    sensor->publish_state(state);
  }
}

// Helper to publish NAN to all float sensors and default to binary sensors in case of read failure
void MT6701SensorComponent::publish_all_nan_or_default_() {
  publish_sensor_state_(this->angle_sensor_, NAN);
  publish_sensor_state_(this->accumulated_angle_sensor_, NAN);
  publish_sensor_state_(this->velocity_rpm_sensor_, NAN);
  publish_sensor_state_(this->raw_count_sensor_, NAN);
  publish_sensor_state_(this->raw_radians_sensor_, NAN);
  publish_sensor_state_(this->accumulated_count_sensor_, NAN);
  publish_sensor_state_(this->accumulated_radians_sensor_, NAN);

  if (this->interface_type_ == MT6701InterfaceType::SSI) {
    publish_sensor_state_(this->magnetic_field_status_sensor_, NAN);
    publish_sensor_state_(this->loss_of_track_status_sensor_, NAN);
    publish_sensor_state_(this->push_button_ssi_binary_sensor_, false); // Default to false for binary sensor
    // CRC error counter is cumulative and not reset or set to NAN here.
  }
}

void MT6701SensorComponent::update() {
  if (!read_sensor_data_()) {
    ESP_LOGW(TAG, "Failed to read data from MT6701 in update cycle.");
    publish_all_nan_or_default_();
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();
  process_angle_data_(this->current_raw_sensor_value_);
}

bool MT6701SensorComponent::read_sensor_data_() {
  uint16_t raw_angle_value = 0; // Will be updated by read methods
  bool read_success = false;

  if (this->interface_type_ == MT6701InterfaceType::I2C) {
    read_success = read_i2c_angle_data_(raw_angle_value);
  } else if (this->interface_type_ == MT6701InterfaceType::SSI) {
    uint8_t ssi_status_bits = 0;
    uint8_t received_ssi_crc = 0;
    read_success = read_ssi_frame_data_(raw_angle_value, ssi_status_bits, received_ssi_crc);
    if (read_success) {
      process_ssi_status_bits_(ssi_status_bits); // Process status bits if read and CRC are OK
    }
  } else {
    ESP_LOGE(TAG, "Unknown interface type in read_sensor_data_");
    return false; // Should not happen if setup() validates interface_type_
  }

  if (read_success) {
    this->current_raw_sensor_value_ = raw_angle_value;
  }
  return read_success;
}

bool MT6701SensorComponent::read_i2c_angle_data_(uint16_t &raw_angle_out) {
  uint8_t buffer[2]; // buffer[0] for ANGLE_H, buffer[1] for ANGLE_L
  // MT6701 sends 8 bits from ANGLE_H and the 6 MSBs from ANGLE_L
  // ANGLE_H: D13 D12 D11 D10 D9 D8 D7 D6
  // ANGLE_L: D5  D4  D3  D2 D1 D0 X  X   (X - unused bits)
  if (this->read_register(static_cast<uint8_t>(MT6701Register::ANGLE_DATA_H), &buffer[0], 1, true) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C read error from ANGLE_DATA_H register");
    return false;
  }
  if (this->read_register(static_cast<uint8_t>(MT6701Register::ANGLE_DATA_L), &buffer[1], 1, true) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C read error from ANGLE_DATA_L register");
    return false;
  }

  raw_angle_out = (static_cast<uint16_t>(buffer[0]) << 6) | (buffer[1] >> 2);
  raw_angle_out &= SSI_RAW_ANGLE_MASK; // Ensure it's within 14-bit range
  return true;
}

bool MT6701SensorComponent::read_ssi_frame_data_(uint16_t &raw_angle_out, uint8_t &status_bits_out, uint8_t &received_crc_out) {
  // Assume SSI_FRAME_SIZE_BYTES is 3
  uint8_t raw_ssi_frame[SSI_FRAME_SIZE_BYTES];
  uint8_t tx_dummy_buffer[SSI_FRAME_SIZE_BYTES] = {0x00, 0x00, 0x00}; // SPI transmit buffer (dummy bytes)

  // SPIDeviceComponent manages the SPI transaction, including CS
  this->enable(); // Activate CS (if not managed by SPIDevice transaction or if explicit control is needed)
  this->transfer_array(tx_dummy_buffer, raw_ssi_frame, SSI_FRAME_SIZE_BYTES);
  this->disable(); // Deactivate CS

  // Decode SSI frame (24 bits, MSB first)
  // Bits 23-10 (14 bits): Angle data
  // Bits 9-6  (4 bits): Status (MgS3, MgS2, MgS1, MgS0)
  // Bits 5-0  (6 bits): CRC

  // raw_ssi_frame[0]: D23 D22 D21 D20 D19 D18 D17 D16 (Angle MSB)
  // raw_ssi_frame[1]: D15 D14 D13 D12 D11 D10 D9  D8  (Angle LSB + Status MSB)
  // raw_ssi_frame[2]: D7  D6  D5  D4  D3  D2  D1  D0  (Status LSB + CRC)

  // Extract 14-bit angle data
  raw_angle_out = (static_cast<uint16_t>(raw_ssi_frame[0]) << 6) | (raw_ssi_frame[1] >> 2);
  raw_angle_out &= SSI_RAW_ANGLE_MASK;

  // Extract 4-bit status data: MGS3(D9), MGS2(D8), MGS1(D7), MGS0(D6)
  // D9, D8 are in raw_ssi_frame[1] (bits 1,0)
  // D7, D6 are in raw_ssi_frame[2] (bits 7,6)
  uint8_t mgs3_mgs2_bits = raw_ssi_frame[1] & 0x03;       // Bits D9, D8 from frame[1]
  uint8_t mgs1_mgs0_bits = (raw_ssi_frame[2] >> 6) & 0x03; // Bits D7, D6 from frame[2]
  status_bits_out = (mgs3_mgs2_bits << 2) | mgs1_mgs0_bits; // Combine into MGS3 MGS2 MGS1 MGS0
  status_bits_out &= SSI_STATUS_MASK; // Ensure only 4 bits

  // Extract 6-bit CRC
  received_crc_out = raw_ssi_frame[2] & SSI_CRC_MASK;

  // Verify CRC
  if (!verify_ssi_crc_(raw_angle_out, status_bits_out, received_crc_out)) {
    return false; // CRC error
  }
  return true;
}

// Calculates CRC-6-ITU for an 18-bit data word (14-bit angle + 4-bit status)
// Polynomial: G(x) = x^6 + x^1 (represented by 0x03 for the reduced poly in some CRC algorithms)
uint8_t MT6701SensorComponent::calculate_ssi_crc6_itu_(uint32_t data_18bit) const {
  uint8_t crc_reg = 0x00; // Initial value for CRC-6-ITU (sometimes 0x3F) - MT6701 datasheet doesn't specify; 0x00 is common.
  const int num_data_bits = SSI_ANGLE_DATA_BITS + SSI_STATUS_DATA_BITS; // 18 bits

  for (int bit_pos = num_data_bits - 1; bit_pos >= 0; bit_pos--) {
    bool data_bit = (data_18bit >> bit_pos) & 0x01;
    bool crc_msb = (crc_reg & (1 << (SSI_CRC_BITS - 1))) != 0; // Check MSB (bit 5 for CRC6)
    
    crc_reg <<= 1;
    if (data_bit ^ crc_msb) {
      crc_reg ^= SSI_CRC6_ITU_POLY_REDUCED; // Use the reduced polynomial
    }
  }
  return crc_reg & SSI_CRC_MASK; // Return only the 6 CRC bits
}

bool MT6701SensorComponent::verify_ssi_crc_(uint16_t angle_data, uint8_t status_data, uint8_t received_crc) {
  // Data word for CRC is [14-bit angle | 4-bit status]
  uint32_t data_word_for_crc = (static_cast<uint32_t>(angle_data & SSI_RAW_ANGLE_MASK) << SSI_STATUS_DATA_BITS) | (status_data & SSI_STATUS_MASK);
  uint8_t calculated_crc = calculate_ssi_crc6_itu_(data_word_for_crc);

  if (calculated_crc != received_crc) {
    ESP_LOGW(TAG, "SSI CRC MISMATCH! Data: Angle=0x%04X, Status=0x%X. CalcCRC=0x%02X, RecvCRC=0x%02X",
             angle_data, status_data, calculated_crc, received_crc);
    this->ssi_crc_error_count_++;
    publish_sensor_state_(this->ssi_crc_error_sensor_, static_cast<float>(this->ssi_crc_error_count_));
    return false;
  }
  ESP_LOGV(TAG, "SSI CRC OK. Data: Angle=0x%04X, Status=0x%X. CRC=0x%02X", angle_data, status_data, received_crc);
  return true;
}

void MT6701SensorComponent::process_ssi_status_bits_(uint8_t status_bits) {
  // status_bits format: [MGS3, MGS2, MGS1, MGS0]
  // MGS1_MgS0 (Magnetic Field Strength): bits 1,0 of status_bits -> (status_bits & 0x03)
  // MGS3 (Loss of Track): bit 3 of status_bits -> (status_bits >> 3) & 0x01
  // MGS2 (Push Button): bit 2 of status_bits -> (status_bits >> 2) & 0x01
  publish_sensor_state_(this->magnetic_field_status_sensor_, static_cast<float>(status_bits & 0x03));
  publish_sensor_state_(this->loss_of_track_status_sensor_, static_cast<float>((status_bits >> 3) & 0x01));
  publish_sensor_state_(this->push_button_ssi_binary_sensor_, static_cast<bool>((status_bits >> 2) & 0x01));
}

void MT6701SensorComponent::process_angle_data_(uint16_t raw_angle_from_sensor) {
  // Publish raw sensor values
  publish_sensor_state_(this->raw_count_sensor_, static_cast<float>(raw_angle_from_sensor));
  publish_sensor_state_(this->raw_radians_sensor_, static_cast<float>(raw_angle_from_sensor) * COUNTS_TO_RADIANS_FACTOR);

  // Calculate accumulated angle
  int16_t current_processed_value = static_cast<int16_t>(raw_angle_from_sensor);
  // Difference from previous raw value
  int16_t diff = current_processed_value - static_cast<int16_t>(this->previous_raw_sensor_value_for_velocity_);

  // Handle wraparound (e.g., transition from 16383 to 0 or vice-versa)
  if (std::abs(diff) > (MT6701_RESOLUTION_COUNTS / 2)) {
    if (diff > 0) { // e.g. prev=16380, current=10 -> diff = -16370. Should be +13
      diff -= MT6701_RESOLUTION_COUNTS; // Corrects large positive diff to small negative
    } else { // e.g. prev=10, current=16380 -> diff = 16370. Should be -13
      diff += MT6701_RESOLUTION_COUNTS; // Corrects large negative diff to small positive
    }
  }

  // Update internal accumulated count based on direction
  if (this->direction_inverted_) {
    this->internal_accumulated_count_ -= diff;
  } else {
    this->internal_accumulated_count_ += diff;
  }

  // Publish accumulated values
  publish_sensor_state_(this->accumulated_count_sensor_, static_cast<float>(this->internal_accumulated_count_));
  float accumulated_degrees_val = static_cast<float>(this->internal_accumulated_count_) * COUNTS_TO_DEGREES_FACTOR;
  publish_sensor_state_(this->accumulated_angle_sensor_, accumulated_degrees_val);
  publish_sensor_state_(this->accumulated_radians_sensor_, static_cast<float>(this->internal_accumulated_count_) * COUNTS_TO_RADIANS_FACTOR);

  // Calculate display angle (0-360 degrees) with zero offset
  float angle_degrees_display = static_cast<float>(raw_angle_from_sensor) * COUNTS_TO_DEGREES_FACTOR;
  angle_degrees_display -= this->zero_offset_degrees_;
  angle_degrees_display = fmodf(angle_degrees_display, 360.0f); // Normalize to (-360, 360)
  if (angle_degrees_display < 0) {
    angle_degrees_display += 360.0f; // Normalize to [0, 360)
  }
  publish_sensor_state_(this->angle_sensor_, angle_degrees_display);

  // Velocity Calculation
  if (this->velocity_rpm_sensor_ != nullptr) {
    uint32_t now_us = micros();
    uint32_t dt_us = now_us - this->last_velocity_update_time_us_;
    this->last_dt_us_for_velocity_ = dt_us; // Store for EMA filter if used when dt_us is too small

    if (dt_us >= this->min_velocity_update_period_us_) {
      float degrees_moved_for_rpm = static_cast<float>(diff) * COUNTS_TO_DEGREES_FACTOR;
      float raw_velocity_rpm_val = (degrees_moved_for_rpm / static_cast<float>(dt_us)) * DEGREES_PER_US_TO_RPM_FACTOR;

      if (this->direction_inverted_) {
        raw_velocity_rpm_val *= -1.0f;
      }

      float final_velocity_rpm;
      bool filter_enabled = (this->velocity_filter_cutoff_hz_ > 0.0f);

      // Apply selected filter
      if (this->velocity_filter_type_ == VelocityFilterType::BUTTERWORTH_2ND_ORDER && filter_enabled) {
        if (this->first_velocity_sample_) { // Initialize filter states on first sample
          this->x_n1_ = raw_velocity_rpm_val; this->x_n2_ = raw_velocity_rpm_val;
          this->y_n1_ = raw_velocity_rpm_val; this->y_n2_ = raw_velocity_rpm_val;
          this->filtered_velocity_rpm_ = raw_velocity_rpm_val;
        }
        this->filtered_velocity_rpm_ = this->apply_butterworth_filter_(raw_velocity_rpm_val);
        final_velocity_rpm = this->filtered_velocity_rpm_;
      } else if (this->velocity_filter_type_ == VelocityFilterType::EMA && filter_enabled) {
        if (this->first_velocity_sample_) { // Initialize EMA state
          this->filtered_velocity_rpm_ = raw_velocity_rpm_val;
        }
        float dt_s = static_cast<float>(dt_us) / 1000000.0f; // Use current dt_us for EMA alpha calculation
        this->filtered_velocity_rpm_ = this->apply_ema_filter_(raw_velocity_rpm_val, dt_s);
        final_velocity_rpm = this->filtered_velocity_rpm_;
      } else { // No filter or filter disabled
        final_velocity_rpm = raw_velocity_rpm_val;
        if (this->first_velocity_sample_) {
          this->filtered_velocity_rpm_ = raw_velocity_rpm_val; // Still initialize for consistency if dt_us is small next
        }
      }
      
      if (this->first_velocity_sample_) { // Common flag reset after first valid calculation
        this->first_velocity_sample_ = false;
      }

      publish_sensor_state_(this->velocity_rpm_sensor_, final_velocity_rpm);
      this->last_velocity_update_time_us_ = now_us; // Update time for next calculation

    } else if (!this->first_velocity_sample_) { // dt_us too small, publish last filtered value
      publish_sensor_state_(this->velocity_rpm_sensor_, this->filtered_velocity_rpm_);
    } else { // dt_us too small and it's the first sample, publish 0
      publish_sensor_state_(this->velocity_rpm_sensor_, 0.0f);
    }
  }
  this->previous_raw_sensor_value_for_velocity_ = current_processed_value; // Update previous value for next cycle
}

// --- Filter Methods ---
void MT6701SensorComponent::calculate_butterworth_coeffs_() {
  if (this->velocity_filter_cutoff_hz_ <= 0.0f || this->get_update_interval() == 0) {
    ESP_LOGW(TAG, "Cannot calculate Butterworth coeffs: Cutoff freq or update interval is zero. Filter disabled.");
    // Set to pass-through coefficients
    this->b0_ = 1.0f; this->b1_ = 0.0f; this->b2_ = 0.0f;
    this->a1_ = 0.0f; this->a2_ = 0.0f;
    return;
  }

  float sample_rate_hz = 1000.0f / static_cast<float>(this->get_update_interval()); // Sampling rate in Hz
  
  // Formulas from Robert Bristow-Johnson's "Audio EQ Cookbook" for LPF
  // Q is set to 1/sqrt(2) for Butterworth response
  float omega0 = 2.0f * PI * this->velocity_filter_cutoff_hz_ / sample_rate_hz; // Normalized angular frequency
  float cos_omega0 = cosf(omega0);
  float sin_omega0 = sinf(omega0);
  float alpha_q = sin_omega0 / (2.0f * (1.0f / sqrtf(2.0f))); // alpha = sin(w0)/(2*Q)

  float a0_norm_factor = 1.0f + alpha_q; // Normalization factor for a0

  // Numerator coefficients (b)
  this->b0_ = (1.0f - cos_omega0) / 2.0f;
  this->b1_ = 1.0f - cos_omega0;
  this->b2_ = (1.0f - cos_omega0) / 2.0f;
  
  // Denominator coefficients (a) for D(z) = a0_norm_factor + a1_raw*z^-1 + a2_raw*z^-2
  // After normalization by a0_norm_factor: D(z) = 1 + a1*z^-1 + a2*z^-2
  // where a1 = a1_raw / a0_norm_factor, a2 = a2_raw / a0_norm_factor
  // this->a0_ is implicitly 1.0 after normalization.
  this->a1_ = (-2.0f * cos_omega0);
  this->a2_ = (1.0f - alpha_q);

  // Normalize all coefficients by a0_norm_factor
  this->b0_ /= a0_norm_factor;
  this->b1_ /= a0_norm_factor;
  this->b2_ /= a0_norm_factor;
  this->a1_ /= a0_norm_factor;
  this->a2_ /= a0_norm_factor;

  ESP_LOGD(TAG, "Butterworth Coeffs (Cookbook): SR:%.2f Hz, Cutoff:%.2f Hz, w0:%.4f, alpha_q:%.4f, a0_norm:%.4f",
           sample_rate_hz, this->velocity_filter_cutoff_hz_, omega0, alpha_q, a0_norm_factor);
  ESP_LOGD(TAG, "  b0:%.6f, b1:%.6f, b2:%.6f", this->b0_, this->b1_, this->b2_);
  ESP_LOGD(TAG, "  a1:%.6f (for y[n] -= a1*y[n-1]), a2:%.6f (for y[n] -= a2*y[n-2])", this->a1_, this->a2_);
}

float MT6701SensorComponent::apply_butterworth_filter_(float raw_velocity) {
  // Difference equation for biquad filter (Direct Form I):
  // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
  // Where H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
  // The cookbook coefficients a1, a2 are already suitable for this form.

  float filtered_val = this->b0_ * raw_velocity +
                       this->b1_ * this->x_n1_ +
                       this->b2_ * this->x_n2_ -
                       this->a1_ * this->y_n1_ - // Note: a1 and a2 from calculation already have the correct sign for this formula
                       this->a2_ * this->y_n2_;

  // Shift values in history buffers
  this->x_n2_ = this->x_n1_;
  this->x_n1_ = raw_velocity;
  this->y_n2_ = this->y_n1_;
  this->y_n1_ = filtered_val;
  
  return filtered_val;
}

float MT6701SensorComponent::apply_ema_filter_(float raw_velocity, float dt_s) {
  // EMA formula: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
  // y[n-1] state is stored in this->filtered_velocity_rpm_
  // this->first_velocity_sample_ is handled outside this function to initialize EMA
  if (dt_s <= 0.0f || this->velocity_filter_cutoff_hz_ <= 0.0f) {
    return raw_velocity; // If alpha cannot be calculated, return raw value
  }

  float rc = 1.0f / (2.0f * PI * this->velocity_filter_cutoff_hz_); // RC time constant
  float alpha = dt_s / (rc + dt_s);
  
  // Clamp alpha (though mathematically it should be 0 <= alpha <= 1 if dt_s and rc are positive)
  alpha = fmaxf(0.0f, fminf(1.0f, alpha)); 

  return alpha * raw_velocity + (1.0f - alpha) * this->filtered_velocity_rpm_;
}

// --- Service Methods ---
void MT6701SensorComponent::on_set_zero_offset() {
  ESP_LOGD(TAG, "Service 'set_zero_offset' called.");
  // Calculate current angle without considering the current offset
  // Use this->current_raw_sensor_value_, which is the last correctly read value
  if (this->status_has_warning()) { // If last read was bad, don't process
      ESP_LOGW(TAG, "Cannot set new offset due to current warning state (likely last sensor read failed).");
      return;
  }

  float current_angle_raw_degrees = static_cast<float>(this->current_raw_sensor_value_) * COUNTS_TO_DEGREES_FACTOR;
  this->zero_offset_degrees_ = current_angle_raw_degrees;
  ESP_LOGCONFIG(TAG, "New zero offset set to: %.2f degrees (based on raw value %u)",
                this->zero_offset_degrees_, this->current_raw_sensor_value_);

  // Force recalculation and publication of angle with new offset,
  // but don't perform a full `update()` to avoid re-reading from sensor.
  // Just re-process `process_angle_data_` with the last known raw value.
  this->process_angle_data_(this->current_raw_sensor_value_);
}

void MT6701SensorComponent::on_reset_crc_error_count() {
  ESP_LOGD(TAG, "Service 'reset_ssi_crc_errors' called.");
  this->ssi_crc_error_count_ = 0;
  publish_sensor_state_(this->ssi_crc_error_sensor_, static_cast<float>(this->ssi_crc_error_count_));
  ESP_LOGCONFIG(TAG, "SSI CRC error count reset to 0.");
}

} // namespace mt6701
} // namespace esphome