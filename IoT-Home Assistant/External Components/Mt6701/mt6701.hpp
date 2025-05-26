#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"       // Required for GPIOPin if used directly
#include "esphome/core/defines.h"   // For PI

#include <vector>
#include <cmath> // For fabsf, fmodf, std::abs

namespace esphome {
namespace mt6701 {

// Enum for interface type
enum class MT6701InterfaceType {
  NONE = 0,
  I2C,
  SSI
};

// Enum for MT6701 I2C Registers
enum class MT6701Register : uint8_t {
  ANGLE_DATA_H = 0x03, // Angle[13:6]
  ANGLE_DATA_L = 0x04, // Angle[5:0] (in bits 7:2 of this register)
};

// Constants
static const char *const TAG = "mt6701";
static const uint16_t MT6701_RESOLUTION_COUNTS = 16384U; // 14-bit resolution
static const uint8_t SSI_FRAME_SIZE_BYTES = 3U;          // 24 bits = 3 bytes (14-bit angle + 4-bit status + 6-bit CRC)
static const float COUNTS_TO_DEGREES_FACTOR = 360.0f / MT6701_RESOLUTION_COUNTS;
static const float COUNTS_TO_RADIANS_FACTOR = (2.0f * PI) / MT6701_RESOLUTION_COUNTS;
static const float DEGREES_PER_US_TO_RPM_FACTOR = (60.0f * 1000000.0f) / 360.0f;
// For CRC-6/ITU G(x)=x^6+x+1. The 0x03 is x+1, as x^6 is implicit in 6-bit CRC algorithms.
static const uint8_t SSI_CRC6_ITU_POLY_REDUCED = 0x03; 


class MT6701SensorComponent : public PollingComponent, public Component, public i2c::I2CDevice,
                              public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH, spi::CLOCK_PHASE_LEADING_EDGE> { // SPI Mode 2
 public:
  MT6701SensorComponent() = default;

  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  void set_interface_i2c() { this->interface_type_ = MT6701InterfaceType::I2C; }
  void set_interface_ssi() { this->interface_type_ = MT6701InterfaceType::SSI; }
  void set_ssi_clock_speed(uint32_t clock_speed) { this->ssi_configured_clock_speed_ = clock_speed; }
  void set_zero_offset(float offset_degrees) { this->zero_offset_degrees_ = offset_degrees; }
  void set_direction_inverted(bool inverted) { this->direction_inverted_ = inverted; }
  void set_velocity_filter_cutoff(float freq_hz) { this->velocity_filter_cutoff_hz_ = freq_hz; }

  void set_angle_sensor(sensor::Sensor *sensor) { this->angle_sensor_ = sensor; }
  void set_accumulated_angle_sensor(sensor::Sensor *sensor) { this->accumulated_angle_sensor_ = sensor; }
  void set_velocity_rpm_sensor(sensor::Sensor *sensor) { this->velocity_rpm_sensor_ = sensor; }
  void set_raw_count_sensor(sensor::Sensor *sensor) { this->raw_count_sensor_ = sensor; }
  void set_raw_radians_sensor(sensor::Sensor *sensor) { this->raw_radians_sensor_ = sensor; }
  void set_accumulated_count_sensor(sensor::Sensor *sensor) { this->accumulated_count_sensor_ = sensor; }
  void set_accumulated_radians_sensor(sensor::Sensor *sensor) { this->accumulated_radians_sensor_ = sensor; }
  
  void set_magnetic_field_status_sensor(sensor::Sensor *sensor) { this->magnetic_field_status_sensor_ = sensor; }
  void set_loss_of_track_status_sensor(sensor::Sensor *sensor) { this->loss_of_track_status_sensor_ = sensor; }
  void set_push_button_ssi_binary_sensor(binary_sensor::BinarySensor *bsensor) { this->push_button_ssi_binary_sensor_ = bsensor; }

 protected:
  bool read_sensor_data_();
  bool read_i2c_angle_data_(uint16_t &raw_angle_out);
  // Zmieniona nazwa ostatniego parametru dla spójności z .cpp
  bool read_ssi_frame_data_(uint16_t &raw_angle_out, uint8_t &status_bits_out, uint8_t &received_crc_out); 
  void process_angle_data_(uint16_t raw_angle_from_sensor);
  void process_ssi_status_bits_(uint8_t status_bits);
  uint8_t calculate_ssi_crc6_itu_(uint32_t data_18bit);
  bool verify_ssi_crc_(uint16_t angle_data, uint8_t status_data, uint8_t received_crc);


  MT6701InterfaceType interface_type_{MT6701InterfaceType::NONE};
  uint32_t ssi_configured_clock_speed_{4000000}; 

  float zero_offset_degrees_{0.0f};
  bool direction_inverted_{false};
  float velocity_filter_cutoff_hz_{0.0f}; 

  sensor::Sensor *angle_sensor_{nullptr};
  sensor::Sensor *accumulated_angle_sensor_{nullptr};
  sensor::Sensor *velocity_rpm_sensor_{nullptr};
  sensor::Sensor *raw_count_sensor_{nullptr};
  sensor::Sensor *raw_radians_sensor_{nullptr};
  sensor::Sensor *accumulated_count_sensor_{nullptr};
  sensor::Sensor *accumulated_radians_sensor_{nullptr};
  sensor::Sensor *magnetic_field_status_sensor_{nullptr};
  sensor::Sensor *loss_of_track_status_sensor_{nullptr};
  binary_sensor::BinarySensor *push_button_ssi_binary_sensor_{nullptr};

  uint16_t current_raw_sensor_value_{0};
  int32_t internal_accumulated_count_{0}; 
  uint16_t previous_raw_sensor_value_for_velocity_{0};
  uint32_t last_velocity_update_time_us_{0};

  float filtered_velocity_rpm_{0.0f};
  bool first_velocity_sample_{true};
};

} // namespace mt6701
} // namespace esphome