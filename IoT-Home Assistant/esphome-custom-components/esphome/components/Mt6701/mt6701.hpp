#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"     // For PI

#include <string>
#include <cmath>  // For fabsf, fmodf, std::abs, cosf, sinf, sqrtf

namespace esphome {
namespace mt6701 {

// --- MT6701 Chip-Specific Constants ---
static constexpr uint16_t MT6701_RESOLUTION_COUNTS = 1 << 14; // 14-bit resolution
static constexpr float COUNTS_PER_REVOLUTION = static_cast<float>(MT6701_RESOLUTION_COUNTS);
static constexpr float COUNTS_TO_DEGREES_FACTOR = 360.0f / COUNTS_PER_REVOLUTION;
static constexpr float COUNTS_TO_RADIANS_FACTOR = (2.0f * PI) / COUNTS_PER_REVOLUTION;
static constexpr float DEGREES_PER_US_TO_RPM_FACTOR = (1.0f / 360.0f) * 60.0f * 1000000.0f;

// Constants for SSI Interface
static constexpr uint8_t SSI_ANGLE_DATA_BITS = 14;
static constexpr uint8_t SSI_STATUS_DATA_BITS = 4;
static constexpr uint8_t SSI_CRC_BITS = 6;
static constexpr uint8_t SSI_TOTAL_FRAME_BITS = SSI_ANGLE_DATA_BITS + SSI_STATUS_DATA_BITS + SSI_CRC_BITS; // 24 bits
static constexpr uint8_t SSI_FRAME_SIZE_BYTES = (SSI_TOTAL_FRAME_BITS + 7) / 8; // 3 bytes
static constexpr uint8_t SSI_CRC6_ITU_POLY_REDUCED = 0x03; // G(x)=x^6+x^1. Verify with datasheet.
static constexpr uint16_t SSI_RAW_ANGLE_MASK = (1 << SSI_ANGLE_DATA_BITS) - 1; // 0x3FFF
static constexpr uint8_t SSI_STATUS_MASK = (1 << SSI_STATUS_DATA_BITS) - 1;    // 0x0F
static constexpr uint8_t SSI_CRC_MASK = (1 << SSI_CRC_BITS) - 1;          // 0x3F

enum class MT6701InterfaceType { NONE = 0, I2C, SSI };
enum class VelocityFilterType { NONE = 0, EMA, BUTTERWORTH_2ND_ORDER };
enum class MT6701Register : uint8_t { ANGLE_DATA_H = 0x03, ANGLE_DATA_L = 0x04 };

// SPI Mode 0: CPOL=0 (SCLK idle LOW), CPHA=0 (sample on leading/first edge)
// Based on user-provided reference code.
class MT6701SensorComponent : public PollingComponent, public Component, public i2c::I2CDevice,
                              public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                                    spi::CLOCK_POLARITY_LOW,       // CPOL = 0
                                                    spi::CLOCK_PHASE_LEADING_EDGE> { // CPHA = 0
 public:
  MT6701SensorComponent() = default;

  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  // Configuration setters
  void set_interface_i2c() { this->interface_type_ = MT6701InterfaceType::I2C; }
  void set_interface_ssi() { this->interface_type_ = MT6701InterfaceType::SSI; }
  void set_zero_offset_degrees(float offset_degrees) { this->zero_offset_degrees_ = offset_degrees; }
  void set_direction_inverted(bool inverted) { this->direction_inverted_ = inverted; }
  void set_velocity_filter_type_str(const std::string &type_str);
  void set_velocity_filter_cutoff_hz(float freq_hz) { this->velocity_filter_cutoff_hz_ = freq_hz; }
  void set_min_velocity_update_period_us(uint32_t period_us) { this->min_velocity_update_period_us_ = period_us; }

  // Sensor setters
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
  void set_ssi_crc_error_sensor(sensor::Sensor *sensor) { this->ssi_crc_error_sensor_ = sensor; }

  // Service call handlers
  void on_set_zero_offset();
  void on_reset_crc_error_count();

 protected:
  // Data reading methods
  bool read_sensor_data_();
  bool read_i2c_angle_data_(uint16_t &raw_angle_out);
  bool read_ssi_frame_data_(uint16_t &raw_angle_out, uint8_t &status_bits_out, uint8_t &received_crc_out);
  
  // Data processing methods
  void process_angle_data_(uint16_t raw_angle_from_sensor);
  void process_ssi_status_bits_(uint8_t status_bits);
  
  // CRC methods
  uint8_t calculate_ssi_crc6_itu_(uint32_t data_18bit) const;
  bool verify_ssi_crc_(uint16_t angle_data, uint8_t status_data, uint8_t received_crc);

  // Filter methods
  void calculate_butterworth_coeffs_();
  float apply_butterworth_filter_(float raw_velocity);
  float apply_ema_filter_(float raw_velocity, float dt_s);

  // Helper methods for publishing sensor states
  template <typename T>
  void publish_sensor_state_(esphome::sensor::Sensor *sensor, T state);
  void publish_sensor_state_(esphome::binary_sensor::BinarySensor *sensor, bool state);
  void publish_all_nan_or_default_();

  // Member variables
  MT6701InterfaceType interface_type_{MT6701InterfaceType::NONE};
  float zero_offset_degrees_{0.0f};
  bool direction_inverted_{false};
  
  VelocityFilterType velocity_filter_type_{VelocityFilterType::EMA}; // Default to EMA
  float velocity_filter_cutoff_hz_{0.0f}; // 0.0f means filter is disabled or uses raw value
  uint32_t min_velocity_update_period_us_{1000}; // Default 1ms

  // Pointers to sensor objects
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
  sensor::Sensor *ssi_crc_error_sensor_{nullptr};

  // Internal state variables
  uint16_t current_raw_sensor_value_{0}; // Last successfully read raw angle value
  int32_t internal_accumulated_count_{0}; // Accumulated count for multi-turn tracking
  uint16_t previous_raw_sensor_value_for_velocity_{0}; // Previous raw value for velocity calculation
  uint32_t last_velocity_update_time_us_{0}; // Timestamp of the last velocity calculation

  // Filter states
  float filtered_velocity_rpm_{0.0f}; // Current filtered velocity value
  bool first_velocity_sample_{true};  // Flag for initializing filters/velocity
  // Butterworth filter coefficients
  float b0_{1.0f}, b1_{0.0f}, b2_{0.0f}; // Numerator coeffs (initialized for pass-through)
  float a1_{0.0f}, a2_{0.0f};          // Denominator coeffs (for 1 + a1*z^-1 + a2*z^-2)
  // Butterworth history for input (x) and output (y)
  float x_n1_{0.0f}, x_n2_{0.0f};
  float y_n1_{0.0f}, y_n2_{0.0f};

  uint32_t ssi_crc_error_count_{0}; // Counter for SSI CRC errors
};

} // namespace mt6701
} // namespace esphome
