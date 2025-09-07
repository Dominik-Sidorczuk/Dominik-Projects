#ifndef MT6701_H
#define MT6701_H

#include "esphome/core/component.h"
#include "esphome/core/hal.h" // Dla esphome::GPIOPin
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#ifdef MT6701_HAS_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/spi/spi.h"
#include <cmath>
#include <vector> 

namespace esphome {
namespace mt6701 {

// Stałe dla MT6701
static constexpr uint16_t MT6701_RESOLUTION_COUNTS_PER_ELECTRICAL_CYCLE = 16384; // Zliczenia na jeden cykl elektryczny sensora

// Liczba cykli elektrycznych sensora na jeden pełny obrót mechaniczny (fizyczny) magnesu.
// Ustawione na 1.0f zakładając standardowy magnes dwubiegunowy (1 obrót fizyczny = 1 cykl elektryczny).
// Jeśli obserwujesz, że 90 stopni fizycznie = 360 stopni elektrycznie (pełen zakres 0-16383),
// to zmień tę wartość na 4.0f.
static constexpr float ELECTRICAL_CYCLES_PER_MECHANICAL_REVOLUTION = 4.0f; 

static constexpr float COUNTS_TO_DEGREES_FACTOR = 360.0f / (static_cast<float>(MT6701_RESOLUTION_COUNTS_PER_ELECTRICAL_CYCLE) * ELECTRICAL_CYCLES_PER_MECHANICAL_REVOLUTION);
static constexpr float COUNTS_TO_RADIANS_FACTOR = (2.0f * M_PI) / (static_cast<float>(MT6701_RESOLUTION_COUNTS_PER_ELECTRICAL_CYCLE) * ELECTRICAL_CYCLES_PER_MECHANICAL_REVOLUTION);

// Stałe specyficzne dla interfejsu SSI
static constexpr uint8_t SSI_FRAME_SIZE_BYTES = 3;
static constexpr uint16_t SSI_RAW_ANGLE_MASK = 0x3FFF; 
static constexpr uint8_t SSI_STATUS_MASK = 0x0F;
static constexpr uint8_t SSI_CRC_MASK = 0x3F;
static constexpr uint8_t SSI_CRC6_ITU_POLY_REDUCED = 0x03;
static constexpr int SSI_ANGLE_DATA_BITS = 14;
static constexpr int SSI_STATUS_DATA_BITS = 4;
static constexpr int SSI_CRC_BITS = 6;

// Domyślne wartości dla nowych parametrów konfiguracyjnych
static constexpr float DEFAULT_ANGLE_EMA_ALPHA = 0.5f; 
static constexpr float DEFAULT_ANGLE_FILTER_CUTOFF_HZ = 20.0f; 
static constexpr int DEFAULT_RPM_ACCUMULATION_SAMPLES = 5; 

enum class MT6701Register : uint8_t {
  ANGLE_DATA_H = 0x03,
  ANGLE_DATA_L = 0x04,
};

enum class MT6701InterfaceType {
  NONE = 0,
  I2C = 1,
  SSI = 2,
};

enum class FilterType {
  NONE = 0,
  EMA = 1,
  BUTTERWORTH_2ND_ORDER = 2,
};

class MT6701SensorComponent : public PollingComponent,
                              public i2c::I2CDevice,
                              public esphome::spi::SPIDevice<
                                  esphome::spi::BIT_ORDER_MSB_FIRST,
                                  esphome::spi::CLOCK_POLARITY_LOW,
                                  esphome::spi::CLOCK_PHASE_TRAILING,
                                  esphome::spi::DATA_RATE_4MHZ
                              > {
 public:
  MT6701SensorComponent() :
    PollingComponent(),
    i2c::I2CDevice(),
    esphome::spi::SPIDevice<
        esphome::spi::BIT_ORDER_MSB_FIRST,
        esphome::spi::CLOCK_POLARITY_LOW,
        esphome::spi::CLOCK_PHASE_TRAILING,
        esphome::spi::DATA_RATE_4MHZ
    >() {}

   MT6701SensorComponent(esphome::spi::SPIComponent *parent_spi_bus, esphome::GPIOPin *cs_pin_obj) :
     PollingComponent(),
     i2c::I2CDevice(),
     esphome::spi::SPIDevice<
         esphome::spi::BIT_ORDER_MSB_FIRST,
         esphome::spi::CLOCK_POLARITY_LOW,
         esphome::spi::CLOCK_PHASE_TRAILING,
         esphome::spi::DATA_RATE_4MHZ
     >() {
     this->set_spi_parent(parent_spi_bus);
     this->set_cs_pin(cs_pin_obj);
   }

  void setup() override;
  void dump_config() override;
  void update() override;

  void set_interface_i2c() { this->interface_type_ = MT6701InterfaceType::I2C; ESP_LOGD("mt6701", "Interface set to I2C");}
  void set_interface_ssi() { this->interface_type_ = MT6701InterfaceType::SSI; ESP_LOGD("mt6701", "Interface set to SSI");}
  void set_zero_offset_degrees(float offset) { this->zero_offset_degrees_ = offset; }
  void set_direction_inverted(bool inverted) { this->direction_inverted_ = inverted; }

  void set_angle_filter_type(FilterType type) { this->angle_filter_type_ = type; }
  void set_angle_filter_cutoff_hz(float freq) { this->angle_filter_cutoff_hz_ = freq; }
  void set_angle_ema_alpha(float alpha) { this->angle_ema_alpha_ = alpha; }

  void set_velocity_filter_type(FilterType type) { this->velocity_filter_type_ = type; }
  void set_velocity_filter_cutoff_hz(float freq) { this->velocity_filter_cutoff_hz_ = freq; }
  void set_rpm_accumulation_samples(int samples) { this->rpm_accumulation_samples_ = samples; }
  void set_min_velocity_update_period_us(uint32_t period) { this->min_velocity_update_period_us_ = period; }


  void set_angle_sensor(sensor::Sensor *s) { this->angle_sensor_ = s; }
  void set_accumulated_angle_sensor(sensor::Sensor *s) { this->accumulated_angle_sensor_ = s; }
  void set_velocity_rpm_sensor(sensor::Sensor *s) { this->velocity_rpm_sensor_ = s; }
  void set_raw_count_sensor(sensor::Sensor *s) { this->raw_count_sensor_ = s; } 
  void set_filtered_angle_count_sensor(sensor::Sensor *s) { this->filtered_angle_count_sensor_ = s; } 
  void set_raw_radians_sensor(sensor::Sensor *s) { this->raw_radians_sensor_ = s; }
  void set_accumulated_count_sensor(sensor::Sensor *s) { this->accumulated_count_sensor_ = s; }
  void set_accumulated_radians_sensor(sensor::Sensor *s) { this->accumulated_radians_sensor_ = s; }
  void set_magnetic_field_status_sensor(sensor::Sensor *s) { this->magnetic_field_status_sensor_ = s; }
  void set_loss_of_track_status_sensor(sensor::Sensor *s) { this->loss_of_track_status_sensor_ = s; }
  void set_ssi_crc_error_sensor(sensor::Sensor *s) { this->ssi_crc_error_sensor_ = s; }
#ifdef MT6701_HAS_BINARY_SENSOR
  void set_push_button_ssi_binary_sensor(binary_sensor::BinarySensor *s) { this->push_button_ssi_binary_sensor_ = s; }
#endif

 protected:
  bool read_sensor_data_();
  bool read_i2c_angle_data_(uint16_t &raw_angle_out);
  bool read_ssi_frame_data_(uint16_t &raw_angle_out, uint8_t &status_bits_out, uint8_t &received_crc_out);
  void process_angle_data_(uint16_t current_raw_angle_value);
  void process_ssi_status_bits_(uint8_t status_bits);
  bool verify_ssi_crc_(uint16_t angle_data, uint8_t status_data, uint8_t received_crc);
  uint8_t calculate_ssi_crc6_itu_(uint32_t data_18bit) const;

  float apply_angle_filter_(float raw_angle);
  void calculate_angle_butterworth_coeffs_();
  float apply_velocity_filter_(float raw_velocity); 
  void calculate_velocity_butterworth_coeffs_(); 

  template <typename T_> void publish_sensor_state_(sensor::Sensor *sensor, T_ state);
#ifdef MT6701_HAS_BINARY_SENSOR
  void publish_sensor_state_(binary_sensor::BinarySensor *sensor, bool state);
#endif
  void publish_all_nan_or_default_();
  void on_set_zero_offset();
  void on_reset_crc_error_count();

  MT6701InterfaceType interface_type_{MT6701InterfaceType::NONE};
  float zero_offset_degrees_{0.0f};
  bool direction_inverted_{false};

  FilterType angle_filter_type_{FilterType::EMA}; 
  float angle_filter_cutoff_hz_{DEFAULT_ANGLE_FILTER_CUTOFF_HZ};
  float angle_ema_alpha_{DEFAULT_ANGLE_EMA_ALPHA};

  FilterType velocity_filter_type_{FilterType::EMA}; 
  float velocity_filter_cutoff_hz_{10.0f}; 
  int rpm_accumulation_samples_{DEFAULT_RPM_ACCUMULATION_SAMPLES};
  uint32_t min_velocity_update_period_us_{1000}; 

  sensor::Sensor *angle_sensor_{nullptr};
  sensor::Sensor *accumulated_angle_sensor_{nullptr};
  sensor::Sensor *velocity_rpm_sensor_{nullptr};
  sensor::Sensor *raw_count_sensor_{nullptr};
  sensor::Sensor *filtered_angle_count_sensor_{nullptr}; 
  sensor::Sensor *raw_radians_sensor_{nullptr};
  sensor::Sensor *accumulated_count_sensor_{nullptr};
  sensor::Sensor *accumulated_radians_sensor_{nullptr};
  sensor::Sensor *magnetic_field_status_sensor_{nullptr};
  sensor::Sensor *loss_of_track_status_sensor_{nullptr};
  sensor::Sensor *ssi_crc_error_sensor_{nullptr};
#ifdef MT6701_HAS_BINARY_SENSOR
  binary_sensor::BinarySensor *push_button_ssi_binary_sensor_{nullptr};
#endif

  uint16_t current_raw_angle_value_{0}; 
  float filtered_angle_value_counts_{0.0f}; 
  uint16_t previous_filtered_angle_counts_{0}; 
  uint16_t previous_raw_angle_value_for_accum_{0}; // <<< DODANA DEKLARACJA
  bool first_angle_sample_{true}; 

  int32_t internal_accumulated_count_{0}; // Akumuluje zliczenia *elektryczne*

  uint32_t last_sample_time_us_{0}; 
 
  int32_t rpm_accumulated_diff_sum_{0};
  uint32_t rpm_accumulated_dt_us_sum_{0};
  int rpm_sample_counter_{0};
  bool first_rpm_calculation_{true}; 
  float current_filtered_rpm_{0.0f}; 

  float angle_b0_{1.0f}, angle_b1_{0.0f}, angle_b2_{0.0f};
  float angle_a1_{0.0f}, angle_a2_{0.0f};
  float angle_x_n1_{0.0f}, angle_x_n2_{0.0f};
  float angle_y_n1_{0.0f}, angle_y_n2_{0.0f};

  float velocity_b0_{1.0f}, velocity_b1_{0.0f}, velocity_b2_{0.0f};
  float velocity_a1_{0.0f}, velocity_a2_{0.0f};
  float velocity_x_n1_{0.0f}, velocity_x_n2_{0.0f};
  float velocity_y_n1_{0.0f}, velocity_y_n2_{0.0f};
 
  uint32_t ssi_crc_error_count_{0};
};

template <typename T_>
void MT6701SensorComponent::publish_sensor_state_(sensor::Sensor *sensor, T_ state) {
  if (sensor != nullptr) {
    sensor->publish_state(state);
  }
}

} // namespace mt6701
} // namespace esphome

#endif // MT6701_H