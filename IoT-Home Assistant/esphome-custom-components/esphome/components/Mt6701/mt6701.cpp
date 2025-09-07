#include "mt6701.h" // Plik nagłówkowy komponentu
#include "esphome/core/log.h"
#include "esphome/core/helpers.h" // Dla YESNO
#include "esphome/core/hal.h"     // Dla micros()
#include "esphome/components/i2c/i2c.h" // Dla i2c::ERROR_OK
#include "esphome/components/spi/spi.h"  // Dla funkcjonalności SPIDevice
#include <cmath>                      // Dla fmodf, fabsf, M_PI itp.

namespace esphome {
namespace mt6701 {

static const char *const TAG = "mt6701"; // Tag do logowania

// Współczynnik konwersji różnicy zliczeń i dt_us na RPM.
// Ta stała jest obliczana na podstawie wartości z pliku nagłówkowego:
// MT6701_RESOLUTION_COUNTS_PER_ELECTRICAL_CYCLE (np. 16384)
// oraz ELECTRICAL_CYCLES_PER_MECHANICAL_REVOLUTION (w Twoim przypadku 4.0f).
// RPM = (zmiana_licznika_elektrycznego / czas_us) * (60 * 1000000) / (liczniki_na_cykl_elektr * cykle_elektr_na_obrot_mech)
static const float COUNTS_DIFF_PER_US_TO_RPM_FACTOR =
    (60.0f * 1000000.0f) /
    (static_cast<float>(MT6701_RESOLUTION_COUNTS_PER_ELECTRICAL_CYCLE) * ELECTRICAL_CYCLES_PER_MECHANICAL_REVOLUTION);

void MT6701SensorComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MT6701 Sensor Component...");
  ESP_LOGCONFIG(TAG, "Electrical cycles per mechanical revolution: %.1f", ELECTRICAL_CYCLES_PER_MECHANICAL_REVOLUTION);
  ESP_LOGCONFIG(TAG, "Counts to Degrees Factor: %f", COUNTS_TO_DEGREES_FACTOR);
  ESP_LOGCONFIG(TAG, "Counts to Radians Factor: %f", COUNTS_TO_RADIANS_FACTOR);
  ESP_LOGCONFIG(TAG, "Counts Diff per us to RPM Factor: %f", COUNTS_DIFF_PER_US_TO_RPM_FACTOR);

  if (this->interface_type_ == MT6701InterfaceType::NONE) {
    ESP_LOGE(TAG, "Interface type not set! Aborting setup.");
    this->mark_failed();
    return;
  }

  if (this->interface_type_ == MT6701InterfaceType::I2C) {
    ESP_LOGCONFIG(TAG, "Interface: I2C (Address: 0x%02X)", this->address_);
    uint8_t temp_buffer[2];
    ESP_LOGD(TAG, "Attempting I2C communication test read (2 bytes from ANGLE_DATA_H)...");
    if (this->read_register(static_cast<uint8_t>(MT6701Register::ANGLE_DATA_H), temp_buffer, 2, true) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "I2C communication test failed during setup. Check address, connections, and pull-up resistors.");
      this->mark_failed();
      return;
    }
    ESP_LOGD(TAG, "I2C communication test successful (read two bytes). Values: 0x%02X 0x%02X", temp_buffer[0], temp_buffer[1]);
  } else if (this->interface_type_ == MT6701InterfaceType::SSI) {
    ESP_LOGCONFIG(TAG, "Interface: SSI (SPI)");
    if (this->parent_ == nullptr || this->cs_ == nullptr) {
        ESP_LOGE(TAG, "SPI bus (parent_) or CS pin (cs_) not properly configured for SSI interface!");
        this->mark_failed();
        return;
    }
    ESP_LOGD(TAG, "Calling spi_setup() for SSI interface...");
    this->spi_setup();
  }

  this->last_sample_time_us_ = micros();
  this->first_angle_sample_ = true; // Używane do inicjalizacji filtrów i previous_raw_angle_value_for_accum_
  this->first_rpm_calculation_ = true;
  this->current_filtered_rpm_ = 0.0f;

  this->angle_x_n1_ = 0.0f; this->angle_x_n2_ = 0.0f;
  this->angle_y_n1_ = 0.0f; this->angle_y_n2_ = 0.0f;
  this->velocity_x_n1_ = 0.0f; this->velocity_x_n2_ = 0.0f;
  this->velocity_y_n1_ = 0.0f; this->velocity_y_n2_ = 0.0f;

  this->ssi_crc_error_count_ = 0;
  this->rpm_accumulated_diff_sum_ = 0;
  this->rpm_accumulated_dt_us_sum_ = 0;
  this->rpm_sample_counter_ = 0;

  if (this->angle_filter_type_ == FilterType::BUTTERWORTH_2ND_ORDER && this->angle_filter_cutoff_hz_ > 0.0f) {
    this->calculate_angle_butterworth_coeffs_();
  }
  if (this->velocity_filter_type_ == FilterType::BUTTERWORTH_2ND_ORDER && this->velocity_filter_cutoff_hz_ > 0.0f) {
    this->calculate_velocity_butterworth_coeffs_();
  }

  ESP_LOGD(TAG, "Attempting initial full sensor data read...");
  if (this->read_sensor_data_()) {
    // Inicjalizacja wartości przefiltrowanej (dla sensora filtered_angle_count_sensor_)
    this->filtered_angle_value_counts_ = this->apply_angle_filter_(static_cast<float>(this->current_raw_angle_value_));
    // Inicjalizacja wartości poprzedniej surowej dla logiki akumulacji
    this->previous_raw_angle_value_for_accum_ = this->current_raw_angle_value_;
    // Inicjalizacja poprzedniej wartości przefiltrowanej (jeśli jakiś filtr tego potrzebuje, choć obecne nie)
    this->previous_filtered_angle_counts_ = static_cast<uint16_t>(roundf(this->filtered_angle_value_counts_));

    this->first_angle_sample_ = false; // Oznacz, że inicjalizacja została wykonana

    this->internal_accumulated_count_ = 0;
    ESP_LOGD(TAG, "Initial raw sensor value: %u. Filtered angle (for sensor) initialized to: %.2f. Accumulator zeroed. Prev raw for accum: %u",
             this->current_raw_angle_value_, this->filtered_angle_value_counts_, this->previous_raw_angle_value_for_accum_);
  } else {
    ESP_LOGE(TAG, "Failed to read initial sensor value during setup. Component marked as FAILED.");
    this->mark_failed();
    return;
  }
  ESP_LOGCONFIG(TAG, "MT6701 setup finished.");
}

void MT6701SensorComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "MT6701 Sensor Component Configuration:");
  LOG_UPDATE_INTERVAL(this);

  if (this->interface_type_ == MT6701InterfaceType::I2C) {
    ESP_LOGCONFIG(TAG, "  Interface: I2C");
    LOG_I2C_DEVICE(this);
  } else if (this->interface_type_ == MT6701InterfaceType::SSI) {
    ESP_LOGCONFIG(TAG, "  Interface: SSI (SPI)");
    LOG_PIN("    CS Pin: ", this->cs_);
  }

  ESP_LOGCONFIG(TAG, "  Zero Offset: %.2f degrees", this->zero_offset_degrees_);
  ESP_LOGCONFIG(TAG, "  Direction Inverted: %s", YESNO(this->direction_inverted_));
  ESP_LOGCONFIG(TAG, "  Electrical Cycles per Mechanical Revolution: %.1f", ELECTRICAL_CYCLES_PER_MECHANICAL_REVOLUTION);

  const char *angle_filter_type_str = "UNKNOWN";
  switch (this->angle_filter_type_) {
    case FilterType::NONE: angle_filter_type_str = "None"; break;
    case FilterType::EMA: angle_filter_type_str = "EMA"; break;
    case FilterType::BUTTERWORTH_2ND_ORDER: angle_filter_type_str = "Butterworth (2nd order)"; break;
  }
  ESP_LOGCONFIG(TAG, "  Angle Filter Type (for filtered_angle_count_sensor): %s", angle_filter_type_str);
  if (this->angle_filter_type_ == FilterType::EMA) {
    ESP_LOGCONFIG(TAG, "    Angle EMA Alpha: %.2f", this->angle_ema_alpha_);
  } else if (this->angle_filter_type_ == FilterType::BUTTERWORTH_2ND_ORDER) {
    ESP_LOGCONFIG(TAG, "    Angle Filter Cutoff Frequency: %.2f Hz", this->angle_filter_cutoff_hz_);
  }

  ESP_LOGCONFIG(TAG, "  RPM Accumulation Samples: %d", this->rpm_accumulation_samples_);
  ESP_LOGCONFIG(TAG, "  Min Velocity Update Period (per sample): %u us", this->min_velocity_update_period_us_);

  const char *vel_filter_type_str = "UNKNOWN";
  switch (this->velocity_filter_type_) {
    case FilterType::NONE: vel_filter_type_str = "None"; break;
    case FilterType::EMA: vel_filter_type_str = "EMA"; break;
    case FilterType::BUTTERWORTH_2ND_ORDER: vel_filter_type_str = "Butterworth (2nd order)"; break;
  }
  ESP_LOGCONFIG(TAG, "  Velocity Filter Type (final RPM): %s", vel_filter_type_str);
  ESP_LOGCONFIG(TAG, "  Velocity Filter Cutoff Frequency: %.2f Hz (Filter: %s)",
                this->velocity_filter_cutoff_hz_,
                (this->velocity_filter_cutoff_hz_ > 0.0f && this->velocity_filter_type_ != FilterType::NONE ? "Active" : "Inactive"));

  LOG_SENSOR("  ", "Angle Sensor (Main, 0-360 deg mechanical)", this->angle_sensor_);
  LOG_SENSOR("  ", "Accumulated Angle Sensor (Total mechanical deg)", this->accumulated_angle_sensor_);
  LOG_SENSOR("  ", "Velocity RPM Sensor (Mechanical RPM)", this->velocity_rpm_sensor_);
  LOG_SENSOR("  ", "Raw Count Sensor (Electrical counts, 0-16383)", this->raw_count_sensor_);
  LOG_SENSOR("  ", "Filtered Angle Count Sensor (Filtered electrical counts - for display/debug)", this->filtered_angle_count_sensor_);
  LOG_SENSOR("  ", "Raw Radians Sensor (Electrical angle, 0-2PI)", this->raw_radians_sensor_);
  LOG_SENSOR("  ", "Accumulated Count Sensor (Total electrical counts)", this->accumulated_count_sensor_);
  LOG_SENSOR("  ", "Accumulated Radians Sensor (Total mechanical radians)", this->accumulated_radians_sensor_);

  if (this->interface_type_ == MT6701InterfaceType::SSI) {
    LOG_SENSOR("  ", "Magnetic Field Status Sensor", this->magnetic_field_status_sensor_);
    LOG_SENSOR("  ", "Loss of Track Status Sensor", this->loss_of_track_status_sensor_);
#ifdef MT6701_HAS_BINARY_SENSOR
    LOG_BINARY_SENSOR("  ", "Push Button Binary Sensor (SSI)", this->push_button_ssi_binary_sensor_);
#endif
    LOG_SENSOR("  ", "SSI CRC Error Count Sensor", this->ssi_crc_error_sensor_);
  }
}

#ifdef MT6701_HAS_BINARY_SENSOR
void MT6701SensorComponent::publish_sensor_state_(binary_sensor::BinarySensor *sensor, bool state) {
  if (sensor != nullptr) {
    sensor->publish_state(state);
  }
}
#endif

void MT6701SensorComponent::publish_all_nan_or_default_() {
  if (this->angle_sensor_ != nullptr) publish_sensor_state_(this->angle_sensor_, NAN);
  if (this->accumulated_angle_sensor_ != nullptr) publish_sensor_state_(this->accumulated_angle_sensor_, NAN);
  if (this->velocity_rpm_sensor_ != nullptr) publish_sensor_state_(this->velocity_rpm_sensor_, NAN);
  if (this->raw_count_sensor_ != nullptr) publish_sensor_state_(this->raw_count_sensor_, NAN);
  if (this->filtered_angle_count_sensor_ != nullptr) publish_sensor_state_(this->filtered_angle_count_sensor_, NAN);
  if (this->raw_radians_sensor_ != nullptr) publish_sensor_state_(this->raw_radians_sensor_, NAN);
  if (this->accumulated_count_sensor_ != nullptr) publish_sensor_state_(this->accumulated_count_sensor_, NAN);
  if (this->accumulated_radians_sensor_ != nullptr) publish_sensor_state_(this->accumulated_radians_sensor_, NAN);

  if (this->interface_type_ == MT6701InterfaceType::SSI) {
    if (this->magnetic_field_status_sensor_ != nullptr) publish_sensor_state_(this->magnetic_field_status_sensor_, NAN);
    if (this->loss_of_track_status_sensor_ != nullptr) publish_sensor_state_(this->loss_of_track_status_sensor_, NAN);
#ifdef MT6701_HAS_BINARY_SENSOR
    if (this->push_button_ssi_binary_sensor_ != nullptr) publish_sensor_state_(this->push_button_ssi_binary_sensor_, false);
#endif
    if (this->ssi_crc_error_sensor_ != nullptr) {
        publish_sensor_state_(this->ssi_crc_error_sensor_, static_cast<float>(this->ssi_crc_error_count_));
    }
  }
}

void MT6701SensorComponent::update() {
  ESP_LOGV(TAG, "Update cycle started.");
  if (!read_sensor_data_()) {
    ESP_LOGW(TAG, "Failed to read data from MT6701 in update cycle.");
    publish_all_nan_or_default_();
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();
  process_angle_data_(this->current_raw_angle_value_);
  ESP_LOGV(TAG, "Update cycle finished. OriginalRaw=%u, FilteredForSensor=%.2f",
           this->current_raw_angle_value_, this->filtered_angle_value_counts_);
}

bool MT6701SensorComponent::read_sensor_data_() {
  uint16_t raw_angle_value_temp = 0;
  bool read_success = false;

  if (this->interface_type_ == MT6701InterfaceType::I2C) {
    ESP_LOGV(TAG, "Reading sensor data via I2C...");
    read_success = read_i2c_angle_data_(raw_angle_value_temp);
  } else if (this->interface_type_ == MT6701InterfaceType::SSI) {
    ESP_LOGV(TAG, "Reading sensor data via SSI...");
    if (this->parent_ == nullptr || this->cs_ == nullptr) {
        ESP_LOGE(TAG, "Attempting SSI read without configured SPI bus (parent_) or CS pin (cs_).");
        return false;
    }
    uint8_t ssi_status_bits = 0;
    uint8_t received_ssi_crc = 0;
    read_success = read_ssi_frame_data_(raw_angle_value_temp, ssi_status_bits, received_ssi_crc);
    if (read_success) {
      process_ssi_status_bits_(ssi_status_bits);
    }
  } else {
    ESP_LOGE(TAG, "Unknown interface type (%d) in read_sensor_data_", static_cast<int>(this->interface_type_));
    return false;
  }

  if (read_success) {
    this->current_raw_angle_value_ = raw_angle_value_temp;
    ESP_LOGV(TAG, "Sensor data read successfully. Original raw electrical angle value: %u", this->current_raw_angle_value_);
  } else {
    ESP_LOGW(TAG, "Sensor data read failed for interface type %d.", static_cast<int>(this->interface_type_));
  }
  return read_success;
}

bool MT6701SensorComponent::read_i2c_angle_data_(uint16_t &raw_angle_out) {
  uint8_t buffer[2];
  i2c::ErrorCode err = this->read_register(static_cast<uint8_t>(MT6701Register::ANGLE_DATA_H), buffer, 2, true);
  if (err != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C read error from ANGLE_DATA_H/L. Error: %d", static_cast<int>(err));
    return false;
  }
  ESP_LOGVV(TAG, "I2C raw bytes: H=0x%02X, L=0x%02X", buffer[0], buffer[1]);
  raw_angle_out = (static_cast<uint16_t>(buffer[0] & 0x3F) << 8) | buffer[1];
  raw_angle_out &= (MT6701_RESOLUTION_COUNTS_PER_ELECTRICAL_CYCLE - 1);
  ESP_LOGVV(TAG, "I2C processed: raw_angle_out=%u (0x%04X)", raw_angle_out, raw_angle_out);
  return true;
}

bool MT6701SensorComponent::read_ssi_frame_data_(uint16_t &raw_angle_out, uint8_t &status_bits_out, uint8_t &received_crc_out) {
  uint8_t raw_ssi_frame[SSI_FRAME_SIZE_BYTES];
  for (size_t i = 0; i < SSI_FRAME_SIZE_BYTES; ++i) { raw_ssi_frame[i] = 0x00; }
  this->enable();
  this->transfer_array(raw_ssi_frame, SSI_FRAME_SIZE_BYTES);
  this->disable();
  ESP_LOGVV(TAG, "SSI raw frame: 0x%02X 0x%02X 0x%02X", raw_ssi_frame[0], raw_ssi_frame[1], raw_ssi_frame[2]);
  raw_angle_out = (static_cast<uint16_t>(raw_ssi_frame[0]) << 6) | (raw_ssi_frame[1] >> 2);
  raw_angle_out &= SSI_RAW_ANGLE_MASK;
  uint8_t status_S3S2 = raw_ssi_frame[1] & 0x03;
  uint8_t status_S1S0 = (raw_ssi_frame[2] >> 6) & 0x03;
  status_bits_out = (status_S3S2 << 2) | status_S1S0;
  status_bits_out &= SSI_STATUS_MASK;
  received_crc_out = raw_ssi_frame[2] & SSI_CRC_MASK;
  if (!verify_ssi_crc_(raw_angle_out, status_bits_out, received_crc_out)) {
    return false;
  }
  return true;
}

uint8_t MT6701SensorComponent::calculate_ssi_crc6_itu_(uint32_t data_18bit) const {
  uint8_t crc_reg = 0x00;
  const int num_data_bits = SSI_ANGLE_DATA_BITS + SSI_STATUS_DATA_BITS;
  for (int bit_pos = num_data_bits - 1; bit_pos >= 0; bit_pos--) {
    bool data_bit_is_set = (data_18bit >> bit_pos) & 0x01;
    bool crc_msb_is_set = (crc_reg & (1 << (SSI_CRC_BITS - 1))) != 0;
    crc_reg <<= 1;
    if (data_bit_is_set ^ crc_msb_is_set) { crc_reg ^= SSI_CRC6_ITU_POLY_REDUCED; }
  }
  return crc_reg & SSI_CRC_MASK;
}

bool MT6701SensorComponent::verify_ssi_crc_(uint16_t angle_data, uint8_t status_data, uint8_t received_crc) {
  uint32_t data_word_for_crc = (static_cast<uint32_t>(angle_data & SSI_RAW_ANGLE_MASK) << SSI_STATUS_DATA_BITS) |
                               (status_data & SSI_STATUS_MASK);
  uint8_t calculated_crc = calculate_ssi_crc6_itu_(data_word_for_crc);
  if (calculated_crc != received_crc) {
    ESP_LOGW(TAG, "SSI CRC ERROR! Angle=0x%04X, Status=0x%X. CalcCRC=0x%02X, RecvCRC=0x%02X",
             angle_data, status_data, calculated_crc, received_crc);
    this->ssi_crc_error_count_++;
    if (this->ssi_crc_error_sensor_ != nullptr) {
        publish_sensor_state_(this->ssi_crc_error_sensor_, static_cast<float>(this->ssi_crc_error_count_));
    }
    return false;
  }
  ESP_LOGVV(TAG, "SSI CRC OK. Angle=0x%04X, Status=0x%X. CRC=0x%02X", angle_data, status_data, received_crc);
  return true;
}

void MT6701SensorComponent::process_ssi_status_bits_(uint8_t status_bits) {
  if (this->magnetic_field_status_sensor_ != nullptr) {
    publish_sensor_state_(this->magnetic_field_status_sensor_, static_cast<float>((status_bits >> 2) & 0x03));
  }
  if (this->loss_of_track_status_sensor_ != nullptr) {
    publish_sensor_state_(this->loss_of_track_status_sensor_, static_cast<float>((status_bits >> 0) & 0x01));
  }
#ifdef MT6701_HAS_BINARY_SENSOR
  if (this->push_button_ssi_binary_sensor_ != nullptr) {
    publish_sensor_state_(this->push_button_ssi_binary_sensor_, static_cast<bool>((status_bits >> 1) & 0x01));
  }
#endif
}

float MT6701SensorComponent::apply_angle_filter_(float raw_angle_counts) {
    float filtered_value = raw_angle_counts;

    // first_angle_sample_ jest true tylko podczas setup() dla pierwszej inicjalizacji filtru.
    // W kolejnych wywołaniach apply_angle_filter_ (z process_angle_data_), first_angle_sample_ będzie false.
    bool is_initializing_filter = this->first_angle_sample_;


    switch (this->angle_filter_type_) {
        case FilterType::EMA:
            if (is_initializing_filter) {
                this->filtered_angle_value_counts_ = raw_angle_counts;
            } else {
                this->filtered_angle_value_counts_ = this->angle_ema_alpha_ * raw_angle_counts +
                                                     (1.0f - this->angle_ema_alpha_) * this->filtered_angle_value_counts_;
            }
            filtered_value = this->filtered_angle_value_counts_;
            break;
        case FilterType::BUTTERWORTH_2ND_ORDER:
            if (is_initializing_filter) {
                this->angle_x_n1_ = raw_angle_counts; this->angle_x_n2_ = raw_angle_counts;
                this->angle_y_n1_ = raw_angle_counts; this->angle_y_n2_ = raw_angle_counts;
                this->filtered_angle_value_counts_ = raw_angle_counts;
            } else {
                 this->filtered_angle_value_counts_ = this->angle_b0_ * raw_angle_counts +
                                                 this->angle_b1_ * this->angle_x_n1_ +
                                                 this->angle_b2_ * this->angle_x_n2_ -
                                                 this->angle_a1_ * this->angle_y_n1_ -
                                                 this->angle_a2_ * this->angle_y_n2_;
                this->angle_x_n2_ = this->angle_x_n1_;
                this->angle_x_n1_ = raw_angle_counts;
                this->angle_y_n2_ = this->angle_y_n1_;
                this->angle_y_n1_ = this->filtered_angle_value_counts_;
            }
            filtered_value = this->filtered_angle_value_counts_;
            break;
        case FilterType::NONE:
        default:
            this->filtered_angle_value_counts_ = raw_angle_counts;
            filtered_value = raw_angle_counts;
            break;
    }
    return filtered_value;
}


void MT6701SensorComponent::process_angle_data_(uint16_t current_raw_angle_value) {
  // Zastosuj skonfigurowany filtr kąta (EMA, Butterworth, None) do surowej wartości
  // Wynik jest przechowywany w this->filtered_angle_value_counts_ i używany tylko dla sensora filtered_angle_count_sensor_
  float filtered_output_for_sensor = this->apply_angle_filter_(static_cast<float>(current_raw_angle_value));

  ESP_LOGVV(TAG, "AngleProc: Raw=%u, FilteredForSensor=%.2f",
            current_raw_angle_value, filtered_output_for_sensor);

  // Publikowanie surowych wartości kąta elektrycznego
  if (this->raw_count_sensor_ != nullptr) {
    publish_sensor_state_(this->raw_count_sensor_, static_cast<float>(current_raw_angle_value));
  }
  if (this->raw_radians_sensor_ != nullptr) {
    publish_sensor_state_(this->raw_radians_sensor_, static_cast<float>(current_raw_angle_value) * ( (2.0f * M_PI) / static_cast<float>(MT6701_RESOLUTION_COUNTS_PER_ELECTRICAL_CYCLE) ) );
  }
  // Publikowanie przefiltrowanej wartości kąta elektrycznego (jeśli sensor jest zdefiniowany)
  if (this->filtered_angle_count_sensor_ != nullptr) {
      publish_sensor_state_(this->filtered_angle_count_sensor_, filtered_output_for_sensor);
  }

  // Obliczanie różnicy zliczeń na podstawie SUROWYCH wartości dla akumulacji i RPM
  int16_t actual_diff_for_accumulation;
  // Flaga first_angle_sample_ jest ustawiana na false w setup() po pierwszym odczycie i inicjalizacji previous_raw_angle_value_for_accum_
  // Więc w pierwszym wywołaniu process_angle_data_ z update(), diff będzie obliczony poprawnie.
  actual_diff_for_accumulation = static_cast<int16_t>(this->current_raw_angle_value_) - static_cast<int16_t>(this->previous_raw_angle_value_for_accum_);

  // Korekta przejścia przez zero dla surowych wartości
  if (std::abs(actual_diff_for_accumulation) > (MT6701_RESOLUTION_COUNTS_PER_ELECTRICAL_CYCLE / 2)) {
    if (actual_diff_for_accumulation > 0) {
        actual_diff_for_accumulation -= MT6701_RESOLUTION_COUNTS_PER_ELECTRICAL_CYCLE;
    } else {
        actual_diff_for_accumulation += MT6701_RESOLUTION_COUNTS_PER_ELECTRICAL_CYCLE;
    }
  }

  // Akumulacja całkowitej liczby zliczeń elektrycznych
  if (this->direction_inverted_) {
    this->internal_accumulated_count_ -= actual_diff_for_accumulation;
  } else {
    this->internal_accumulated_count_ += actual_diff_for_accumulation;
  }

  // Publikowanie zakumulowanych wartości
  if (this->accumulated_count_sensor_ != nullptr) {
    publish_sensor_state_(this->accumulated_count_sensor_, static_cast<float>(this->internal_accumulated_count_));
  }
  float accumulated_degrees_val = static_cast<float>(this->internal_accumulated_count_) * COUNTS_TO_DEGREES_FACTOR;
  if (this->accumulated_angle_sensor_ != nullptr) {
    publish_sensor_state_(this->accumulated_angle_sensor_, accumulated_degrees_val);
  }
  if (this->accumulated_radians_sensor_ != nullptr) {
    publish_sensor_state_(this->accumulated_radians_sensor_, static_cast<float>(this->internal_accumulated_count_) * COUNTS_TO_RADIANS_FACTOR);
  }

  // Obliczanie i publikowanie kąta mechanicznego w zakresie 0-360 stopni
  ESP_LOGD(TAG, "AngleDisplayCalc: internal_accum_counts=%.0f, zero_offset=%.2f, calculated_accum_deg=%.2f",
           static_cast<float>(this->internal_accumulated_count_), this->zero_offset_degrees_, accumulated_degrees_val);

  float angle_degrees_display = accumulated_degrees_val; // Zaczynamy od zakumulowanego kąta
  angle_degrees_display -= this->zero_offset_degrees_;
  angle_degrees_display = fmodf(angle_degrees_display, 360.0f);
  if (angle_degrees_display < 0) {
    angle_degrees_display += 360.0f;
  }
  if (this->angle_sensor_ != nullptr) {
    publish_sensor_state_(this->angle_sensor_, angle_degrees_display);
  }

  // Obliczanie i publikowanie prędkości RPM
  if (this->velocity_rpm_sensor_ != nullptr) {
    uint32_t now_us = micros();
    uint32_t dt_us_current_sample = now_us - this->last_sample_time_us_;

    if (dt_us_current_sample >= this->min_velocity_update_period_us_) {
      this->rpm_accumulated_diff_sum_ += actual_diff_for_accumulation; // Używamy actual_diff_for_accumulation
      this->rpm_accumulated_dt_us_sum_ += dt_us_current_sample;
      this->rpm_sample_counter_++;

      ESP_LOGVV(TAG, "RPM Accum: diff_sum=%d, dt_sum=%u, count=%d (curr_dt=%u, curr_raw_diff=%d)",
                this->rpm_accumulated_diff_sum_, this->rpm_accumulated_dt_us_sum_, this->rpm_sample_counter_,
                dt_us_current_sample, actual_diff_for_accumulation);

      if (this->rpm_sample_counter_ >= this->rpm_accumulation_samples_) {
        float raw_rpm_value = 0.0f;
        if (this->rpm_accumulated_dt_us_sum_ > 0) {
          raw_rpm_value = (static_cast<float>(this->rpm_accumulated_diff_sum_) / static_cast<float>(this->rpm_accumulated_dt_us_sum_)) * COUNTS_DIFF_PER_US_TO_RPM_FACTOR;
        }
        ESP_LOGD(TAG, "RPM Calculated (raw, before final filter): %.2f RPM from diff_sum=%d, dt_sum=%u us",
                 raw_rpm_value, this->rpm_accumulated_diff_sum_, this->rpm_accumulated_dt_us_sum_);

        if (this->direction_inverted_) {
          raw_rpm_value *= -1.0f;
        }
        
        this->current_filtered_rpm_ = this->apply_velocity_filter_(raw_rpm_value);
        publish_sensor_state_(this->velocity_rpm_sensor_, this->current_filtered_rpm_);
        ESP_LOGD(TAG, "RPM Published (final): %.2f RPM", this->current_filtered_rpm_);
        
        this->rpm_accumulated_diff_sum_ = 0;
        this->rpm_accumulated_dt_us_sum_ = 0;
        this->rpm_sample_counter_ = 0;
        this->first_rpm_calculation_ = false;
      }
    } else if (!this->first_rpm_calculation_ && this->rpm_sample_counter_ < this->rpm_accumulation_samples_) {
        publish_sensor_state_(this->velocity_rpm_sensor_, this->current_filtered_rpm_);
        ESP_LOGVV(TAG, "RPM Published (stale, accumulating): %.2f RPM", this->current_filtered_rpm_);
    } else if (this->first_rpm_calculation_ && this->rpm_sample_counter_ < this->rpm_accumulation_samples_) {
        publish_sensor_state_(this->velocity_rpm_sensor_, 0.0f);
        ESP_LOGVV(TAG, "RPM Published (initial, 0.0 RPM)");
    }
  }
  this->previous_raw_angle_value_for_accum_ = this->current_raw_angle_value_; // Zapisz aktualną SUROWĄ wartość jako poprzednią dla następnej iteracji akumulacji
  this->previous_filtered_angle_counts_ = static_cast<uint16_t>(roundf(filtered_output_for_sensor)); // Zapisz poprzednią przefiltrowaną wartość (dla spójności lub przyszłego użytku)
  this->last_sample_time_us_ = micros();
}

void MT6701SensorComponent::calculate_angle_butterworth_coeffs_() {
    if (this->angle_filter_cutoff_hz_ <= 0.0f || this->get_update_interval() == 0) {
        ESP_LOGW(TAG, "Cannot calculate Angle Butterworth coeffs: Cutoff or interval invalid. Filter disabled.");
        this->angle_b0_ = 1.0f; this->angle_b1_ = 0.0f; this->angle_b2_ = 0.0f;
        this->angle_a1_ = 0.0f; this->angle_a2_ = 0.0f;
        return;
    }
    float sample_rate_hz = 1000.0f / static_cast<float>(this->get_update_interval());
    float nyquist_freq_hz = sample_rate_hz / 2.0f;
    float cutoff_hz = this->angle_filter_cutoff_hz_;
    if (cutoff_hz >= nyquist_freq_hz) {
        ESP_LOGW(TAG, "Angle Filter: Cutoff (%.2fHz) too high for SR (%.2fHz). Limiting to %.2fHz.",
                 cutoff_hz, sample_rate_hz, nyquist_freq_hz * 0.99f);
        cutoff_hz = nyquist_freq_hz * 0.99f;
    }
     if (cutoff_hz <= 0.0f) {
      ESP_LOGW(TAG, "Angle Filter: Cutoff is zero or negative after adjustment. Filter disabled.");
      this->angle_b0_ = 1.0f; this->angle_b1_ = 0.0f; this->angle_b2_ = 0.0f; this->angle_a1_ = 0.0f; this->angle_a2_ = 0.0f;
      return;
    }
    float K = tanf(M_PI * cutoff_hz / sample_rate_hz);
    float K_sq = K * K;
    float norm_factor = 1.0f / (1.0f + sqrtf(2.0f) * K + K_sq);
    this->angle_b0_ = K_sq * norm_factor;
    this->angle_b1_ = 2.0f * this->angle_b0_;
    this->angle_b2_ = this->angle_b0_;
    this->angle_a1_ = 2.0f * (K_sq - 1.0f) * norm_factor;
    this->angle_a2_ = (1.0f - sqrtf(2.0f) * K + K_sq) * norm_factor;
    ESP_LOGD(TAG, "Angle Butterworth Coeffs: SR:%.2fHz, Cutoff:%.2fHz -> b0:%.6f, b1:%.6f, b2:%.6f, a1:%.6f, a2:%.6f",
             sample_rate_hz, cutoff_hz, this->angle_b0_, this->angle_b1_, this->angle_b2_, this->angle_a1_, this->angle_a2_);
}

void MT6701SensorComponent::calculate_velocity_butterworth_coeffs_() {
    if (this->velocity_filter_cutoff_hz_ <= 0.0f) {
        ESP_LOGW(TAG, "Cannot calculate Velocity Butterworth coeffs: Cutoff invalid. Filter disabled.");
        this->velocity_b0_ = 1.0f; this->velocity_b1_ = 0.0f; this->velocity_b2_ = 0.0f;
        this->velocity_a1_ = 0.0f; this->velocity_a2_ = 0.0f;
        return;
    }
    float effective_sample_period_ms = static_cast<float>(this->get_update_interval() * this->rpm_accumulation_samples_);
    if (effective_sample_period_ms == 0) {
        ESP_LOGW(TAG, "Effective sample period for Velocity filter is zero. Filter disabled.");
        this->velocity_b0_ = 1.0f; this->velocity_b1_ = 0.0f; this->velocity_b2_ = 0.0f; this->velocity_a1_ = 0.0f; this->velocity_a2_ = 0.0f;
        return;
    }
    float sample_rate_hz = 1000.0f / effective_sample_period_ms;
    float nyquist_freq_hz = sample_rate_hz / 2.0f;
    float cutoff_hz = this->velocity_filter_cutoff_hz_;

    if (cutoff_hz >= nyquist_freq_hz) {
        ESP_LOGW(TAG, "Velocity Filter: Cutoff (%.2fHz) too high for effective SR (%.2fHz). Limiting to %.2fHz.",
                 cutoff_hz, sample_rate_hz, nyquist_freq_hz * 0.99f);
        cutoff_hz = nyquist_freq_hz * 0.99f;
    }
    if (cutoff_hz <= 0.0f) {
      ESP_LOGW(TAG, "Velocity Filter: Cutoff is zero or negative after adjustment. Filter disabled.");
      this->velocity_b0_ = 1.0f; this->velocity_b1_ = 0.0f; this->velocity_b2_ = 0.0f; this->velocity_a1_ = 0.0f; this->velocity_a2_ = 0.0f;
      return;
    }
    float K = tanf(M_PI * cutoff_hz / sample_rate_hz);
    float K_sq = K * K;
    float norm_factor = 1.0f / (1.0f + sqrtf(2.0f) * K + K_sq);
    this->velocity_b0_ = K_sq * norm_factor;
    this->velocity_b1_ = 2.0f * this->velocity_b0_;
    this->velocity_b2_ = this->velocity_b0_;
    this->velocity_a1_ = 2.0f * (K_sq - 1.0f) * norm_factor;
    this->velocity_a2_ = (1.0f - sqrtf(2.0f) * K + K_sq) * norm_factor;
    ESP_LOGD(TAG, "Velocity Butterworth Coeffs: EffSR:%.2fHz, Cutoff:%.2fHz -> b0:%.6f, b1:%.6f, b2:%.6f, a1:%.6f, a2:%.6f",
             sample_rate_hz, cutoff_hz, this->velocity_b0_, this->velocity_b1_, this->velocity_b2_, this->velocity_a1_, this->velocity_a2_);
}

float MT6701SensorComponent::apply_velocity_filter_(float raw_rpm_value) {
    float filtered_value = raw_rpm_value;
    switch (this->velocity_filter_type_) {
        case FilterType::EMA:
            if (this->first_rpm_calculation_) {
                this->current_filtered_rpm_ = raw_rpm_value;
            } else {
                float effective_dt_s = static_cast<float>(this->get_update_interval() * this->rpm_accumulation_samples_) / 1000000.0f;
                if (this->rpm_accumulated_dt_us_sum_ > 0 && this->rpm_sample_counter_ >= this->rpm_accumulation_samples_) {
                     effective_dt_s = static_cast<float>(this->rpm_accumulated_dt_us_sum_) / 1000000.0f;
                }

                if (effective_dt_s <= 0.0f || this->velocity_filter_cutoff_hz_ <= 0.0f) {
                    this->current_filtered_rpm_ = raw_rpm_value;
                } else {
                    float rc = 1.0f / (2.0f * M_PI * this->velocity_filter_cutoff_hz_);
                    float alpha = effective_dt_s / (rc + effective_dt_s);
                    alpha = fmaxf(0.0f, fminf(1.0f, alpha));
                    this->current_filtered_rpm_ = alpha * raw_rpm_value + (1.0f - alpha) * this->current_filtered_rpm_;
                }
            }
            filtered_value = this->current_filtered_rpm_;
            break;
        case FilterType::BUTTERWORTH_2ND_ORDER:
            if (this->first_rpm_calculation_) {
                this->velocity_x_n1_ = raw_rpm_value; this->velocity_x_n2_ = raw_rpm_value;
                this->velocity_y_n1_ = raw_rpm_value; this->velocity_y_n2_ = raw_rpm_value;
                this->current_filtered_rpm_ = raw_rpm_value;
            } else {
                 this->current_filtered_rpm_ = this->velocity_b0_ * raw_rpm_value +
                                          this->velocity_b1_ * this->velocity_x_n1_ +
                                          this->velocity_b2_ * this->velocity_x_n2_ -
                                          this->velocity_a1_ * this->velocity_y_n1_ -
                                          this->velocity_a2_ * this->velocity_y_n2_;
                this->velocity_x_n2_ = this->velocity_x_n1_;
                this->velocity_x_n1_ = raw_rpm_value;
                this->velocity_y_n2_ = this->velocity_y_n1_;
                this->velocity_y_n1_ = this->current_filtered_rpm_;
            }
            filtered_value = this->current_filtered_rpm_;
            break;
        case FilterType::NONE:
        default:
            this->current_filtered_rpm_ = raw_rpm_value;
            filtered_value = raw_rpm_value;
            break;
    }
    return filtered_value;
}

void MT6701SensorComponent::on_set_zero_offset() {
  ESP_LOGD(TAG, "'set_zero_offset' service called.");
  if (this->status_has_warning()) {
    ESP_LOGW(TAG, "Cannot set new offset: Sensor is in warning state (last read likely failed).");
    return;
  }
  float total_physical_angle_degrees = static_cast<float>(this->internal_accumulated_count_) * COUNTS_TO_DEGREES_FACTOR;
  float current_display_angle_for_offset = fmodf(total_physical_angle_degrees, 360.0f);
    if (current_display_angle_for_offset < 0) {
        current_display_angle_for_offset += 360.0f;
    }
  this->zero_offset_degrees_ = current_display_angle_for_offset;
  ESP_LOGCONFIG(TAG, "New zero offset set to: %.2f degrees (based on total physical angle)", this->zero_offset_degrees_);
  
  this->process_angle_data_(this->current_raw_angle_value_);
}

void MT6701SensorComponent::on_reset_crc_error_count() {
  ESP_LOGD(TAG, "'reset_ssi_crc_errors' service called.");
  this->ssi_crc_error_count_ = 0;
  if (this->ssi_crc_error_sensor_ != nullptr) {
    publish_sensor_state_(this->ssi_crc_error_sensor_, static_cast<float>(this->ssi_crc_error_count_));
  }
  ESP_LOGCONFIG(TAG, "SSI CRC error counter reset to 0.");
}

} // namespace mt6701
} // namespace esphome