#include "mt6701.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h" // For YESNO

namespace esphome {
namespace mt6701 {

// TAG jest zdefiniowany w .h

void MT6701SensorComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up MT6701 Sensor Component...");
    
    if (this->interface_type_ == MT6701InterfaceType::NONE) {
        ESP_LOGE(TAG, "Interface type not set! Aborting setup.");
        this->mark_failed();
        return;
    }

    if (this->interface_type_ == MT6701InterfaceType::I2C) {
        ESP_LOGCONFIG(TAG, "Interface: I2C (Address: 0x%02X)", this->address_);
        uint8_t temp_val;
        // Użycie this->read_register z domyślnym stop=true
        if (this->read_register(static_cast<uint8_t>(MT6701Register::ANGLE_DATA_H), &temp_val, 1) != i2c::ERROR_OK) {
            ESP_LOGE(TAG, "I2C communication test failed at ANGLE_DATA_H. Check address and wiring.");
            this->mark_failed();
            return;
        }
        ESP_LOGD(TAG, "I2C communication test successful.");
    } else if (this->interface_type_ == MT6701InterfaceType::SSI) {
        ESP_LOGCONFIG(TAG, "Interface: SSI (SPI)");
        ESP_LOGCONFIG(TAG, "  SPI Mode (from template): %d (CPOL=%d, CPHA=%d)", 
                      static_cast<int>(this->mode), 
                      (this->mode & 0b10 ? 1:0), 
                      (this->mode & 0b01 ? 1:0)
                      ); 
        ESP_LOGCONFIG(TAG, "  SPI Target Speed: %u Hz", this->ssi_configured_clock_speed_);
        // Rzeczywista prędkość magistrali `this->frequency_` będzie dostępna po konfiguracji magistrali.
        // Można ją zalogować w dump_config() lub po pierwszym użyciu this->frequency_.
    }

    this->last_velocity_update_time_us_ = micros();
    this->first_velocity_sample_ = true;
    this->filtered_velocity_rpm_ = 0.0f;

    uint16_t initial_raw_angle_value = 0;
    bool initial_read_ok = false;

    if (this->interface_type_ == MT6701InterfaceType::I2C) {
        initial_read_ok = read_i2c_angle_data_(initial_raw_angle_value);
    } else if (this->interface_type_ == MT6701InterfaceType::SSI) {
        uint8_t status_bits, crc;  
        initial_read_ok = read_ssi_frame_data_(initial_raw_angle_value, status_bits, crc);
    }

    if (initial_read_ok) {
        this->current_raw_sensor_value_ = initial_raw_angle_value;
        this->internal_accumulated_count_ = 0;  
        this->previous_raw_sensor_value_for_velocity_ = initial_raw_angle_value;  
        ESP_LOGD(TAG, "Initial raw sensor value: %u. Accumulator zeroed.", this->current_raw_sensor_value_);
    } else {
        ESP_LOGE(TAG, "Failed to read initial sensor value during setup. Component marked as failed.");
        this->mark_failed();
        return;
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
        ESP_LOGCONFIG(TAG, "  SPI Mode (from template): %d (CPOL=%d, CPHA=%d)", 
                      static_cast<int>(this->mode),
                      (this->mode & 0b10 ? 1:0), 
                      (this->mode & 0b01 ? 1:0)
                      );
        ESP_LOGCONFIG(TAG, "  SPI Target Speed: %u Hz", this->ssi_configured_clock_speed_);
        if (this->spi_bus_ != nullptr) { // Log actual frequency if bus is initialized
             ESP_LOGCONFIG(TAG, "  SPI Actual Bus Speed: %lu Hz", static_cast<unsigned long>(this->frequency_));
        }
    }
    ESP_LOGCONFIG(TAG, "  Zero Offset: %.2f degrees", this->zero_offset_degrees_);
    ESP_LOGCONFIG(TAG, "  Direction Inverted: %s", YESNO(this->direction_inverted_));
    ESP_LOGCONFIG(TAG, "  Velocity Filter Cutoff: %.2f Hz (Filter: %s)", 
                  this->velocity_filter_cutoff_hz_, 
                  (this->velocity_filter_cutoff_hz_ > 0.0f ? "Basic LPF (EMA)" : "Disabled"));
    
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
    }
}

void MT6701SensorComponent::update() {
    if (!read_sensor_data_()) { 
        ESP_LOGW(TAG, "Failed to read data from MT6701 in update cycle.");
        // Publikuj NAN/false dla wszystkich aktywnych sensorów
        if(this->angle_sensor_ && this->angle_sensor_->get_icon() != nullptr) this->angle_sensor_->publish_state(NAN); // Sprawdź, czy sensor jest skonfigurowany
        if(this->accumulated_angle_sensor_ && this->accumulated_angle_sensor_->get_icon() != nullptr) this->accumulated_angle_sensor_->publish_state(NAN);
        if(this->velocity_rpm_sensor_ && this->velocity_rpm_sensor_->get_icon() != nullptr) this->velocity_rpm_sensor_->publish_state(NAN);
        if(this->raw_count_sensor_ && this->raw_count_sensor_->get_icon() != nullptr) this->raw_count_sensor_->publish_state(NAN);
        if(this->raw_radians_sensor_ && this->raw_radians_sensor_->get_icon() != nullptr) this->raw_radians_sensor_->publish_state(NAN);
        if(this->accumulated_count_sensor_ && this->accumulated_count_sensor_->get_icon() != nullptr) this->accumulated_count_sensor_->publish_state(NAN);
        if(this->accumulated_radians_sensor_ && this->accumulated_radians_sensor_->get_icon() != nullptr) this->accumulated_radians_sensor_->publish_state(NAN);
        
        if (this->interface_type_ == MT6701InterfaceType::SSI) {
            if(this->magnetic_field_status_sensor_ && this->magnetic_field_status_sensor_->get_icon() != nullptr) this->magnetic_field_status_sensor_->publish_state(NAN);
            if(this->loss_of_track_status_sensor_ && this->loss_of_track_status_sensor_->get_icon() != nullptr) this->loss_of_track_status_sensor_->publish_state(NAN);
            if(this->push_button_ssi_binary_sensor_ && this->push_button_ssi_binary_sensor_->get_icon() != nullptr) this->push_button_ssi_binary_sensor_->publish_state(false);
        }
        this->status_set_warning();
        return;
    }
    this->status_clear_warning();
    process_angle_data_(this->current_raw_sensor_value_);
}

bool MT6701SensorComponent::read_sensor_data_() {
    uint16_t raw_angle_value = 0;
    uint8_t ssi_status_bits = 0;  
    uint8_t received_ssi_crc = 0; 

    bool read_success = false;
    if (this->interface_type_ == MT6701InterfaceType::I2C) {
        read_success = read_i2c_angle_data_(raw_angle_value);
    } else if (this->interface_type_ == MT6701InterfaceType::SSI) {
        read_success = read_ssi_frame_data_(raw_angle_value, ssi_status_bits, received_ssi_crc);
        if (read_success) { // Jeśli odczyt SPI się powiódł i CRC jest OK
            process_ssi_status_bits_(ssi_status_bits);
        }
    } else {
        ESP_LOGE(TAG, "Unknown interface type in read_sensor_data");
        return false;
    }

    if (read_success) {
        this->current_raw_sensor_value_ = raw_angle_value;
    }
    return read_success;
}

bool MT6701SensorComponent::read_i2c_angle_data_(uint16_t &raw_angle_out) {
    uint8_t buffer[2];
    if (this->read_register(static_cast<uint8_t>(MT6701Register::ANGLE_DATA_H), &buffer[0], 1) != i2c::ERROR_OK) {
        return false;
    }
    if (this->read_register(static_cast<uint8_t>(MT6701Register::ANGLE_DATA_L), &buffer[1], 1) != i2c::ERROR_OK) {
        return false;
    }
    
    raw_angle_out = (static_cast<uint16_t>(buffer[0]) << 6) | (buffer[1] >> 2);
    raw_angle_out &= (MT6701_RESOLUTION_COUNTS - 1);  
    return true;
}

bool MT6701SensorComponent::read_ssi_frame_data_(uint16_t &raw_angle_out, uint8_t &status_bits_out, uint8_t &received_crc_out) {
    if (this->interface_type_ != MT6701InterfaceType::SSI) return false;
    
    uint8_t raw_ssi_frame[SSI_FRAME_SIZE_BYTES]; 
    this->enable(); 
    uint8_t tx_dummy_buffer[SSI_FRAME_SIZE_BYTES] = {0x00, 0x00, 0x00}; 
    this->transfer_array(tx_dummy_buffer, raw_ssi_frame, SSI_FRAME_SIZE_BYTES);
    this->disable(); 

    raw_angle_out = (static_cast<uint16_t>(raw_ssi_frame[0]) << 6) | (raw_ssi_frame[1] >> 2);
    raw_angle_out &= (MT6701_RESOLUTION_COUNTS - 1);

    // *** POPRAWIONA INTERPRETACJA BITÓW STATUSU SSI ZGODNIE Z DATASHEET MT6701 ***
    // Byte 1 (raw_ssi_frame[1]): D5 D4 D3 D2 D1 D0 MgS1 MgS0
    // Byte 2 (raw_ssi_frame[2]): MgS3 MgS2 CRC5 CRC4 CRC3 CRC2 CRC1 CRC0
    uint8_t mgs0_val = (raw_ssi_frame[1] >> 0) & 0x01; // MgS0 z raw_ssi_frame[1], bit 0
    uint8_t mgs1_val = (raw_ssi_frame[1] >> 1) & 0x01; // MgS1 z raw_ssi_frame[1], bit 1
    uint8_t mgs2_val = (raw_ssi_frame[2] >> 6) & 0x01; // MgS2 z raw_ssi_frame[2], bit 6
    uint8_t mgs3_val = (raw_ssi_frame[2] >> 7) & 0x01; // MgS3 z raw_ssi_frame[2], bit 7
    status_bits_out = (mgs3_val << 3) | (mgs2_val << 2) | (mgs1_val << 1) | mgs0_val; // Format: MgS3_MgS2_MgS1_MgS0

    received_crc_out = raw_ssi_frame[2] & 0x3F; 
    
    if (!verify_ssi_crc_(raw_angle_out, status_bits_out, received_crc_out)) {
      // Logowanie błędu CRC już jest w verify_ssi_crc_
      return false; 
    }
    return true;
}

// Implementacja CRC-6/ITU (G(x) = x^6 + x + 1), wartość początkowa 0x00
// Dane wejściowe: 18 bitów (14 bitów kąta + 4 bity statusu), MSB pierwszy
uint8_t MT6701SensorComponent::calculate_ssi_crc6_itu_(uint32_t data_18bit) {
    uint8_t crc_reg = 0x00; // Wartość początkowa CRC
    
    // Wielomian x^6+x+1 jest reprezentowany jako 0x03 dla operacji na 6-bitowym rejestrze,
    // gdzie x^6 jest niejawne i obsługiwane przez przesunięcie i warunkowy XOR.
    // Alternatywnie, pełny wielomian 0x43 (0b1000011) może być użyty z odpowiednią logiką.
    // Poniższy algorytm jest typowy dla CRC liczonego od MSB danych.

    for (int bit_pos = 17; bit_pos >= 0; bit_pos--) {
        bool data_bit = (data_18bit >> bit_pos) & 0x01; // Aktualny bit danych (od MSB)
        bool crc_msb = (crc_reg & 0x20) != 0;      // Najstarszy bit obecnego CRC (bit 5)

        crc_reg <<= 1; // Przesuń CRC w lewo
        if (data_bit ^ crc_msb) { // Jeśli (bit danych XOR MSB rejestru CRC) == 1
            crc_reg ^= SSI_CRC6_ITU_POLY_REDUCED; // XOR z "odchudzonym" wielomianem (0x03)
        }
    }
    return crc_reg & 0x3F; // Zapewnij 6-bitowy wynik
}

bool MT6701SensorComponent::verify_ssi_crc_(uint16_t angle_data, uint8_t status_data, uint8_t received_crc) {
    // Skonstruuj 18-bitowe słowo danych dla CRC: Kąt[13:0]MgS[3:0]
    // angle_data to 14 bitów, status_data to 4 bity (MgS3_MgS2_MgS1_MgS0)
    uint32_t data_word_for_crc = (static_cast<uint32_t>(angle_data) << 4) | (status_data & 0x0F);

    uint8_t calculated_crc = calculate_ssi_crc6_itu_(data_word_for_crc);
    
    if (calculated_crc != received_crc) {
        ESP_LOGW(TAG, "SSI CRC MISMATCH! Data: Angle=0x%04X, Status=0x%X. CalcCRC=0x%02X, RecvCRC=0x%02X",
                 angle_data, status_data, calculated_crc, received_crc);
        return false; 
    }
    ESP_LOGV(TAG, "SSI CRC OK. Data: Angle=0x%04X, Status=0x%X. CRC=0x%02X", 
             angle_data, status_data, received_crc);
    return true;
}

void MT6701SensorComponent::process_angle_data_(uint16_t raw_angle_from_sensor) {
    if (this->raw_count_sensor_ != nullptr && this->raw_count_sensor_->get_icon() != nullptr) {
        this->raw_count_sensor_->publish_state(raw_angle_from_sensor);
    }
    if (this->raw_radians_sensor_ != nullptr && this->raw_radians_sensor_->get_icon() != nullptr) {
        float raw_radians = static_cast<float>(raw_angle_from_sensor) * COUNTS_TO_RADIANS_FACTOR;
        this->raw_radians_sensor_->publish_state(raw_radians);
    }

    int16_t current_processed_value = static_cast<int16_t>(raw_angle_from_sensor);  
    int16_t diff = current_processed_value - static_cast<int16_t>(this->previous_raw_sensor_value_for_velocity_);

    if (std::abs(diff) > (MT6701_RESOLUTION_COUNTS / 2)) {
        if (diff > 0) {
            diff -= MT6701_RESOLUTION_COUNTS;
        } else {
            diff += MT6701_RESOLUTION_COUNTS;
        }
    }

    if (this->direction_inverted_) {
        this->internal_accumulated_count_ -= diff;
    } else {
        this->internal_accumulated_count_ += diff;
    }
    
    if (this->accumulated_count_sensor_ != nullptr && this->accumulated_count_sensor_->get_icon() != nullptr) {
        this->accumulated_count_sensor_->publish_state(this->internal_accumulated_count_);
    }
    
    if (this->accumulated_angle_sensor_ != nullptr && this->accumulated_angle_sensor_->get_icon() != nullptr) {
        float accumulated_degrees_val = static_cast<float>(this->internal_accumulated_count_) * COUNTS_TO_DEGREES_FACTOR;
        this->accumulated_angle_sensor_->publish_state(accumulated_degrees_val);
    }
    if (this->accumulated_radians_sensor_ != nullptr && this->accumulated_radians_sensor_->get_icon() != nullptr) {
        float accumulated_radians_val = static_cast<float>(this->internal_accumulated_count_) * COUNTS_TO_RADIANS_FACTOR;
        this->accumulated_radians_sensor_->publish_state(accumulated_radians_val);
    }

    float angle_degrees_display = static_cast<float>(raw_angle_from_sensor) * COUNTS_TO_DEGREES_FACTOR;
    angle_degrees_display -= this->zero_offset_degrees_;  
    
    angle_degrees_display = fmodf(angle_degrees_display, 360.0f);
    if (angle_degrees_display < 0) {
        angle_degrees_display += 360.0f;
    }

    if (this->angle_sensor_ != nullptr && this->angle_sensor_->get_icon() != nullptr) {
        this->angle_sensor_->publish_state(angle_degrees_display);
    }

    uint32_t now_us = micros();
    uint32_t dt_us = now_us - this->last_velocity_update_time_us_;
    
    if (this->velocity_rpm_sensor_ != nullptr && this->velocity_rpm_sensor_->get_icon() != nullptr) {
        if (dt_us >= 500) { 
            float degrees_moved_for_rpm = static_cast<float>(diff) * COUNTS_TO_DEGREES_FACTOR;
            float velocity_rpm_val = (degrees_moved_for_rpm / static_cast<float>(dt_us)) * DEGREES_PER_US_TO_RPM_FACTOR;
            
            if (this->direction_inverted_){
                 velocity_rpm_val *= -1.0f;
            }

            if (this->velocity_filter_cutoff_hz_ > 0.0f) { 
                float dt_s = static_cast<float>(dt_us) / 1000000.0f;
                float rc = 1.0f / (2.0f * PI * this->velocity_filter_cutoff_hz_);
                float alpha = dt_s / (rc + dt_s);
                if (alpha > 1.0f) alpha = 1.0f; 
                if (alpha < 0.0f) alpha = 0.0f;

                if (this->first_velocity_sample_) {
                    this->filtered_velocity_rpm_ = velocity_rpm_val;
                    this->first_velocity_sample_ = false;
                } else {
                    this->filtered_velocity_rpm_ = alpha * velocity_rpm_val + (1.0f - alpha) * this->filtered_velocity_rpm_;
                }
                this->velocity_rpm_sensor_->publish_state(this->filtered_velocity_rpm_);
            } else { 
                this->velocity_rpm_sensor_->publish_state(velocity_rpm_val);
                if (this->first_velocity_sample_) { 
                    this->filtered_velocity_rpm_ = velocity_rpm_val; // Inicjalizuj filtr, nawet jeśli nieaktywny
                    this->first_velocity_sample_ = false;
                }
            }
            this->last_velocity_update_time_us_ = now_us;
        } else if (!this->first_velocity_sample_) { 
            this->velocity_rpm_sensor_->publish_state(this->filtered_velocity_rpm_);
        } else { 
            this->velocity_rpm_sensor_->publish_state(0.0f);
        }
    }
        
    this->previous_raw_sensor_value_for_velocity_ = current_processed_value;
}

void MT6701SensorComponent::process_ssi_status_bits_(uint8_t status_bits) {
    // status_bits jest w formacie: MgS3_MgS2_MgS1_MgS0
    if (this->magnetic_field_status_sensor_ != nullptr && this->magnetic_field_status_sensor_->get_icon() != nullptr) {
        this->magnetic_field_status_sensor_->publish_state(status_bits & 0x03); 
    }
    if (this->loss_of_track_status_sensor_ != nullptr && this->loss_of_track_status_sensor_->get_icon() != nullptr) {
        this->loss_of_track_status_sensor_->publish_state((status_bits >> 3) & 0x01); 
    }
    if (this->push_button_ssi_binary_sensor_ != nullptr && this->push_button_ssi_binary_sensor_->get_icon() != nullptr) {
        this->push_button_ssi_binary_sensor_->publish_state((status_bits >> 2) & 0x01); 
    }
}

} // namespace mt6701
} // namespace esphome