# Python script for ESPHome to define the MT6701 sensor component's 
# YAML configuration schema and C++ code generation.

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, spi, binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    # CONF_CS_PIN, # Automatically handled by spi.spi_device_schema for the CS pin object
    # CONF_CLOCK_SPEED, # Automatically handled by spi.spi_device_schema
    UNIT_DEGREES,
    UNIT_REVOLUTIONS_PER_MINUTE,
    UNIT_HERTZ,
    UNIT_EMPTY, # For raw counts
    ICON_ROTATE_RIGHT,
    ICON_SPEEDOMETER,
    ICON_MAGNET_ON, # For magnetic field status
    ICON_RADAR, # For loss of track status
    ICON_COUNTER, # For count-based sensors
    ICON_ALERT_OCTAGON, # For CRC errors
    ICON_ANGLE_ACUTE, # For radian measurements
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING, # For counters that only go up (like error counts)
    DEVICE_CLASS_ROTATION,
    DEVICE_CLASS_BUTTON,
    # CONF_TYPE, # Using custom CONF_VELOCITY_FILTER_TYPE for clarity
    # CONF_MIN_UPDATE_INTERVAL, # Using custom CONF_MIN_VELOCITY_UPDATE_PERIOD for clarity
)

# Custom unit and icon definitions (if any beyond standard)
UNIT_RADIANS = "rad"
ICON_SIGMA = "mdi:sigma" # Icon for accumulated values or standard deviation-like metrics

# --- Configuration Keys (custom to this component) ---
# CONF_MT6701_ID is implicitly handled by cv.GenerateID() mapping to CONF_ID
# It defines the YAML key for the component's ID if specified by user.

# Interface selection keys
CONF_INTERFACE = "interface"
CONF_I2C = "i2c"
CONF_SSI = "ssi" # Synchronous Serial Interface (implemented over SPI)

# Core sensor entity keys (user-defined names in YAML)
CONF_MAIN_ANGLE_SENSOR = "angle"
CONF_ACCUMULATED_ANGLE = "accumulated_angle"
CONF_VELOCITY_RPM = "velocity_rpm"

# Raw/alternative sensor entity keys
CONF_RAW_COUNT = "raw_count"
CONF_RAW_RADIANS = "raw_radians"
CONF_ACCUMULATED_COUNT = "accumulated_count"
CONF_ACCUMULATED_RADIANS = "accumulated_radians"

# SSI-specific status entity keys
CONF_MAGNETIC_FIELD_STATUS = "magnetic_field_status"
CONF_LOSS_OF_TRACK_STATUS = "loss_of_track_status"
CONF_PUSH_BUTTON_SSI = "push_button_ssi"
CONF_SSI_CRC_ERROR_COUNT = "ssi_crc_error_count"

# Component operational parameter keys
CONF_ZERO_OFFSET_DEGREES = "zero_offset_degrees" # YAML key for zero offset setting
CONF_DIRECTION_INVERTED = "direction_inverted"
CONF_VELOCITY_FILTER_TYPE = "velocity_filter_type"
CONF_VELOCITY_FILTER_CUTOFF_FREQUENCY = "velocity_filter_cutoff_frequency" # YAML key for filter cutoff
CONF_MIN_VELOCITY_UPDATE_PERIOD = "min_velocity_update_period"

# --- C++ Namespace and Class Definitions ---
mt6701_ns = cg.esphome_ns.namespace("esphome::mt6701") # C++ namespace
MT6701SensorComponent = mt6701_ns.class_(
    "MT6701SensorComponent", cg.PollingComponent, i2c.I2CDevice, spi.SPIDevice
) # C++ class, inheriting from PollingComponent and relevant device types

# Mirror C++ VelocityFilterType enum for Python-side validation
VelocityFilterType = mt6701_ns.enum("VelocityFilterType")
VELOCITY_FILTER_TYPES_MAP = {
    "NONE": VelocityFilterType.NONE,
    "EMA": VelocityFilterType.EMA,
    "BUTTERWORTH_2ND_ORDER": VelocityFilterType.BUTTERWORTH_2ND_ORDER,
}

# --- Configuration Schemas for Interfaces ---
I2C_INTERFACE_SCHEMA = i2c.i2c_device_schema(default_address=0x06) # Standard I2C device schema

# SSI (SPI) Interface Schema
# Note on MT6701 SPI/SSI Mode:
# Datasheet typically states: SCLK idles HIGH (CPOL=1). Data is valid on RISING edge of SCLK.
# For ESPHome SPI with CPOL=1 (CLOCK_POLARITY_HIGH):
#   - CLOCK_PHASE_LEADING_EDGE (CPHA=0 in some nomenclatures) samples on FALLING edge. (This is SPI Mode 2)
#   - CLOCK_PHASE_TRAILING_EDGE (CPHA=1 in some nomenclatures) samples on RISING edge. (This is SPI Mode 3)
# The C++ code is currently configured for CLOCK_POLARITY_HIGH and CLOCK_PHASE_LEADING_EDGE.
# This schema's default_mode=2 (CPOL=1, CPHA=0) aligns with that C++ configuration.
# **CRITICAL**: This mode MUST be verified against your specific MT6701 datasheet and hardware.
# If data should be sampled on the RISING edge, SPI Mode 3 (default_mode=3 for spi.spi_device_schema) is likely needed,
# and the C++ SPIDevice template parameters should also be updated to spi::CLOCK_PHASE_TRAILING_EDGE.
SSI_INTERFACE_SCHEMA = spi.spi_device_schema(
    cs_pin_required=True,    # Chip Select is mandatory for MT6701 SSI
    clk_pin_required=True,   # Clock pin is mandatory
    miso_pin_required=True,  # MT6701's Data Out (DO) is ESP's MISO
    mosi_pin_required=False, # MOSI is not used for reading from MT6701 via SSI
    default_mode=2           # Corresponds to CPOL=1, CPHA=0. VERIFY THIS!
).extend(
    {
        # CONF_CLOCK_SPEED is part of spi.spi_device_schema() and is automatically
        # handled. It will be available in C++ via this->spi_settings_.data_rate.
    }
)

# --- Main Configuration Schema for the MT6701 Component ---
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MT6701SensorComponent), # Generates CONF_ID for the component instance
        cv.Required(CONF_INTERFACE): cv.typed_schema( # User must specify one interface
            {
                CONF_I2C: I2C_INTERFACE_SCHEMA,
                CONF_SSI: SSI_INTERFACE_SCHEMA,
            },
            lower=True, # Converts keys "i2c", "ssi" to lowercase
        ),

        # Operational parameters
        cv.Optional(CONF_ZERO_OFFSET_DEGREES, default="0.0°"): cv.angle,
        cv.Optional(CONF_DIRECTION_INVERTED, default=False): cv.boolean,
        cv.Optional(CONF_VELOCITY_FILTER_TYPE, default="EMA"): cv.enum(
            VELOCITY_FILTER_TYPES_MAP, upper=True # Validate against defined filter types
        ),
        cv.Optional(CONF_VELOCITY_FILTER_CUTOFF_FREQUENCY, default="10Hz"): cv.positive_frequency,
        cv.Optional(CONF_MIN_VELOCITY_UPDATE_PERIOD, default="1ms"): cv.positive_time_period_microseconds,

        # Core sensor outputs (at least 'angle' is required)
        cv.Required(CONF_MAIN_ANGLE_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            icon=ICON_ROTATE_RIGHT,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_ROTATION,
        ),
        cv.Optional(CONF_ACCUMULATED_ANGLE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            icon=ICON_SIGMA,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT, # Accumulated angle can be positive or negative
            device_class=DEVICE_CLASS_ROTATION,
        ),
        cv.Optional(CONF_VELOCITY_RPM): sensor.sensor_schema(
            unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
            icon=ICON_SPEEDOMETER,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),

        # Raw/alternative sensor outputs (optional)
        cv.Optional(CONF_RAW_COUNT): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY, # Raw counts from the sensor
            icon=ICON_COUNTER,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_RAW_RADIANS): sensor.sensor_schema(
            unit_of_measurement=UNIT_RADIANS,
            icon=ICON_ANGLE_ACUTE,
            accuracy_decimals=3,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ACCUMULATED_COUNT): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY, # Accumulated raw counts
            icon=ICON_COUNTER,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT, # Can be positive or negative
        ),
        cv.Optional(CONF_ACCUMULATED_RADIANS): sensor.sensor_schema(
            unit_of_measurement=UNIT_RADIANS,
            icon=ICON_ANGLE_ACUTE,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        
        # SSI-specific status outputs (optional, only relevant for SSI interface)
        cv.Optional(CONF_MAGNETIC_FIELD_STATUS): sensor.sensor_schema(
            icon=ICON_MAGNET_ON,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            # Reports Mg[1:0] status: e.g., 0=Normal, 1=Too Strong, 2=Too Weak
        ),
        cv.Optional(CONF_LOSS_OF_TRACK_STATUS): sensor.sensor_schema(
            icon=ICON_RADAR, 
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            # Reports Mg[3] status: e.g., 0=Normal, 1=Loss of Track
        ),
        cv.Optional(CONF_PUSH_BUTTON_SSI): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_BUTTON,
            # Reports Mg[2] status: e.g., 0=Not Pressed, 1=Pressed
        ),
        cv.Optional(CONF_SSI_CRC_ERROR_COUNT): sensor.sensor_schema(
            icon=ICON_ALERT_OCTAGON,
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING, # Error count only increases or is reset by service
        ),
    }
).extend(cv.polling_component_schema()) # Standard polling settings (update_interval)


# --- Code Generation Function ---
async def to_code(config):
    """Generate C++ code for the MT6701 component."""
    
    # Create a Pvariable for the C++ component instance using the generated ID
    comp_var = cg.new_Pvariable(config[CONF_ID]) 
    await cg.register_component(comp_var, config) # Register as a generic component

    # Configure the selected interface (I2C or SSI)
    interface_type, interface_config_payload = next(iter(config[CONF_INTERFACE].items()))
    if interface_type == CONF_I2C:
        await i2c.register_i2c_device(comp_var, interface_config_payload) # Setup as I2C device
        cg.add(comp_var.set_interface_i2c()) # Call C++ method to set interface type
    elif interface_type == CONF_SSI:
        await spi.register_spi_device(comp_var, interface_config_payload) # Setup as SPI device
        cg.add(comp_var.set_interface_ssi()) # Call C++ method to set interface type
        # Note: CONF_CLOCK_SPEED for SSI is handled by spi.register_spi_device,
        # which configures spi_settings_.data_rate on the C++ SPIDevice base.

    # Set operational parameters by calling C++ setters
    # These use a direct mapping from config key to C++ setter and value.
    parameter_map = {
        CONF_ZERO_OFFSET_DEGREES: comp_var.set_zero_offset_degrees,
        CONF_DIRECTION_INVERTED: comp_var.set_direction_inverted,
        CONF_VELOCITY_FILTER_TYPE: comp_var.set_velocity_filter_type_str,
        CONF_VELOCITY_FILTER_CUTOFF_FREQUENCY: comp_var.set_velocity_filter_cutoff_hz,
        CONF_MIN_VELOCITY_UPDATE_PERIOD: comp_var.set_min_velocity_update_period_us,
    }
    for conf_key, setter_func in parameter_map.items():
        if conf_key in config:
            cg.add(setter_func(config[conf_key]))

    # Register the required main angle sensor
    # This sensor must be defined in the YAML.
    if CONF_MAIN_ANGLE_SENSOR in config: 
        main_sensor_var = await sensor.new_sensor(config[CONF_MAIN_ANGLE_SENSOR])
        cg.add(comp_var.set_angle_sensor(main_sensor_var))

    # Register optional sensors and binary sensors
    # This uses a list of tuples to map config keys to C++ setters and sensor types.
    optional_entities_setup = [
        (CONF_ACCUMULATED_ANGLE, comp_var.set_accumulated_angle_sensor, sensor),
        (CONF_VELOCITY_RPM, comp_var.set_velocity_rpm_sensor, sensor),
        (CONF_RAW_COUNT, comp_var.set_raw_count_sensor, sensor),
        (CONF_RAW_RADIANS, comp_var.set_raw_radians_sensor, sensor),
        (CONF_ACCUMULATED_COUNT, comp_var.set_accumulated_count_sensor, sensor),
        (CONF_ACCUMULATED_RADIANS, comp_var.set_accumulated_radians_sensor, sensor),
        (CONF_MAGNETIC_FIELD_STATUS, comp_var.set_magnetic_field_status_sensor, sensor),
        (CONF_LOSS_OF_TRACK_STATUS, comp_var.set_loss_of_track_status_sensor, sensor),
        (CONF_PUSH_BUTTON_SSI, comp_var.set_push_button_ssi_binary_sensor, binary_sensor),
        (CONF_SSI_CRC_ERROR_COUNT, comp_var.set_ssi_crc_error_sensor, sensor),
    ]

    for conf_key, setter_func, entity_module in optional_entities_setup:
        if conf_key in config:
            entity_config = config[conf_key]
            if entity_module == sensor:
                var = await sensor.new_sensor(entity_config)
            elif entity_module == binary_sensor:
                var = await binary_sensor.new_binary_sensor(entity_config)
            else:
                # Should not happen with the current setup
                continue 
            cg.add(setter_func(var))