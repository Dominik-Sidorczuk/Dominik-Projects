# Python script for ESPHome to define the MT6701 sensor component's
# YAML configuration schema and C++ code generation.

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, spi, binary_sensor # Upewnij się, że 'binary_sensor' jest potrzebny jeśli używasz CONF_PUSH_BUTTON_SSI
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    UNIT_DEGREES,
    UNIT_REVOLUTIONS_PER_MINUTE,
    UNIT_EMPTY, # For raw counts or typeless numeric values
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING, # For counters that only go up
    DEVICE_CLASS_ROTATION,
    DEVICE_CLASS_BUTTON, # Potrzebne dla CONF_PUSH_BUTTON_SSI
)

# --- Custom Definitions ---
# C++ Namespace and Class for the component
mt6701_ns = cg.esphome_ns.namespace("esphome::mt6701")
MT6701SensorComponent = mt6701_ns.class_(
    "MT6701SensorComponent", cg.PollingComponent, i2c.I2CDevice, spi.SPIDevice
)

# Velocity Filter C++ Enum Definition
VelocityFilterType = mt6701_ns.enum("VelocityFilterType")
VELOCITY_FILTER_TYPES_MAP = {
    "NONE": VelocityFilterType.NONE,
    "EMA": VelocityFilterType.EMA,
    "BUTTERWORTH_2ND_ORDER": VelocityFilterType.BUTTERWORTH_2ND_ORDER,
}

# Custom Units for Sensors
UNIT_RADIANS = "rad" # Zdefiniowane lokalnie, nie z esphome.const

# Custom Icons (MDI strings) for Sensors and Binary Sensors - zdefiniowane lokalnie
ICON_ANGLE_DEGREES = "mdi:rotate-right"
ICON_ACCUMULATED_ANGLE = "mdi:sigma"
ICON_VELOCITY_RPM = "mdi:speedometer"
ICON_RAW_COUNT = "mdi:counter"
ICON_RAW_RADIANS = "mdi:angle-acute"
ICON_ACCUMULATED_COUNT = "mdi:counter"
ICON_ACCUMULATED_RADIANS = "mdi:angle-acute"
ICON_MAGNETIC_FIELD_STATUS = "mdi:magnet-on"
ICON_LOSS_OF_TRACK_STATUS = "mdi:radar"
ICON_PUSH_BUTTON = "mdi:button-pointer" # Example, often overridden by device_class default
ICON_SSI_CRC_ERROR_COUNT = "mdi:alert-octagon"


# --- Configuration Keys ---
# Interface Selection Keys
CONF_INTERFACE = "interface"
CONF_I2C = "i2c" # Used as a key within CONF_INTERFACE for I2C settings
CONF_SSI = "ssi" # Used as a key within CONF_INTERFACE for SSI/SPI settings

# General Component Settings
CONF_ZERO_OFFSET_DEGREES = "zero_offset_degrees"
CONF_DIRECTION_INVERTED = "direction_inverted"
CONF_VELOCITY_FILTER_TYPE = "velocity_filter_type"
CONF_VELOCITY_FILTER_CUTOFF_FREQUENCY = "velocity_filter_cutoff_frequency"
CONF_MIN_VELOCITY_UPDATE_PERIOD = "min_velocity_update_period"

# Sensor Entity Configuration Keys
CONF_MAIN_ANGLE_SENSOR = "angle"
CONF_ACCUMULATED_ANGLE = "accumulated_angle"
CONF_VELOCITY_RPM = "velocity_rpm"
CONF_RAW_COUNT = "raw_count"
CONF_RAW_RADIANS = "raw_radians"
CONF_ACCUMULATED_COUNT = "accumulated_count"
CONF_ACCUMULATED_RADIANS = "accumulated_radians"
CONF_MAGNETIC_FIELD_STATUS = "magnetic_field_status"
CONF_LOSS_OF_TRACK_STATUS = "loss_of_track_status"
CONF_SSI_CRC_ERROR_COUNT = "ssi_crc_error_count"

# Binary Sensor Entity Configuration Keys
CONF_PUSH_BUTTON_SSI = "push_button_ssi" # Specific to SSI interface


# --- Interface Schemas ---
# I2C Interface Schema: Defines configuration options for I2C communication.
# Default I2C address for MT6701 is typically 0x06.
I2C_INTERFACE_SCHEMA = i2c.i2c_device_schema(default_address=0x06)

# SSI (Synchronous Serial Interface) over SPI Schema: Defines options for SPI.
SSI_INTERFACE_SCHEMA = spi.spi_device_schema(
    cs_pin_required=True,
    clk_pin_required=True,
    miso_pin_required=True,
    mosi_pin_required=False, # MT6701 DI not used for reads
    default_mode=2 # Verify for MT6701 (often Mode 2 or 3)
)

# --- Main Component Configuration Schema ---
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MT6701SensorComponent),

        cv.Required(CONF_INTERFACE): cv.typed_schema(
            {
                CONF_I2C: I2C_INTERFACE_SCHEMA,
                CONF_SSI: SSI_INTERFACE_SCHEMA,
            },
            lower=True,
        ),

        cv.Optional(CONF_ZERO_OFFSET_DEGREES, default="0.0deg"): cv.angle, # Zmieniono z "0.0°" na "0.0deg" dla spójności
        cv.Optional(CONF_DIRECTION_INVERTED, default=False): cv.boolean,
        cv.Optional(CONF_VELOCITY_FILTER_TYPE, default="EMA"): cv.enum(
            VELOCITY_FILTER_TYPES_MAP, upper=True
        ),
        cv.Optional(CONF_VELOCITY_FILTER_CUTOFF_FREQUENCY, default="10Hz"): cv.positive_frequency,
        cv.Optional(CONF_MIN_VELOCITY_UPDATE_PERIOD, default="1ms"): cv.positive_time_period_microseconds,

        cv.Required(CONF_MAIN_ANGLE_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            icon=ICON_ANGLE_DEGREES,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_ROTATION,
        ),
        cv.Optional(CONF_ACCUMULATED_ANGLE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            icon=ICON_ACCUMULATED_ANGLE,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_ROTATION,
        ),
        cv.Optional(CONF_VELOCITY_RPM): sensor.sensor_schema(
            unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
            icon=ICON_VELOCITY_RPM,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            # device_class=DEVICE_CLASS_SPEED, # Można dodać, jeśli jest odpowiedni w esphome.const
        ),
        cv.Optional(CONF_RAW_COUNT): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_RAW_COUNT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_RAW_RADIANS): sensor.sensor_schema(
            unit_of_measurement=UNIT_RADIANS, # Używa lokalnie zdefiniowanego UNIT_RADIANS
            icon=ICON_RAW_RADIANS,
            accuracy_decimals=3,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ACCUMULATED_COUNT): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_ACCUMULATED_COUNT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ACCUMULATED_RADIANS): sensor.sensor_schema(
            unit_of_measurement=UNIT_RADIANS, # Używa lokalnie zdefiniowanego UNIT_RADIANS
            icon=ICON_ACCUMULATED_RADIANS,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MAGNETIC_FIELD_STATUS): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_MAGNETIC_FIELD_STATUS,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_LOSS_OF_TRACK_STATUS): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_LOSS_OF_TRACK_STATUS,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SSI_CRC_ERROR_COUNT): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_SSI_CRC_ERROR_COUNT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_PUSH_BUTTON_SSI): binary_sensor.binary_sensor_schema( # Wymaga importu 'binary_sensor'
            device_class=DEVICE_CLASS_BUTTON,
            # icon=ICON_PUSH_BUTTON, # Można odkomentować, jeśli chcesz nadpisać domyślną ikonę
        ),
    }
).extend(cv.polling_component_schema())


# --- Code Generation Function ---
async def to_code(config):
    """Generates C++ code for the MT6701 component based on YAML configuration."""
    comp_var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(comp_var, config)

    interface_type, interface_config_payload = next(iter(config[CONF_INTERFACE].items()))

    if interface_type == CONF_I2C:
        await i2c.register_i2c_device(comp_var, interface_config_payload)
        cg.add(comp_var.set_interface_i2c())
    elif interface_type == CONF_SSI:
        await spi.register_spi_device(comp_var, interface_config_payload)
        cg.add(comp_var.set_interface_ssi())

    parameter_setters = {
        CONF_ZERO_OFFSET_DEGREES: comp_var.set_zero_offset_degrees,
        CONF_DIRECTION_INVERTED: comp_var.set_direction_inverted,
        CONF_VELOCITY_FILTER_TYPE: comp_var.set_velocity_filter_type_str,
        CONF_VELOCITY_FILTER_CUTOFF_FREQUENCY: comp_var.set_velocity_filter_cutoff_hz,
        CONF_MIN_VELOCITY_UPDATE_PERIOD: comp_var.set_min_velocity_update_period_us,
    }
    for conf_key, setter_method in parameter_setters.items():
        if conf_key in config:
            cg.add(setter_method(config[conf_key]))

    if CONF_MAIN_ANGLE_SENSOR in config:
        main_sensor_var = await sensor.new_sensor(config[CONF_MAIN_ANGLE_SENSOR])
        cg.add(comp_var.set_angle_sensor(main_sensor_var))

    optional_entities = [
        (CONF_ACCUMULATED_ANGLE, comp_var.set_accumulated_angle_sensor, sensor),
        (CONF_VELOCITY_RPM, comp_var.set_velocity_rpm_sensor, sensor),
        (CONF_RAW_COUNT, comp_var.set_raw_count_sensor, sensor),
        (CONF_RAW_RADIANS, comp_var.set_raw_radians_sensor, sensor),
        (CONF_ACCUMULATED_COUNT, comp_var.set_accumulated_count_sensor, sensor),
        (CONF_ACCUMULATED_RADIANS, comp_var.set_accumulated_radians_sensor, sensor),
        (CONF_MAGNETIC_FIELD_STATUS, comp_var.set_magnetic_field_status_sensor, sensor),
        (CONF_LOSS_OF_TRACK_STATUS, comp_var.set_loss_of_track_status_sensor, sensor),
        (CONF_SSI_CRC_ERROR_COUNT, comp_var.set_ssi_crc_error_sensor, sensor),
        (CONF_PUSH_BUTTON_SSI, comp_var.set_push_button_ssi_binary_sensor, binary_sensor),
    ]

    for conf_key, setter_method, entity_module in optional_entities:
        if conf_key in config:
            entity_config_payload = config[conf_key]
            entity_var = None

            if entity_module == sensor:
                entity_var = await sensor.new_sensor(entity_config_payload)
            elif entity_module == binary_sensor:
                entity_var = await binary_sensor.new_binary_sensor(entity_config_payload)
            else:
                cg.logger.warning(
                    f"MT6701: Unknown entity module type for configuration key '{conf_key}'."
                )
                continue
            
            if entity_var:
                cg.add(setter_method(entity_var))