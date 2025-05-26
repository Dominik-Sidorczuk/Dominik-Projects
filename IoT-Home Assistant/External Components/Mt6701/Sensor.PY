# custom_components/mt6701/sensor.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, spi, binary_sensor # Added binary_sensor for push button
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    UNIT_DEGREES,
    UNIT_REVOLUTIONS_PER_MINUTE,
    UNIT_HERTZ,
    UNIT_EMPTY,
    ICON_ROTATE_RIGHT,
    ICON_SPEEDOMETER,
    UNIT_HERTZ,
    ICON_MAGNET_ON, # Suitable for magnetic strength and push button status
    ICON_RADAR,
    ICON_COUNTER,
    ICON_ANGLE_ACUTE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    CONF_CS_PIN, # Chip Select pin, will be used by SSI
    CONF_CLOCK_SPEED,
    DEVICE_CLASS_BUTTON, # For the push button binary sensor
)

# Custom unit and icon definitions
UNIT_RADIANS = "rad"
ICON_SIGMA = "mdi:sigma"

# --- Configuration Keys ---
CONF_MT6701_ID = "mt6701_id"
CONF_INTERFACE = "interface"
CONF_I2C = "i2c"
CONF_SSI = "ssi" # Changed from SPI to SSI

CONF_MAIN_ANGLE_SENSOR = "angle"
CONF_ACCUMULATED_ANGLE = "accumulated_angle"
CONF_VELOCITY_RPM = "velocity_rpm"

CONF_RAW_COUNT = "raw_count"
CONF_RAW_RADIANS = "raw_radians"
CONF_ACCUMULATED_COUNT = "accumulated_count"
CONF_ACCUMULATED_RADIANS = "accumulated_radians"

# Optional sensors/binary_sensors based on SSI Mg status bits
CONF_MAGNETIC_FIELD_STATUS = "magnetic_field_status" # Renamed for clarity, reports Mg[1:0]
CONF_LOSS_OF_TRACK_STATUS = "loss_of_track_status"   # Reports Mg[3]
CONF_PUSH_BUTTON_SSI = "push_button_ssi"          # Reports Mg[2] (Push button from SSI stream)

# Keys for internal operational parameters
CONF_ZERO_OFFSET = "zero_offset"
CONF_DIRECTION_INVERTED = "direction_inverted"
CONF_VELOCITY_FILTER_CUTOFF = "velocity_filter_cutoff_frequency"

# --- C++ Namespace and Classes ---
mt6701_ns = cg.esphome_ns.namespace("esphome::mt6701")
MT6701SensorComponent = mt6701_ns.class_("MT6701SensorComponent", cg.PollingComponent)

# --- Configuration Schemas for Interfaces ---
I2C_INTERFACE_SCHEMA = i2c.i2c_device_schema(default_address=0x06)

# SSI Interface Schema
# MT6701 SSI: CLK idle high, data driven on CLK rising edge, master captures on CLK falling edge.
# This typically corresponds to SPI Mode 2 (CPOL=1, CPHA=0).
# Pins: DO (Data Out from MT6701 -> MISO on ESP), CLK, CSN (Chip Select Not)
SSI_INTERFACE_SCHEMA = spi.spi_device_schema(
    cs_pin_required=True, # CSN is required
    clk_pin_required=True,
    miso_pin_required=True, # DO (Data Out) from MT6701 is MISO for the ESP
    mosi_pin_required=False, # No MOSI needed for reading from MT6701 via SSI
    default_mode=2 # CPOL=1 (idle high), CPHA=0
).extend(
    {
        cv.Optional(CONF_CLOCK_SPEED, default="4MHz"): cv.positive_frequency, # Max 15.6MHz (64ns period)
    }
)

# --- Main Configuration Schema for the Component ---
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MT6701_ID): cv.declare_id(MT6701SensorComponent),
        cv.Required(CONF_INTERFACE): cv.typed_schema(
            {
                CONF_I2C: I2C_INTERFACE_SCHEMA,
                CONF_SSI: SSI_INTERFACE_SCHEMA, # Changed from CONF_SPI
            },
            lower=True,
        ),

        cv.Optional(CONF_ZERO_OFFSET, default="0.0°"): cv.angle,
        cv.Optional(CONF_DIRECTION_INVERTED, default=False): cv.boolean,
        cv.Optional(CONF_VELOCITY_FILTER_CUTOFF, default="10Hz"): cv.positive_frequency,

        cv.Required(CONF_MAIN_ANGLE_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            icon=ICON_ROTATE_RIGHT,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=sensor.SensorDeviceClass.ROTATION,
        ).extend({cv.GenerateID(): cv.declare_id(sensor.Sensor)}),

        cv.Optional(CONF_ACCUMULATED_ANGLE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES, icon=ICON_SIGMA, accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT, device_class=sensor.SensorDeviceClass.ROTATION,
        ),
        cv.Optional(CONF_VELOCITY_RPM): sensor.sensor_schema(
            unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE, icon=ICON_SPEEDOMETER,
            accuracy_decimals=1, state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_RAW_COUNT): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY, icon=ICON_COUNTER, accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_RAW_RADIANS): sensor.sensor_schema(
            unit_of_measurement=UNIT_RADIANS, icon=ICON_ANGLE_ACUTE, accuracy_decimals=3,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ACCUMULATED_COUNT): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY, icon=ICON_COUNTER, accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_ACCUMULATED_RADIANS): sensor.sensor_schema(
            unit_of_measurement=UNIT_RADIANS, icon=ICON_ANGLE_ACUTE, accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        
        # Optional sensors/binary_sensor specific to SSI interface (from Mg status bits)
        cv.Optional(CONF_MAGNETIC_FIELD_STATUS): sensor.sensor_schema(
            icon=ICON_MAGNET_ON, accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT,
            # Reports Mg[1:0]: 0=Normal, 1=Too Strong, 2=Too Weak
        ),
        cv.Optional(CONF_LOSS_OF_TRACK_STATUS): sensor.sensor_schema(
            icon=ICON_RADAR, accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT,
            # Reports Mg[3]: 0=Normal, 1=Loss of Track
        ),
        cv.Optional(CONF_PUSH_BUTTON_SSI): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_BUTTON, # Or DEVICE_CLASS_OCCUPANCY if more generic
            # Reports Mg[2]: 0=Not Pressed, 1=Pressed
        ),
    }
).extend(cv.polling_component_schema())


async def to_code(config):
    comp_var = cg.new_Pvariable(config[CONF_MT6701_ID])
    await cg.register_component(comp_var, config)

    interface_type, interface_config_payload = next(iter(config[CONF_INTERFACE].items()))
    if interface_type == CONF_I2C:
        await i2c.register_i2c_device(comp_var, interface_config_payload)
        cg.add(comp_var.set_interface_i2c())
    elif interface_type == CONF_SSI: # Changed from CONF_SPI
        # spi.register_spi_device will handle pin setup (CS, CLK, MISO) and mode
        await spi.register_spi_device(comp_var, interface_config_payload)
        if CONF_CLOCK_SPEED in interface_config_payload:
            cg.add(comp_var.set_ssi_clock_speed(interface_config_payload[CONF_CLOCK_SPEED])) # C++: void set_ssi_clock_speed(uint32_t clock_speed);
        cg.add(comp_var.set_interface_ssi()) # Call C++ method: void set_interface_ssi();

    if CONF_ZERO_OFFSET in config:
        cg.add(comp_var.set_zero_offset(config[CONF_ZERO_OFFSET]))
    if CONF_DIRECTION_INVERTED in config:
        cg.add(comp_var.set_direction_inverted(config[CONF_DIRECTION_INVERTED]))
    if CONF_VELOCITY_FILTER_CUTOFF in config:
        cg.add(comp_var.set_velocity_filter_cutoff(config[CONF_VELOCITY_FILTER_CUTOFF]))

    main_sensor_config = config[CONF_MAIN_ANGLE_SENSOR]
    main_sensor_var = cg.new_Pvariable(main_sensor_config[CONF_ID])
    await sensor.register_sensor(main_sensor_var, main_sensor_config)
    cg.add(comp_var.set_angle_sensor(main_sensor_var))

    # Standard optional sensors
    if CONF_ACCUMULATED_ANGLE in config:
        sens_var = await sensor.new_sensor(config[CONF_ACCUMULATED_ANGLE])
        cg.add(comp_var.set_accumulated_angle_sensor(sens_var))
    if CONF_VELOCITY_RPM in config:
        sens_var = await sensor.new_sensor(config[CONF_VELOCITY_RPM])
        cg.add(comp_var.set_velocity_rpm_sensor(sens_var))
    if CONF_RAW_COUNT in config:
        sens_var = await sensor.new_sensor(config[CONF_RAW_COUNT])
        cg.add(comp_var.set_raw_count_sensor(sens_var))
    if CONF_RAW_RADIANS in config:
        sens_var = await sensor.new_sensor(config[CONF_RAW_RADIANS])
        cg.add(comp_var.set_raw_radians_sensor(sens_var))
    if CONF_ACCUMULATED_COUNT in config:
        sens_var = await sensor.new_sensor(config[CONF_ACCUMULATED_COUNT])
        cg.add(comp_var.set_accumulated_count_sensor(sens_var))
    if CONF_ACCUMULATED_RADIANS in config:
        sens_var = await sensor.new_sensor(config[CONF_ACCUMULATED_RADIANS])
        cg.add(comp_var.set_accumulated_radians_sensor(sens_var))

    # SSI-specific optional sensors / binary_sensor
    # Corresponding C++ setter methods will be needed:
    # - void set_magnetic_field_status_sensor(esphome::sensor::Sensor *sensor);
    # - void set_loss_of_track_status_sensor(esphome::sensor::Sensor *sensor);
    # - void set_push_button_ssi_binary_sensor(esphome::binary_sensor::BinarySensor *bsensor);
    if CONF_MAGNETIC_FIELD_STATUS in config:
        sens_var = await sensor.new_sensor(config[CONF_MAGNETIC_FIELD_STATUS])
        cg.add(comp_var.set_magnetic_field_status_sensor(sens_var))
    if CONF_LOSS_OF_TRACK_STATUS in config:
        sens_var = await sensor.new_sensor(config[CONF_LOSS_OF_TRACK_STATUS])
        cg.add(comp_var.set_loss_of_track_status_sensor(sens_var))
    if CONF_PUSH_BUTTON_SSI in config:
        bsens_var = await binary_sensor.new_binary_sensor(config[CONF_PUSH_BUTTON_SSI])
        cg.add(comp_var.set_push_button_ssi_binary_sensor(bsens_var))