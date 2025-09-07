import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, spi, binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_NAME,
    CONF_DEVICE_CLASS,
    UNIT_DEGREES,
    UNIT_REVOLUTIONS_PER_MINUTE,
    UNIT_EMPTY,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    DEVICE_CLASS_BUTTON,
    CONF_CS_PIN,
)

# Przestrzeń nazw dla komponentu C++
mt6701_ns = cg.esphome_ns.namespace("esphome::mt6701")

# Definicja klasy C++ komponentu
MT6701SensorComponent = mt6701_ns.class_(
    "MT6701SensorComponent", cg.PollingComponent, i2c.I2CDevice, spi.SPIDevice
)

# Definicja enuma C++ dla FilterType (zastępuje VelocityFilterType)
FilterType = mt6701_ns.enum("FilterType", is_class=True)
FILTER_TYPES_MAP = {
    "NONE": FilterType.NONE,
    "EMA": FilterType.EMA,
    "BUTTERWORTH_2ND_ORDER": FilterType.BUTTERWORTH_2ND_ORDER,
}

# Stałe dla jednostek i ikon
UNIT_RADIANS = "rad"
ICON_ANGLE_DEGREES = "mdi:rotate-right"
ICON_ACCUMULATED_ANGLE = "mdi:sigma"
ICON_VELOCITY_RPM = "mdi:speedometer"
ICON_RAW_COUNT = "mdi:counter"
ICON_FILTERED_COUNT = "mdi:filter-variant" # Ikona dla przefiltrowanej wartości kąta
ICON_RAW_RADIANS = "mdi:angle-acute"
ICON_ACCUMULATED_COUNT = "mdi:counter"
ICON_ACCUMULATED_RADIANS = "mdi:angle-acute"
ICON_MAGNETIC_FIELD_STATUS = "mdi:magnet-on"
ICON_LOSS_OF_TRACK_STATUS = "mdi:radar"
ICON_PUSH_BUTTON = "mdi:button-pointer"
ICON_SSI_CRC_ERROR_COUNT = "mdi:alert-octagon"

# Klucze konfiguracyjne
CONF_I2C = "i2c"
CONF_SSI = "ssi"
CONF_ZERO_OFFSET_DEGREES = "zero_offset_degrees"
CONF_DIRECTION_INVERTED = "direction_inverted"

# Nowe klucze konfiguracyjne dla filtrowania kąta
CONF_ANGLE_FILTER_TYPE = "angle_filter_type"
CONF_ANGLE_FILTER_CUTOFF_FREQUENCY = "angle_filter_cutoff_frequency"
CONF_ANGLE_EMA_ALPHA = "angle_ema_alpha"

# Klucze konfiguracyjne dla filtrowania prędkości i akumulacji RPM
CONF_VELOCITY_FILTER_TYPE = "velocity_filter_type" # Użyje nowego FilterType
CONF_VELOCITY_FILTER_CUTOFF_FREQUENCY = "velocity_filter_cutoff_frequency"
CONF_RPM_ACCUMULATION_SAMPLES = "rpm_accumulation_samples"
CONF_MIN_VELOCITY_UPDATE_PERIOD = "min_velocity_update_period" # Pozostaje bez zmian

# Klucze dla poszczególnych sensorów
CONF_ANGLE = "angle"
CONF_ACCUMULATED_ANGLE = "accumulated_angle"
CONF_VELOCITY_RPM = "velocity_rpm"
CONF_RAW_COUNT = "raw_count"
CONF_FILTERED_ANGLE_COUNT = "filtered_angle_count" # Nowy sensor
CONF_RAW_RADIANS = "raw_radians"
CONF_ACCUMULATED_COUNT = "accumulated_count"
CONF_ACCUMULATED_RADIANS = "accumulated_radians"
CONF_MAGNETIC_FIELD_STATUS = "magnetic_field_status"
CONF_LOSS_OF_TRACK_STATUS = "loss_of_track_status"
CONF_SSI_CRC_ERROR_COUNT = "ssi_crc_error_count"
CONF_PUSH_BUTTON_SSI = "push_button_ssi"

AUTO_LOAD = ["i2c", "spi"]

# Schematy dla sensorów
# Dodajemy schemat dla nowego sensora filtered_angle_count
COMMON_SENSOR_SCHEMAS = {
    CONF_ANGLE: sensor.sensor_schema(
        unit_of_measurement=UNIT_DEGREES,
        icon=ICON_ANGLE_DEGREES,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_ACCUMULATED_ANGLE: sensor.sensor_schema(
        unit_of_measurement=UNIT_DEGREES,
        icon=ICON_ACCUMULATED_ANGLE,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_VELOCITY_RPM: sensor.sensor_schema(
        unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
        icon=ICON_VELOCITY_RPM,
        accuracy_decimals=1, # Może wymagać dostosowania w zależności od stabilności
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_RAW_COUNT: sensor.sensor_schema(
        unit_of_measurement=UNIT_EMPTY, # Lub "counts"
        icon=ICON_RAW_COUNT,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_FILTERED_ANGLE_COUNT: sensor.sensor_schema( # Nowy sensor
        unit_of_measurement=UNIT_EMPTY, # Lub "counts"
        icon=ICON_FILTERED_COUNT,
        accuracy_decimals=2, # Wartość po filtrze może być float
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_RAW_RADIANS: sensor.sensor_schema(
        unit_of_measurement=UNIT_RADIANS,
        icon=ICON_RAW_RADIANS,
        accuracy_decimals=3,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_ACCUMULATED_COUNT: sensor.sensor_schema(
        unit_of_measurement=UNIT_EMPTY,
        icon=ICON_ACCUMULATED_COUNT,
        accuracy_decimals=0,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    CONF_ACCUMULATED_RADIANS: sensor.sensor_schema(
        unit_of_measurement=UNIT_RADIANS,
        icon=ICON_ACCUMULATED_RADIANS,
        accuracy_decimals=2,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    CONF_MAGNETIC_FIELD_STATUS: sensor.sensor_schema(
        unit_of_measurement=UNIT_EMPTY,
        icon=ICON_MAGNETIC_FIELD_STATUS,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_LOSS_OF_TRACK_STATUS: sensor.sensor_schema(
        unit_of_measurement=UNIT_EMPTY,
        icon=ICON_LOSS_OF_TRACK_STATUS,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}

SSI_ONLY_SENSOR_SCHEMAS = {
    CONF_SSI_CRC_ERROR_COUNT: sensor.sensor_schema(
        unit_of_measurement=UNIT_EMPTY,
        icon=ICON_SSI_CRC_ERROR_COUNT,
        accuracy_decimals=0,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
}

SSI_ONLY_BINARY_SENSOR_SCHEMAS = {
    CONF_PUSH_BUTTON_SSI: binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_BUTTON
    ),
}

I2C_INTERFACE_SCHEMA = i2c.i2c_device_schema(default_address=0x06)
SSI_INTERFACE_SCHEMA = spi.spi_device_schema(cs_pin_required=True).extend(
    {
        **{cv.Optional(key): schema for key, schema in SSI_ONLY_SENSOR_SCHEMAS.items()},
        **{cv.Optional(key): schema for key, schema in SSI_ONLY_BINARY_SENSOR_SCHEMAS.items()},
    }
)

# Główny schemat konfiguracji
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MT6701SensorComponent),
        cv.Exclusive(CONF_I2C, "interface_type_group"): I2C_INTERFACE_SCHEMA,
        cv.Exclusive(CONF_SSI, "interface_type_group"): SSI_INTERFACE_SCHEMA,
        
        # Podstawowe parametry
        cv.Optional(CONF_ZERO_OFFSET_DEGREES, default="0.0deg"): cv.angle,
        cv.Optional(CONF_DIRECTION_INVERTED, default=False): cv.boolean,
        
        # Parametry filtrowania kąta
        cv.Optional(CONF_ANGLE_FILTER_TYPE, default="EMA"): cv.enum(FILTER_TYPES_MAP, upper=True),
        cv.Optional(CONF_ANGLE_FILTER_CUTOFF_FREQUENCY, default="20Hz"): cv.frequency,
        cv.Optional(CONF_ANGLE_EMA_ALPHA, default=0.5): cv.positive_float,

        # Parametry filtrowania prędkości i akumulacji RPM
        cv.Optional(CONF_VELOCITY_FILTER_TYPE, default="EMA"): cv.enum(FILTER_TYPES_MAP, upper=True),
        cv.Optional(CONF_VELOCITY_FILTER_CUTOFF_FREQUENCY, default="10Hz"): cv.frequency,
        cv.Optional(CONF_RPM_ACCUMULATION_SAMPLES, default=5): cv.positive_int,
        cv.Optional(CONF_MIN_VELOCITY_UPDATE_PERIOD, default="1ms"): cv.positive_time_period_microseconds,
        
        # Definicje sensorów
        **{cv.Optional(key): schema for key, schema in COMMON_SENSOR_SCHEMAS.items()},
    }
).extend(cv.polling_component_schema("50ms")) # Domyślny interwał odpytywania, można dostosować


async def to_code(config):
    comp_var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(comp_var, config)

    is_ssi_interface = False
    if CONF_I2C in config:
        await i2c.register_i2c_device(comp_var, config[CONF_I2C])
        cg.add(comp_var.set_interface_i2c())
    elif CONF_SSI in config:
        is_ssi_interface = True
        await spi.register_spi_device(comp_var, config[CONF_SSI])
        cg.add(comp_var.set_interface_ssi())
        if any(bs_key in config[CONF_SSI] for bs_key in SSI_ONLY_BINARY_SENSOR_SCHEMAS):
            cg.add_global_define("MT6701_HAS_BINARY_SENSOR")
    else:
        raise cv.ValidationError("Nie określono prawidłowego typu interfejsu ('i2c' lub 'ssi').")

    # Ustawianie parametrów konfiguracyjnych
    parameter_map = [
        (CONF_ZERO_OFFSET_DEGREES, "set_zero_offset_degrees"),
        (CONF_DIRECTION_INVERTED, "set_direction_inverted"),
        # Filtrowanie kąta
        (CONF_ANGLE_FILTER_TYPE, "set_angle_filter_type"),
        (CONF_ANGLE_FILTER_CUTOFF_FREQUENCY, "set_angle_filter_cutoff_hz"),
        (CONF_ANGLE_EMA_ALPHA, "set_angle_ema_alpha"),
        # Filtrowanie prędkości i akumulacja RPM
        (CONF_VELOCITY_FILTER_TYPE, "set_velocity_filter_type"),
        (CONF_VELOCITY_FILTER_CUTOFF_FREQUENCY, "set_velocity_filter_cutoff_hz"),
        (CONF_RPM_ACCUMULATION_SAMPLES, "set_rpm_accumulation_samples"),
        (CONF_MIN_VELOCITY_UPDATE_PERIOD, "set_min_velocity_update_period_us"),
    ]
    for conf_key, setter_name in parameter_map:
        if conf_key in config:
            value_to_set = config[conf_key]
            cg.add(getattr(comp_var, setter_name)(value_to_set))

    # Mapowanie i rejestracja sensorów
    sensor_setter_map = {
        CONF_ANGLE: "set_angle_sensor",
        CONF_ACCUMULATED_ANGLE: "set_accumulated_angle_sensor",
        CONF_VELOCITY_RPM: "set_velocity_rpm_sensor",
        CONF_RAW_COUNT: "set_raw_count_sensor",
        CONF_FILTERED_ANGLE_COUNT: "set_filtered_angle_count_sensor", # Nowy sensor
        CONF_RAW_RADIANS: "set_raw_radians_sensor",
        CONF_ACCUMULATED_COUNT: "set_accumulated_count_sensor",
        CONF_ACCUMULATED_RADIANS: "set_accumulated_radians_sensor",
        CONF_MAGNETIC_FIELD_STATUS: "set_magnetic_field_status_sensor",
        CONF_LOSS_OF_TRACK_STATUS: "set_loss_of_track_status_sensor",
        CONF_SSI_CRC_ERROR_COUNT: "set_ssi_crc_error_sensor",
    }
    binary_sensor_setter_map = {
        CONF_PUSH_BUTTON_SSI: "set_push_button_ssi_binary_sensor",
    }

    # Rejestracja wspólnych sensorów (w tym nowego filtered_angle_count)
    for conf_key in COMMON_SENSOR_SCHEMAS: 
        if conf_key in config: 
            entity_config = config[conf_key]
            sens_var = await sensor.new_sensor(entity_config)
            cg.add(getattr(comp_var, sensor_setter_map[conf_key])(sens_var))

    # Rejestracja sensorów i binarnych sensorów tylko dla SSI
    if is_ssi_interface:
        ssi_config_block = config[CONF_SSI]
        for conf_key in SSI_ONLY_SENSOR_SCHEMAS:
            if conf_key in ssi_config_block:
                entity_config = ssi_config_block[conf_key]
                sens_var = await sensor.new_sensor(entity_config)
                cg.add(getattr(comp_var, sensor_setter_map[conf_key])(sens_var))
        
        for conf_key in SSI_ONLY_BINARY_SENSOR_SCHEMAS:
            if conf_key in ssi_config_block:
                entity_config = ssi_config_block[conf_key]
                bin_sens_var = await binary_sensor.new_binary_sensor(entity_config)
                cg.add(getattr(comp_var, binary_sensor_setter_map[conf_key])(bin_sens_var))
