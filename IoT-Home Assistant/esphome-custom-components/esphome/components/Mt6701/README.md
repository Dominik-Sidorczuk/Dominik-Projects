# ESPhome Custom Component: MT6701 Magnetic Rotary Encoder

This custom component integrates the MagnTek MT6701 14-bit on-axis magnetic rotary position sensor with ESPhome. It allows you to read absolute angular position, accumulated rotation, velocity, and diagnostic information from the sensor using either I²C or a 3-wire SSI (SPI-like) interface.

This component is designed for applications requiring precise and high-resolution rotational measurements, such as motor control, robotics, user input dials, and other mechatronic systems.

## Key Features

* **High-Resolution Absolute Angle:** 14-bit resolution (16384 counts per revolution) for precise 0-360° readings.
* **Accumulated Tracking:** Tracks total rotation (degrees, radians, raw counts) across multiple turns.
* **Velocity Measurement:** Calculates rotational speed in Revolutions Per Minute (RPM) with an optional configurable EMA low-pass filter.

* **Dual Interface Support:**
    * **I²C Interface:** Simple communication for angle reading.
    * **SSI (Synchronous Serial Interface):** 3-wire SPI-like interface for angle data, diagnostic status, and CRC.
* **SSI Diagnostic Information:** Magnetic field strength, loss of track status, push-button detection, and CRC-6/ITU data integrity check.
* **Highly Configurable:** Customize zero offset, direction of rotation, SSI clock speed, update interval, and exposed sensor entities.

## Supported Interfaces

* **I²C:** Default address `0x06`.
* **SSI (Synchronous Serial Interface):** Operates in **SPI Mode 2** (CPOL=1, CPHA=0). Requires `cs_pin`, `clk_pin`, `miso_pin`.

### Method 1: External Components (Recommended)

1.  Add the following to your device's YAML configuration file (e.g., `your_device.yaml`):
    ```yaml
    external_components:
      - source: github://Dominik-Sidorczuk/Dominik-Projects/IoT-Home%20Assistant/External%20Components@main
        components: [ mt6701 ]
    ```

    Alternatively, you can use the more verbose configuration format, which can be clearer for complex paths:
    ```yaml
    external_components:
      - source:
          type: github
          username: Dominik-Sidorczuk
          repo: Dominik-Projects
          ref: main 
          path: "IoT-Home Assistant/External Components"
        components: [ mt6701 ]
    ```

2.  Configure the `mt6701` sensor platform as per the [Configuration Details](#configuration-details) section.

### Method 2: Manual Installation

1.  Create a directory named `custom_components/mt6701/` within your main ESPhome configuration directory.
2.  Copy the following component files from the [GitHub repository](https://github.com/Dominik-Sidorczuk/Dominik-Projects/tree/main/IoT-Home%20Assistant/External%20Components/Mt6701) into the newly created `custom_components/mt6701/` directory:
    * `sensor.py`
    * `mt6701.h`
    * `mt6701.cpp`
3.  Configure the `mt6701` sensor platform as per the [Configuration Details](#configuration-details) section.

    **Note:** This method is less recommended as it does not provide automatic component updates.

## Configuration Details

This section provides a detailed explanation of all configuration parameters for the `mt6701` sensor platform.

The basic structure of the component configuration is as follows:

```yaml
# This defines a sensor platform based on the mt6701 component
sensor:
  - platform: mt6701
    id: my_mt6701_sensor      # Optional: A unique ID for this MT6701 component instance.
    update_interval: 50ms     # Optional, defaults to 60s. Recommended: 10ms-100ms for encoders.
                              # This determines how frequently all sensor data is read and processed.

    # --- Operational Parameters (see "Parameters" section below for details) ---
    zero_offset_degrees: "0.0deg"  # Optional, default "0.0deg". Adjusts the reported zero position.
    direction_inverted: false    # Optional, default false. Inverts the direction of rotation.
    velocity_filter_type: "EMA"  # Optional, default "EMA". Options: NONE, EMA, BUTTERWORTH_2ND_ORDER.
    velocity_filter_cutoff_frequency: "10Hz" # Optional, default "10Hz". Cutoff for the velocity filter.
    min_velocity_update_period: "1ms" # Optional, default "1ms". Minimum dt for velocity calculation.

    # --- Interface Configuration (REQUIRED - choose ONE: i2c OR ssi block) ---
    interface:
      # Option 1: I2C Interface Configuration Block (Example)
      # If using I2C, uncomment and configure this block.
      i2c:
        address: 0x06        # Optional. Defaults to 0x06, the standard MT6701 I2C address.
        # i2c_id: bus_a      # Optional. Use if you have multiple I2C buses and need to specify one.

      # Option 2: SSI (SPI) Interface Configuration Block (Example)
      # If using SSI, comment out the i2c block and uncomment/configure this ssi block.
      # ssi:
      #   cs_pin: GPIO5            # REQUIRED for SSI: Chip Select pin.
      #   # spi_id: my_spi_bus     # Optional: Use a pre-defined global SPI bus.
      #   # If not using spi_id, you must define clk_pin and miso_pin here:
      #   # clk_pin: GPIO18
      #   # miso_pin: GPIO19
      #   # clock_speed: 4MHz      # Optional: Default is 4MHz. Max recommended is ~15MHz.
      #   # mode: 2                # Optional: Default is 2 (CPOL=1, CPHA=0). VERIFY FOR YOUR SENSOR!
                                 # Mode 3 (CPOL=1, CPHA=1) might be needed for rising edge sampling.

    # --- Sensor Entity Configurations (at least 'angle' is REQUIRED) ---
    # Define which sensor values you want to expose to Home Assistant.
    # The 'angle' sensor is mandatory. All others are optional.

    angle: # REQUIRED sensor entity
      name: "MT6701 Angle"        # REQUIRED: Name for this sensor in Home Assistant.
      # id: my_angle_sensor       # Optional: Local ID for this specific sensor entity.
      # accuracy_decimals: 1      # Optional: Number of decimal places (default defined in sensor.py).
      # icon: "mdi:rotate-right"  # Optional: Icon (default defined in sensor.py).

    # --- Optional Sensor Entities ---
    # Uncomment and configure any of these optional sensors as needed.
    # Refer to the "Available Sensor Entities" section for details on each.
    #
    # accumulated_angle:
    #   name: "MT6701 Total Rotation"
    #
    # velocity_rpm:
    #   name: "MT6701 Speed"
    #
    # raw_count:
    #   name: "MT6701 Raw Count"
    #
    # # ... and so on for:
    # # raw_radians, accumulated_count, accumulated_radians
    #
    # # SSI-specific sensors (only configure if using the 'ssi' interface):
    # # magnetic_field_status:
    # #   name: "MT6701 Mag Field Status"
    # #
    # # loss_of_track_status:
    # #   name: "MT6701 Track Status"
    # #
    # # push_button_ssi: # This will be a binary_sensor
    # #   name: "MT6701 Push Button"
    # #
    # # ssi_crc_error_count:
    # #   name: "MT6701 SSI CRC Errors"
```

## Parameters

These parameters are configured directly under the `platform: mt6701` block.

* **`id`** (Optional)
    * **Type:** `ID`
    * **Description:** Manually specify the ID for this MT6701 component instance. This allows you to reference the component from other parts of your ESPhome configuration, such as in lambdas or custom C++ code.
    * *Why change it?* If you need to interact with the component programmatically or have multiple MT6701 instances and need to distinguish them.
    * **Example:** `id: my_rotary_encoder`

* **`update_interval`** (Optional)
    * **Type:** `Time` (e.g., `50ms`, `1s`)
    * **Default:** `60s` (standard ESPhome polling component default)
    * **Description:** Specifies how often the sensor should be polled for new data.
    * *Why change it?* The default `60s` is very infrequent for a rotary encoder. For responsive tracking of rotation, you'll want a much shorter interval. For user interfaces or motor control, values between `10ms` and `100ms` are common. Shorter intervals increase ESP CPU load and bus traffic.
    * **Example:** `update_interval: 20ms`

* **`zero_offset_degrees`** (Optional)  * **Type:** `Angle` (string, e.g., `"0.0°"`, `"15.2deg"`, `"-0.1rad"`)
    * **Default:** `"0.0°"`
    * **Description:** Defines a custom zero (0°) position for the angle sensor readings. The raw angle from the sensor is adjusted by this offset before being reported. For example, if the sensor's raw reading is 30° and `zero_offset_degrees` is `"10°"`, the `angle` sensor will output 20°. If `zero_offset_degrees` is `"-10°"`, it will output 40°. The final angle is normalized to the 0-360° range.
    * *Why change it?* To align the sensor's reported zero position with a specific mechanical reference point on your device (e.g., when a knob pointer is at the "0" mark on a panel).
    * **Example:** `zero_offset_degrees: "45.0deg"`

* **`direction_inverted`** (Optional)
    * **Type:** `boolean` (`true` or `false`)
    * **Default:** `false`
    * **Description:** Inverts the reported direction of rotation. When `false` (default), clockwise rotation typically increases angle/count values. When `true`, clockwise rotation will decrease angle/count values (or vice-versa, depending on magnet orientation). This affects `angle` (after zero offset), all `accumulated_*` sensors, and `velocity_rpm`.
    * *Why change it?* If the sensor reports increasing values for counter-clockwise rotation (or vice-versa) and you want to standardize it or match a specific mechanical behavior.
    * **Example:** `direction_inverted: true`

* **`velocity_filter_type`** (Optional)
    * **Type:** `string` (one of `NONE`, `EMA`, `BUTTERWORTH_2ND_ORDER`)
    * **Default:** `"EMA"`
    * **Description:** Selects the type of low-pass filter applied to the `velocity_rpm` readings.
        * `NONE`: No filter is applied. Velocity readings will be raw and potentially noisy.
        * `EMA`: A first-order Exponential Moving Average filter. Provides basic smoothing.
        * `BUTTERWORTH_2ND_ORDER`: A second-order Butterworth filter. Offers better smoothing and a flatter passband compared to EMA, but is slightly more computationally intensive.
    * *Why change it?* To choose the desired level and type of smoothing for velocity readings based on application needs. `NONE` for immediate response, `EMA` for simple smoothing, `BUTTERWORTH_2ND_ORDER` for more advanced smoothing.
    * **Example:** `velocity_filter_type: "BUTTERWORTH_2ND_ORDER"`

* **`velocity_filter_cutoff_frequency`** (Optional)
    * **Type:** `Frequency` (string, e.g., `"10Hz"`, `"0.5Hz"`)
    * **Default:** `"10Hz"`
    * **Description:** Configures the cutoff frequency for the selected low-pass filter (`EMA` or `BUTTERWORTH_2ND_ORDER`) applied to the `velocity_rpm` readings. This helps to smooth out noisy velocity measurements. A lower frequency means more smoothing but slower response to changes in speed. This parameter has no effect if `velocity_filter_type` is `NONE`. Set to `"0Hz"` to effectively disable the selected filter (though setting type to `NONE` is preferred).
    * *Why change it?* To adjust the responsiveness versus smoothness of the RPM readings. If RPM values are jumpy, lower the frequency. If the RPM reading lags too much behind actual speed changes, increase it.
    * **Example:** `velocity_filter_cutoff_frequency: "5Hz"`

* **`min_velocity_update_period`** (Optional)
    * **Type:** `Time Period` (string, e.g., `"1ms"`, `"500us"`)
    * **Default:** `"1ms"`
    * **Description:** Specifies the minimum time delta (dt) between raw angle readings that will be used for velocity calculation. If the actual time since the last reading is less than this value, the velocity will not be recalculated in that cycle (it will either report the previous filtered value or 0 if it's the first sample). This prevents issues with division by a very small `dt` which can lead to extremely noisy or erroneous velocity spikes, especially with very short `update_interval` values.
    * *Why change it?* If you are using a very fast `update_interval` (e.g., sub-millisecond) and observe velocity spikes, you might slightly increase this value. For most use cases, the default of `1ms` should be adequate.
    * **Example:** `min_velocity_update_period: "2ms"`

#### `i2c` Interface Block

Use this block if you are connecting the MT6701 via the I²C interface.

```yaml
# Inside the 'interface:' block:
i2c:
  address: 0x06       # Optional. Defaults to 0x06.
  i2c_id:             # Optional. Specify if using a non-default I2C bus.
```

### Sensor Entity Configuration

For each piece of data you want to expose from the MT6701 to Home Assistant, you need to configure a sensor entity block under the `platform: mt6701` block.

**Common Configuration for All Sensor Entities:**

* **`name`** (Required)
    * **Type:** `string`
    * **Description:** The name of the sensor as it will appear in Home Assistant.
    * *Why change it?* To give a descriptive and unique name for this sensor.
    * **Example:** `name: "My Encoder Angle"`

* **`id`** (Optional)
    * **Type:** `ID`
    * **Description:** An ID for this specific sensor entity, allowing you to reference it in ESPhome automations, lambdas, or other components.
    * *Why change it?* If you need to programmatically access this sensor's state or attributes within ESPhome.
    * **Example:** `id: encoder_angle_deg`

* **Other Standard Sensor Options:**
    You can also use standard ESPhome sensor options within each entity block, such as:
    * `accuracy_decimals` (integer): Number of decimal places for the sensor's state.
    * `unit_of_measurement` (string): Overrides the default unit (rarely needed here as defaults are sensible).
    * `icon` (string): Sets a custom icon in Home Assistant.
    * `filters`: Apply various filters (e.g., `median`, `throttle`, `lambda`) to the sensor values.
    * `on_value`, `on_raw_value`: For automations based on sensor values.

**Available Sensor Entities:**

* **`angle`** (Sensor, Required)
    * **Description:** The primary absolute rotational angle, adjusted by `zero_offset` and `direction_inverted`.
    * **Unit:** Degrees (`°`)
    * **Range:** Typically `0.0` to `<360.0` (after `zero_offset`).
    * **Configuration Example under `platform: mt6701`:**
        ```yaml
        angle: # This block name 'angle' enables this specific sensor
          name: "Knob Position" # REQUIRED: Name in Home Assistant
          accuracy_decimals: 1  # Optional: Display with 1 decimal place
          filters:              # Optional: Apply standard ESPhome filters
            - debounce: 0.1s    # Example: only update if value is stable for 0.1s
        ```

* **`accumulated_angle`** (Sensor, Optional)
    * **Description:** Unwrapped angle, tracking multiple full rotations. Affected by `zero_offset` (for its initial value) and `direction_inverted`. Useful for tracking total rotation distance.
    * **Unit:** Degrees (`°`)
    * **Configuration Example under `platform: mt6701`:**
        ```yaml
        accumulated_angle: # This block name enables this sensor
          name: "Total Shaft Rotation" # REQUIRED: Name in Home Assistant
          accuracy_decimals: 0         # Optional: Display without decimals
        ```

* **`velocity_rpm`** (Sensor, Optional)
    * **Description:** Calculated rotational speed. Affected by `direction_inverted`. Smoothed by the `velocity_filter_cutoff_frequency` if enabled.
    * **Unit:** Revolutions Per Minute (`RPM`)
    * **Configuration Example under `platform: mt6701`:**
        ```yaml
        velocity_rpm: # This block name enables this sensor
          name: "Motor Speed"           # REQUIRED: Name in Home Assistant
          icon: "mdi:fan"               # Optional: Custom icon
          accuracy_decimals: 1          # Optional: Display with 1 decimal place
        ```

* **`raw_count`** (Sensor, Optional)
    * **Description:** The raw 14-bit digital output directly from the encoder (0-16383). Not affected by `zero_offset` or `direction_inverted`. Useful for direct access to the sensor's fundamental output or for custom calculations.
    * **Unit:** None (integer count)
    * **Range:** `0` to `16383`
    * **Configuration Example under `platform: mt6701`:**
        ```yaml
        raw_count: # This block name enables this sensor
          name: "Encoder Raw Value" # REQUIRED: Name in Home Assistant
        ```

* **`raw_radians`** (Sensor, Optional)
    * **Description:** Angle in radians, directly calculated from `raw_count`. Not affected by `zero_offset` or `direction_inverted`.
    * **Unit:** Radians (`rad`)
    * **Range:** Approximately `0` to `$2\pi$` (0 to ~6.283)
    * **Configuration Example under `platform: mt6701`:**
        ```yaml
        raw_radians: # This block name enables this sensor
          name: "Angle in Radians"     # REQUIRED: Name in Home Assistant
          accuracy_decimals: 3        # Optional: Display with 3 decimal places
        ```

* **`accumulated_count`** (Sensor, Optional)
    * **Description:** Unwrapped raw count, tracking total movement in terms of raw sensor steps. Affected by `direction_inverted`.
    * **Unit:** None (integer count)
    * **Configuration Example under `platform: mt6701`:**
        ```yaml
        accumulated_count: # This block name enables this sensor
          name: "Total Raw Steps" # REQUIRED: Name in Home Assistant
        ```

* **`accumulated_radians`** (Sensor, Optional)
    * **Description:** Unwrapped angle in radians, tracking multiple full rotations. Affected by `direction_inverted`.
    * **Unit:** Radians (`rad`)
    * **Configuration Example under `platform: mt6701`:**
        ```yaml
        accumulated_radians: # This block name enables this sensor
          name: "Total Rotation Radians" # REQUIRED: Name in Home Assistant
          accuracy_decimals: 2          # Optional: Display with 2 decimal places
        ```
## Full Configuration Examples

Below are complete, heavily commented examples demonstrating how to use the MT6701 component with both I²C and SSI interfaces. You can adapt these for your specific ESPhome device configuration.

### Example 1: I²C Interface Configuration

This example shows a common setup using the I²C interface, including several optional sensors and filters.

```yaml
# This is an example configuration for the MT6701 sensor using the I2C interface.
# It demonstrates common settings and a few optional sensor entities.

# Substitute your ESP board type if not esp32.
# For ESP8266, you might use 'esp8266: board: d1_mini' for example.
esp32:
  board: esp32dev
  framework:
    type: esp-idf # Or 'arduino' if preferred

# Enable logging for debugging.
# In production, you might set the level to INFO or WARNING for less verbose output.
logger:
  level: DEBUG

# Define the I2C bus. These are common default pins for many ESP32 boards.
# Adjust sda and scl pins if your board uses different ones.
# For ESP8266, default I2C pins are usually D2 (SDA) and D1 (SCL).
i2c:
  sda: GPIO21      # Data pin for I2C communication.
  scl: GPIO22      # Clock pin for I2C communication.
  scan: true        # Recommended during setup: logs found I2C devices at ESP startup,
                    # which helps verify the MT6701 is connected and detected at the expected address.
  id: bus_i2c_main  # Optional ID. Useful if you have multiple I2C buses or want to reference this bus explicitly.

# MT6701 Sensor Platform Configuration
sensor:
  - platform: mt6701
    id: my_i2c_encoder # Optional: A unique ID for this component instance for easier referencing.

    # --- Interface Configuration: Using I2C ---
    interface:
      i2c:
        address: 0x06          # Optional. Defaults to 0x06, the standard MT6701 I2C address.
                               # Only change this if your specific sensor module has a different address.
        i2c_id: bus_i2c_main   # Optional. Specify which I2C bus to use if you have multiple defined.
                               # This matches the ID from the 'i2c:' block above.

    # --- General Operational Parameters ---
    update_interval: 30ms      # How often to read data from the sensor. 30ms provides fairly responsive feedback.
                               # For faster UI updates or control loops, you might go down to 10-20ms.
                               # For slower changing values, you can increase this to reduce CPU load (e.g., 100ms, 250ms).

    zero_offset_degrees: "-10.0deg" # Optional, default "0.0deg". Calibrates the sensor's 'zero' reading.
                                  # Example: If your physical knob points to its "0" mark, but the sensor's raw angle is 10 degrees,
                                  # setting 'zero_offset_degrees' to "-10.0deg" (or "350.0deg") will make the 'angle' sensor report 0.

    direction_inverted: false    # Optional, default 'false'. Set to 'true' if the angle/count increases
                                 # when you turn the knob counter-clockwise (or otherwise opposite to your expectation).
                                 # Default (false) usually means clockwise rotation results in an increasing angle.

    # Velocity calculation and filtering parameters
    velocity_filter_type: "BUTTERWORTH_2ND_ORDER"  # Optional, default "EMA". Filter type for RPM.
                                                   # Options: "NONE", "EMA", "BUTTERWORTH_2ND_ORDER".
                                                   # "BUTTERWORTH_2ND_ORDER" provides smoother results.

    velocity_filter_cutoff_frequency: "8Hz" # Optional, default "10Hz". Cutoff for the selected velocity filter (EMA or Butterworth).
                                            # Lower values = more smoothing, but also more lag in response to speed changes.
                                            # Higher values = less smoothing, quicker response, but potentially more "jittery" readings.
                                            # This parameter has no effect if 'velocity_filter_type' is "NONE".

    min_velocity_update_period: "2ms"   # Optional, default "1ms". Minimum time delta (dt) between raw angle readings
                                        # used for velocity calculation. Prevents issues with division by a very small dt,
                                        # which can lead to extremely noisy velocity spikes, especially with short 'update_interval' values.

    # --- Main Angle Sensor (This 'angle:' block is REQUIRED) ---
    angle:
      name: "Rotary Encoder Angle (I2C)" # Name that will be displayed in Home Assistant for this sensor.
      id: encoder_angle_i2c             # Optional ID. Useful if you want to refer to this sensor
                                        # from other parts of your ESPhome YAML (e.g., in automations or scripts).
      accuracy_decimals: 1              # Display the angle with 1 decimal place in Home Assistant (e.g., 123.5°).
      # Example of using standard ESPhome filters to reduce noise or update frequency:
      filters:
        - or: # This filter combination reports a new value if EITHER condition is met:
            - delta: 0.5 # Condition 1: Report if the angle has changed by at least 0.5 degrees since the last report.
            - throttle: 1s # Condition 2: Or, report the current value at least every 1 second,
                           # even if it hasn't changed by 0.5 degrees. This ensures HA gets periodic updates.

    # --- Optional Sensor Entities (Uncomment and configure any others you need) ---
    # These provide additional views of the sensor data.
    accumulated_angle:
      name: "Rotary Total Rotation (I2C)"
      accuracy_decimals: 0 # Display total accumulated degrees without decimal places.

    velocity_rpm:
      name: "Rotary Speed (I2C)"
      accuracy_decimals: 1           # Display RPM with 1 decimal place.
      icon: "mdi:speedometer"        # Example of a common icon for speed.

    raw_count:
      name: "Rotary Raw Count (I2C)"
      # This sensor provides the raw 14-bit value (0-16383) directly from the encoder.
      # It's not affected by zero_offset_degrees or direction_inverted. Useful for debugging or custom calculations.

    # Further optional sensors (uncomment to enable):
    # raw_radians:
    #   name: "Rotary Raw Radians (I2C)"
    #   accuracy_decimals: 3
    #
    # accumulated_count:
    #   name: "Rotary Total Raw Counts (I2C)"
    #
    # accumulated_radians:
    #   name: "Rotary Total Radians (I2C)"
    #   accuracy_decimals: 2

    # Note: SSI-specific diagnostic sensors (like magnetic_field_status, loss_of_track_status, 
    # push_button_ssi, ssi_crc_error_count) are NOT available when using the I2C interface
    # as I2C communication with MT6701 typically only provides angle data.
```

### Example 2: SSI Interface Configuration

This example shows a setup using the SSI interface, which allows for potentially faster updates and provides access to diagnostic information.

```yaml
# This is an example configuration for the MT6701 sensor using the SSI interface.
# SSI allows for potentially faster updates and provides access to diagnostic information.

# Substitute your ESP board type if not esp32
esp32:
  board: esp32dev
  framework:
    type: esp-idf # Or 'arduino'

# Enable logging for debugging.
logger:
  level: DEBUG

# Define the SPI bus globally if you plan to use `spi_id` in the component configuration.
# This is optional; you can define clk_pin and miso_pin directly in the ssi block instead.
# Ensure these pins match your ESP board's SPI capabilities and your wiring.
spi:
  clk_pin: GPIO18     # ESP's SPI Clock pin (SCK).
  miso_pin: GPIO19    # ESP's SPI MISO (Master In, Slave Out) pin. MT6701's DO connects here.
  # mosi_pin: GPIO23  # MOSI (Master Out, Slave In) is NOT used by the MT6701 for SSI read-only communication,
                      # but it's often included when defining a full SPI bus for other potential devices.
  id: bus_spi_custom  # An ID for this SPI bus configuration, to be referenced by `spi_id` in the component.

# MT6701 Sensor Platform Configuration
sensor:
  - platform: mt6701
    # --- Interface Configuration: Using SSI ---
    interface:
      ssi:
        cs_pin: GPIO5         # REQUIRED for SSI: The Chip Select pin connected to MT6701's CSN.
                              # The ESP will pull this pin low to enable communication with the sensor.

        spi_id: bus_spi_custom # Instructs the component to use the CLK and MISO pins from the global
                              # SPI bus configuration named 'bus_spi_custom' (defined above).
                              # If you prefer not to use a global SPI bus, comment out 'spi_id'
                              # and define 'clk_pin' and 'miso_pin' directly here:
                              # clk_pin: GPIO18  # ESP's SPI Clock pin.
                              # miso_pin: GPIO19 # ESP's SPI MISO pin.

        clock_speed: 8MHz     # SSI clock speed. Default is 4MHz if not specified.
                              # The MT6701 sensor supports up to ~15.6MHz. Higher speeds allow for faster data reads,
                              # which can be beneficial for very small `update_interval` values.
                              # However, higher speeds are more susceptible to noise and signal integrity issues,
                              # potentially requiring shorter wires and careful PCB layout.
                              # If you encounter errors or inconsistent data with SSI, try reducing this speed
                              # (e.g., to 1MHz, 2MHz, or 4MHz).

    # --- General Operational Parameters ---
    update_interval: 15ms   # SSI can often support faster updates than I2C due to dedicated lines
                            # and potentially higher clock speeds. 15ms provides very responsive feedback.

    zero_offset: "90.0deg"  # Example: If the mechanical zero is at what the sensor reads as 270 degrees,
                            # an offset of "90deg" (or "-270deg") will correct it.

    direction_inverted: true # Example: If clockwise rotation decreases the angle, set this to true
                             # to make clockwise increase the angle.

    # velocity_filter_cutoff_frequency: "10Hz" # Default is "10Hz". Uncomment and change if needed
                                             # (e.g., "5Hz" for more smoothing, "15Hz" for less).

    # --- Main Angle Sensor (This 'angle:' block is REQUIRED) ---
    angle:
      name: "Rotary Encoder Angle (SSI)"
      id: encoder_angle_ssi # Optional ID for this specific sensor entity.
      accuracy_decimals: 2  # Display with higher precision, e.g., 123.45°.

    # --- Optional Sensor Entities (Common) ---
    velocity_rpm:
      name: "Rotary Speed (SSI)"
      accuracy_decimals: 0 # Example: Display RPM as a whole number.

    # --- Optional Sensor Entities (SSI Specific Diagnostics) ---
    # These sensors are only available when using the SSI interface and provide valuable
    # insights into the sensor's operational health and magnetic field conditions.

    magnetic_field_status:
      name: "Rotary Mag Field (SSI)"
      # This sensor reports the quality of the magnetic field detected by the MT6701:
      # - Value 0: Normal (Indicates optimal magnetic field strength; this is the desired state).
      # - Value 1: Magnetic field too strong (The magnet might be too close to the sensor or too powerful).
      # - Value 2: Magnetic field too weak (The magnet might be too far, too weak, or misaligned).
      # Consistently non-zero values here suggest a problem with the magnet setup.
      icon: "mdi:magnet" # Custom icon for this sensor.

    loss_of_track_status:
      name: "Rotary Track Status (SSI)"
      # This sensor reports if the MT6701 has lost track of the magnet's position:
      # - Value 0: Tracking OK (The sensor is confidently tracking the magnet's rotation).
      # - Value 1: Loss of Track (The sensor could not determine the magnet's position reliably in the last reading).
      # This might occur if the rotation is extremely fast (exceeding the sensor's tracking capability
      # for the given `update_interval` and `clock_speed`), or if there are severe magnetic field issues
      # (e.g., field too weak/strong, or magnet moving out of valid range).
      icon: "mdi:alert-circle-check-outline" # Custom icon.

    # For 'push_button_ssi', this configuration under 'platform: mt6701' will create a Binary Sensor entity.
    push_button_ssi:
      name: "Rotary Push Button (SSI)"
      # This binary sensor reflects the state of the `Mg[2]` bit in the SSI data stream.
      # This bit is typically used to detect an axial press of the magnet (pushing it towards the sensor IC),
      # if the mechanical setup allows for such movement and the magnet is designed for it.
      # States will be: ON (pressed), OFF (not pressed).
      device_class: "power" # Example: Use a "power" button like icon/representation in Home Assistant.
                            # Other device classes like "moving" or "door" might also be suitable depending on context.

    # You can also enable other common sensors like raw_count, accumulated_angle, etc., as shown in the I2C example.
    # For instance:
    # raw_count:
    #   name: "Rotary Raw Count (SSI)"