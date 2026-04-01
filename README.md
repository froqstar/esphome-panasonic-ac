# Overview

An open source alternative for the Panasonic wi-fi adapter that works locally without the cloud.

# Features

* Control your AC locally via Home Assistant, MQTT or directly
* Instantly control the AC without any delay like in the Comfort Cloud app
* Receive live reports and state from the AC
* Uses the UART interface on the AC instead of the IR interface
* Provides a drop-in replacement for the Panasonic DNSK-P11 and the CZ-TACG1 wifi module

# Supported hardware

This library works with both the CN-CNT port and the CN-WLAN port. CN-WLAN is only available on newer units. Either port can be used on units that have both ports regardless of the usage of the other port (ie. it is possible to leave the DNSK-P11 connected to CN-WLAN and connect the ESP to CN-CNT).

Works on the ESP8266 but ESP32 is preferred for the multiple hardware serial ports.

# Requirements

* ESP32 (or ESP8266) ([supported by ESPHome](https://esphome.io/#devices))
* 5V to 3.3V bi-directional Logic Converter (minimum 2 channels, available as pre-soldered prototyping boards)
* Female-Female Jumper cables
* Soldering iron
* Wires to solder from Logic converter to ESP
* Heat shrink
* ESPHome 2025.11.0 or newer
* Home Assistant 2025.11.0 or newer

# Notes

* **Make sure to disconnect mains power before opening your AC, the mains contacts are exposed and can be touched by accident!**
* **Do not connect your ESP32/ESP8266 directly to the AC, the AC uses 5V while the ESPs use 3.3V!**
* **While installation is fairly straightforward I do not take any responsibility for any damage done to you or your AC during installation**
* The DNSK-P11 and the CZ-TACG1 use different types of connectors, make sure to connect to the correct one

# Software installation

This software installation guide assumes some familiarity with ESPHome.

* Pull this repository or copy the `ac.yaml.example` from the root folder
* Rename the `ac.yaml.example` to `ac.yaml`
* Uncomment the `type` field depending on which AC protocol you want to use
* Adjust the `ac.yaml` to your needs
* Connect your ESP
* Run `esphome ac.yaml run` and choose your serial port (or do this via the Home Assistant UI)
* If you see the handshake messages being sent (DNSK-P11) or polling requests being sent (CZ-TACG1) in the log you are good to go
* Disconnect the ESP and continue with hardware installation

## Setting supported features

Since Panasonic ACs support different features you can comment out the lines at the bottom of your `ac.yaml`:

```
  # Enable as needed
  # eco_switch:
  #   name: Panasonic AC Eco Switch
  # nanoex_switch:
  #   name: Panasonic AC NanoeX Switch
  # mild_dry_switch:
  #   name: Panasonic AC Mild Dry Switch
  # econavi_switch:
  #   name: Econavi switch
  # current_power_consumption:
  #   name: Panasonic AC Power Consumption
```

In order to find out which features are supported by your AC, check the remote that came with it. Please note that eco switch and mild dry switch are not supported on DNSK-P11.

**Enabling unsupported features can lead to undefined behavior and may damage your AC. Make sure to check your remote or manual first.**
**current_power_consumption is just as ESTIMATED value by the AC**

## Setting temperature offsets

As the internal sensors reading might not reflect the actual temperature in the room or outside, you can optionally define a fixed offset for both sensors.
This offset is internally applied to both, the reported temperature in ESPHome/HomeAssistant as well as to the target temperature. Any shown values always include the defined offset.

- Temperature readings of the AC will be reported as (reading + offset)
- Target temperatures will be set as (target - offset)

```
    # Adapt according to your measurements
    current_temperature_offset: 0 
    outside_temperature_offset: 0
```

Examples:
- If the temperature is actually higher than measured by the AC, set the difference as a positive offset.
  - E.g. actual temperature = 23°, AC measured temperature = 20° --> offset = 3°
- If the temperature is actually lower than measured by the AC, set the difference as a negative offset.
  - E.g. actual temperature = 20°, AC measured temperature = 22° --> offset = -2°

## Using an external room temperature sensor

If you have a more accurate temperature sensor in your room (e.g., a separate Home Assistant sensor), you can configure the AC to use it for more precise temperature control. This is useful when the AC's internal sensor doesn't accurately reflect the actual room temperature due to its placement.

```yaml
sensor:
  - platform: homeassistant
    id: room_temp_sensor
    entity_id: sensor.living_room_temperature

climate:
  - platform: panasonic_ac
    type: cnt  # or wlan
    name: Panasonic AC
    room_sensor: room_temp_sensor
    # ... other configuration
```

When a room sensor is configured:

- Home Assistant displays your desired setpoint (the temperature you actually want in the room)
- The component automatically calculates and sends an adjusted internal setpoint to the AC using the formula:
  `internal_setpoint = internal_temperature + (desired_setpoint - room_temperature)`
- The setpoint is automatically recalculated whenever the room sensor, AC internal temperature, or your desired setpoint changes

**Example:** If your room sensor reads 22°C, you set the target to 24°C, and the AC's internal sensor reads 26°C, the component will send 28°C to the AC (26 + (24 - 22) = 28). This compensates for the difference between the AC's internal reading and the actual room temperature.

**Note:** The room sensor feature works independently of the temperature offset settings. If you use both, the offset is applied to the displayed temperature values, while the room sensor affects the actual setpoint sent to the AC.

# Hardware installation

[Hardware installation for DNSK-P11](README.DNSKP11.md)

[Hardware installation for CZ-TACG1](README.CZTACG1.md)
