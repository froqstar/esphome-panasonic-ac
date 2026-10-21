#pragma once

#include <cmath>

#include "esphome/components/climate/climate.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

namespace esphome {

namespace panasonic_ac {

static const char *const VERSION = "2.6.0";

static const uint8_t BUFFER_SIZE = 128;  // The maximum size of a single packet (both receive and transmit)
static const uint8_t READ_TIMEOUT = 20;  // The maximum time to wait before considering a packet complete

static const uint8_t MIN_TEMPERATURE = 16;     // Minimum temperature as reported by Panasonic app
static const uint8_t MAX_TEMPERATURE = 30;     // Maximum temperature as supported by Panasonic app
static const float TEMPERATURE_STEP = 0.5;     // Steps the temperature can be set in
static const float TEMPERATURE_TOLERANCE = 2;  // The tolerance to allow when checking the climate state
static const uint8_t TEMPERATURE_THRESHOLD =
    100;  // Maximum temperature the AC can report before considering the temperature as invalid

static const uint32_t GRACE_MS = 6000;      // Grace after our own command masking echoes/stale state (1 poll + slack)
static const float SETPOINT_EPSILON = 0.25f;  // Echo/adopt tolerance; half a 0.5 step, absorbs only float noise

// v6 gap-scheduling heat law (service-manual derived, cross-gen consistent):
// thermostat OFF when D = intake - b13 > +2.0 K (quantized: first at +2.5),
// restart after 3 min while D < +2.0; frequency MAX zone at D <= -2.5;
// minimum/decrease zone once D >= -1.
static const float HEAT_GAP_WARM = -1.0f;    // Standing gap (b13 - intake) at B = 0 (warm weather, no boost)
static const float HEAT_LEAD = 0.5f;         // Overshoot back-off (K) subtracted from E* (thermal-lag knob)
static const float HEAT_E_MIN = -2.5f;       // E* floor: D = +2.5 > +2.0 manual OFF point (terminal OFF allowed)
static const float HEAT_E_MAX = 2.5f;        // E* cap = frequency MAX zone boundary (D = -2.5)
static const float HEAT_BOOST_SCALE = 4.0f;  // B at which the standing gap reaches 0 (also schedule clamp)

enum class CommandType { Normal, Response, Resend };

enum class ACType {
  DNSKP11,  // New module (via CN-WLAN)
  CZTACG1   // Old module (via CN-CNT)
};

class PanasonicAC : public Component, public uart::UARTDevice, public climate::Climate {
 public:
  void set_outside_temperature_sensor(sensor::Sensor *outside_temperature_sensor);
  void set_outside_temperature_offset(int8_t outside_temperature_offset);
  void set_vertical_swing_select(select::Select *vertical_swing_select);
  void set_horizontal_swing_select(select::Select *horizontal_swing_select);
  void set_nanoex_switch(switch_::Switch *nanoex_switch);
  void set_eco_switch(switch_::Switch *eco_switch);
  void set_econavi_switch(switch_::Switch *econavi_switch);
  void set_mild_dry_switch(switch_::Switch *mild_dry_switch);
  void set_current_power_consumption_sensor(sensor::Sensor *current_power_consumption_sensor);
  void set_internal_boost_sensor(sensor::Sensor *internal_boost_sensor);
  void set_defrost_sensor(binary_sensor::BinarySensor *defrost_sensor);

  void set_current_temperature_sensor(sensor::Sensor *current_temperature_sensor);
  void set_current_temperature_offset(int8_t current_temperature_offset);

  void setup() override;
  void loop() override;

 protected:
  sensor::Sensor *outside_temperature_sensor_ = nullptr;        // Sensor to store outside temperature from queries
  select::Select *vertical_swing_select_ = nullptr;             // Select to store manual position of vertical swing
  select::Select *horizontal_swing_select_ = nullptr;           // Select to store manual position of horizontal swing
  switch_::Switch *nanoex_switch_ = nullptr;                    // Switch to toggle nanoeX on/off
  switch_::Switch *eco_switch_ = nullptr;                       // Switch to toggle eco mode on/off
  switch_::Switch *econavi_switch_ = nullptr;                   // Switch to toggle econavi mode on/off
  switch_::Switch *mild_dry_switch_ = nullptr;                  // Switch to toggle mild dry mode on/off
  sensor::Sensor *current_temperature_sensor_ = nullptr;        // Sensor to use for current temperature where AC does not report
  sensor::Sensor *current_power_consumption_sensor_ = nullptr;  // Sensor to store current power consumption from queries
  sensor::Sensor *internal_boost_sensor_ = nullptr;  // Unit's internal setpoint boost (b13-b3)/2
  binary_sensor::BinarySensor *defrost_sensor_ = nullptr;       // Sensor to store defrost status

  size_t vertical_swing_state_;
  size_t horizontal_swing_state_;

  int8_t current_temperature_offset_ = 0;  // current temperature offset to compensate internal sensor values
  int8_t outside_temperature_offset_ = 0;  // outside temperature offset to compensate internal sensor values
  bool nanoex_state_ = false;    // Stores the state of nanoex to prevent duplicate packets
  bool eco_state_ = false;       // Stores the state of eco to prevent duplicate packets
  bool econavi_state_ = false;   // Stores the state of econavi to prevent duplicate packets
  bool mild_dry_state_ = false;  // Stores the state of mild dry to prevent duplicate packets

  // External-sensor setpoint compensation state (active iff current_temperature_sensor_ != nullptr)
  float room_temperature_ = NAN;           // Raw room sensor state (no offset) for the law
  float intake_temperature_ = NAN;         // Raw packet intake temperature for the law
  float internal_boost_ = NAN;             // Measured boost B = (b13 - b3)/2, always populated each poll
  float desired_setpoint_ = NAN;           // User/HA setpoint in display units (persisted)
  float last_sent_setpoint_ = NAN;         // Wire value we last staged (post-cancellation, actual byte)
  uint32_t last_command_time_ = 0;         // Grace anchor; 0 until first staged command
  bool has_sent_command_ = false;          // Boot guard: unit's retained setpoint not treated as user input
  ESPPreferenceObject desired_setpoint_pref_;  // Persist desired_setpoint_ across reboots

  // Last-poll diagnostics (CNT) appended to the Setpoint calc log
  uint8_t last_state_byte_ = 0;    // Frame byte 12: observed 0x4C running / 0x40 idle (kinderzimmer)
  float last_reported_gap_ = NAN;  // D = intake - b13 from this poll's frame (thermostat input)
  float last_power_w_ = NAN;       // Instantaneous power, bytes 28-30

  bool waiting_for_response_ = false;  // Set to true if we are waiting for a response

  // uint8_t receive_buffer_index = 0;     // Current position of the receive buffer
  // uint8_t receive_buffer[BUFFER_SIZE];  // Stores the packet currently being received

  std::vector<uint8_t> rx_buffer_;

  uint32_t init_time_;             // Stores the current time
  uint32_t last_read_;             // Stores the time at which the last read was done
  uint32_t last_packet_sent_;      // Stores the time at which the last packet was sent
  uint32_t last_packet_received_;  // Stores the time at which the last packet was received

  climate::ClimateTraits traits() override;

  void read_data();

  void update_outside_temperature(int8_t temperature);
  void update_current_temperature(int8_t temperature);
  void update_target_temperature(uint8_t raw_value);
  void update_swing_horizontal(const StringRef &swing);
  void update_swing_vertical(const StringRef &swing);
  void update_nanoex(bool nanoex);
  void update_eco(bool eco);
  void update_econavi(bool econavi);
  void update_mild_dry(bool mild_dry);
  void update_current_power_consumption(int16_t power);
  void update_internal_boost(float boost);
  void update_defrost(bool defrost);

  // Compensation helpers (CNT uses these; WLAN never activates compensation)
  bool compensation_active() const { return this->current_temperature_sensor_ != nullptr; }
  float compute_wire_setpoint() const;  // Law (+boost cancel) or plain fallback; wire space, NAN if desired unknown
  void maybe_restore_desired_setpoint();
  void save_desired_setpoint(float desired);
  virtual void on_room_change() {}  // Called when the room sensor reports; CNT overrides to sync

  virtual void on_horizontal_swing_change(const StringRef &swing) = 0;
  virtual void on_vertical_swing_change(const StringRef &swing) = 0;
  virtual void on_nanoex_change(bool nanoex) = 0;
  virtual void on_eco_change(bool eco) = 0;
  virtual void on_econavi_change(bool econavi) = 0;
  virtual void on_mild_dry_change(bool mild_dry) = 0;

  climate::ClimateAction determine_action();

  void log_packet(std::vector<uint8_t> data, bool outgoing = false);
};

}  // namespace panasonic_ac
}  // namespace esphome
