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

// v7 cruise heat law (2026-10-22 fixed-wire D-threshold experiment, wohnzimmer):
// thermostat OFF when realized D = intake - b13 crosses ~+0.5..+1.0 (drifty:
// observed 0.0 / 0.5 / 1.0 across three sweeps), restart when D falls to
// ~0.0 (started 11 s into a hold at D=0). The > +2.0 K marker from the
// service manual is NOT the real stop point - stops happen far below it.
// Cruise target = LEAD (+0.5), just under the stop threshold, holding the
// unit on with gentle charging inside a ~1 K room band (start ~desired-0.5,
// stop ~desired..+0.5) - long cycles instead of the v6 flapping, which
// cruised above the stop threshold (D +0.5..+1.5).
static const float HEAT_GAP_WARM = 0.0f;     // Standing gap (b13 - intake) at target: 0 -> cruise D = LEAD for any B
static const float HEAT_LEAD = 0.5f;         // Cruise D = -E* at room == desired; also the overshoot back-off
static const float HEAT_E_MIN = -2.5f;       // E* floor: D = +2.5 terminal-OFF dive (room >= desired + 2.0)
static const float HEAT_E_MAX = 2.5f;        // E* cap: D = -2.5 max-heat dive (room <= desired - 3.0)
static const float HEAT_BOOST_SCALE = 4.0f;  // B clamp used by the law (schedule no longer used)
// Overshoot latch: set when the unit goes idle while the room is above the
// desired setpoint (it heated past the user's target). While latched the law
// is bypassed and the wire is pinned at 16 (the lowest the unit accepts),
// putting b13 far below any restart criterion (D ~ +5..+6) - a hard OFF hold.
// Cleared when the room dips to (desired - 0.5) or on any user setpoint/mode
// input (HA or remote).

// Set false to run WITHOUT the compensation law (threshold-experiment builds):
// the climate target passes straight through to the unit's b3, while the
// Setpoint calc log line keeps reporting room/intake/boost/wire/state/D/pw
// so external tooling can still observe and steer the unit.
static const bool COMPENSATION_ENABLED = true;

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
  bool overshoot_ = false;                 // Heat over-target latch (wire pinned at 16 while set)
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
  float compute_wire_setpoint();  // Law (+boost cancel) or plain fallback; wire space, NAN if desired unknown
  void note_unit_idle();   // Idle transition (CNT): latch overshoot when room over desired
  void note_user_change();  // User setpoint/mode input (HA or remote): clear the overshoot latch
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
