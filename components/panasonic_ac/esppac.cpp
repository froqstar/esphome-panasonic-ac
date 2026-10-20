#include "esppac.h"

#include "esphome/core/log.h"

namespace esphome {
namespace panasonic_ac {

static const char *const TAG = "panasonic_ac";

climate::ClimateTraits PanasonicAC::traits() {
  auto traits = climate::ClimateTraits();

  traits.add_feature_flags(
      climate::CLIMATE_SUPPORTS_ACTION |
      climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE
  );

  traits.set_visual_min_temperature(MIN_TEMPERATURE);
  traits.set_visual_max_temperature(MAX_TEMPERATURE);
  traits.set_visual_temperature_step(TEMPERATURE_STEP);

  traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT_COOL, climate::CLIMATE_MODE_COOL,
                              climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_FAN_ONLY, climate::CLIMATE_MODE_DRY});

  traits.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_BOTH,
                                    climate::CLIMATE_SWING_VERTICAL, climate::CLIMATE_SWING_HORIZONTAL});

  return traits;
}

void PanasonicAC::setup() {
  // Initialize times
  this->init_time_ = millis();
  this->last_packet_sent_ = millis();

  this->set_supported_custom_fan_modes({"Automatic", "1", "2", "3", "4", "5"});
  this->set_supported_custom_presets({"Normal", "Powerful", "Quiet"});

  this->maybe_restore_desired_setpoint();

  ESP_LOGI(TAG, "Panasonic AC component v%s starting...", VERSION);
}

void PanasonicAC::loop() {
  read_data();  // Read data from UART (if there is any)
}

void PanasonicAC::read_data() {
  while (available())  // Read while data is available
  {
    // if (this->receive_buffer_index >= BUFFER_SIZE) {
    //   ESP_LOGE(TAG, "Receive buffer overflow");
    //   receiveBufferIndex = 0;
    // }

    uint8_t c;
    this->read_byte(&c);  // Store in receive buffer
    this->rx_buffer_.push_back(c);

    this->last_read_ = millis();  // Update lastRead timestamp
  }
}

void PanasonicAC::update_outside_temperature(int8_t temperature) {
  ESP_LOGV(TAG, "Received outside temperature %d", temperature);
  temperature += this->outside_temperature_offset_;

  if (temperature > TEMPERATURE_THRESHOLD) {
    ESP_LOGW(TAG, "Received out of range outside temperature: %d", temperature);
    return;
  }

  if (this->outside_temperature_sensor_ != nullptr && this->outside_temperature_sensor_->state != temperature) {
    this->outside_temperature_sensor_->publish_state(
        temperature);  // Set current (outside) temperature; no temperature steps
    ESP_LOGV(TAG, "Outside temperature incl. offset: %d", temperature);
  }
}

void PanasonicAC::update_current_temperature(int8_t temperature) {
  ESP_LOGV(TAG, "Received current temperature %d", temperature);

  if (temperature > TEMPERATURE_THRESHOLD) {
    ESP_LOGW(TAG, "Received out of range inside temperature: %d", temperature);
    return;
  }

  this->intake_temperature_ = temperature;  // RAW intake for the law

  if (!this->compensation_active()) {  // plain mode only (upstream display)
    temperature += this->current_temperature_offset_;
    this->current_temperature = temperature;
    ESP_LOGV(TAG, "Current temperature incl. offset: %d", temperature);
  }
}

void PanasonicAC::update_target_temperature(uint8_t raw_value) {
  float temperature = (raw_value * TEMPERATURE_STEP);
  ESP_LOGV(TAG, "Received target temperature %.2f", temperature);

  //Apply offset for displayed value
  temperature += this->current_temperature_offset_;

  if (temperature > TEMPERATURE_THRESHOLD) {
    ESP_LOGW(TAG, "Received out of range target temperature %.2f", temperature);
    return;
  }

  this->target_temperature = temperature;
  ESP_LOGV(TAG, "Target temperature incl. offset: %.2f", temperature);
}

void PanasonicAC::update_swing_horizontal(const StringRef &swing) {
  if (this->horizontal_swing_select_ != nullptr) {
    this->horizontal_swing_state_ = this->horizontal_swing_select_->index_of(swing).value_or(~0UL);

    if (this->horizontal_swing_state_ != this->horizontal_swing_select_->active_index().value_or(~0UL)) {
      this->horizontal_swing_select_->publish_state(this->horizontal_swing_state_);  // Set current horizontal swing position
    }
  }
}

void PanasonicAC::update_swing_vertical(const StringRef &swing) {
  if (this->vertical_swing_select_ != nullptr) {
    this->vertical_swing_state_ = this->vertical_swing_select_->index_of(swing).value_or(~0UL);

    if (this->vertical_swing_state_ != this->vertical_swing_select_->active_index().value_or(~0UL)) {
      this->vertical_swing_select_->publish_state(this->vertical_swing_state_);  // Set current vertical swing position
    }
  }
}

void PanasonicAC::update_nanoex(bool nanoex) {
  if (this->nanoex_switch_ != nullptr) {
    this->nanoex_state_ = nanoex;
    this->nanoex_switch_->publish_state(this->nanoex_state_);
  }
}

void PanasonicAC::update_eco(bool eco) {
  if (this->eco_switch_ != nullptr) {
    this->eco_state_ = eco;
    this->eco_switch_->publish_state(this->eco_state_);
  }
}

void PanasonicAC::update_econavi(bool econavi) {
  if (this->econavi_switch_ != nullptr) {
    this->econavi_state_ = econavi;
    this->econavi_switch_->publish_state(this->econavi_state_);
  }
}

void PanasonicAC::update_mild_dry(bool mild_dry) {
  if (this->mild_dry_switch_ != nullptr) {
    this->mild_dry_state_ = mild_dry;
    this->mild_dry_switch_->publish_state(this->mild_dry_state_);
  }
}

climate::ClimateAction PanasonicAC::determine_action() {
  if (this->mode == climate::CLIMATE_MODE_OFF) {
    return climate::CLIMATE_ACTION_OFF;
  } else if (this->mode == climate::CLIMATE_MODE_FAN_ONLY) {
    return climate::CLIMATE_ACTION_FAN;
  } else if (this->mode == climate::CLIMATE_MODE_DRY) {
    return climate::CLIMATE_ACTION_DRYING;
  }

  // In compensation mode compare raw intake against the wire value we actually
  // sent (like upstream did vs the biased wire); fall back to display values.
  float current = this->current_temperature;
  float target = this->target_temperature;
  if (this->compensation_active() && !std::isnan(this->intake_temperature_) &&
      !std::isnan(this->last_sent_setpoint_)) {
    current = this->intake_temperature_;
    target = this->last_sent_setpoint_;
  }

  if ((this->mode == climate::CLIMATE_MODE_COOL || this->mode == climate::CLIMATE_MODE_HEAT_COOL) &&
      current + TEMPERATURE_TOLERANCE >= target) {
    return climate::CLIMATE_ACTION_COOLING;
  } else if ((this->mode == climate::CLIMATE_MODE_HEAT || this->mode == climate::CLIMATE_MODE_HEAT_COOL) &&
             current - TEMPERATURE_TOLERANCE <= target) {
    return climate::CLIMATE_ACTION_HEATING;
  } else {
    return climate::CLIMATE_ACTION_IDLE;
  }
}

void PanasonicAC::update_current_power_consumption(int16_t power) {
  if (this->current_power_consumption_sensor_ != nullptr && this->current_power_consumption_sensor_->state != power) {
    this->current_power_consumption_sensor_->publish_state(
        power);  // Set current power consumption
  }
}

void PanasonicAC::update_internal_boost(float boost) {
  ESP_LOGV(TAG, "Received internal boost %.1f", boost);
  if (this->internal_boost_sensor_ != nullptr && this->internal_boost_sensor_->state != boost) {
    this->internal_boost_sensor_->publish_state(boost);  // Publish on change only
  }
}

void PanasonicAC::update_defrost(bool defrost) {
  if (this->defrost_sensor_ != nullptr) {
    this->defrost_sensor_->publish_state(defrost);
  }
}

/*
 * External-sensor setpoint compensation
 */

float PanasonicAC::compute_wire_setpoint() const {
  if (std::isnan(this->desired_setpoint_))
    return NAN;

  bool law_valid = !std::isnan(this->room_temperature_) && !std::isnan(this->intake_temperature_);
  float wire;
  if (!law_valid) {
    // Fallback = plain-mode / upstream logic. Deliberately NO boost
    // subtraction - the default path must byte-match what a
    // non-compensating setup sends.
    wire = std::floor((this->desired_setpoint_ - this->current_temperature_offset_) * 2.0f) / 2.0f;
  } else {
    // Intended effective internal setpoint (what byte 13 should end up as):
    // Full proportional on the raw error - 2026-10-20: the earlier
    // (desired - room) / 2 half-step gave the unit too much deadband and
    // produced over/undershoot; drive it aggressively, the unit's own
    // hysteresis still damps the response.
    wire = this->intake_temperature_ + (this->desired_setpoint_ - this->room_temperature_);
    // Never let the unit thermostat-off: it idles when intake > b13 + 1, so
    // the floor applies to the INTENDED value - boost comes off after.
    wire = std::min(30.0f, std::max(wire, std::max(this->intake_temperature_ - 1.0f, 16.0f)));
    wire = std::floor(wire * 2.0f) / 2.0f;

    // Feedforward-cancel the unit's internal boost: the compressor tracks
    // b13 = b3 + B, so the unit settles intake B degrees above whatever we
    // put in b3.
    // RESOLVED 2026-10-20 (live, kinderzimmer): the unit's REALIZED boost
    // deviates from the byte-derived B - pure B cancellation made the room
    // undershoot. Cancel HALF of B instead: a linear scale rather than a
    // fixed offset (a constant BOOST_BASELINE=0.5 correction was fitted at
    // B=1.0; B/2 gives the identical result there while generalizing to
    // other B). The internal_boost member and its sensor keep the true B.
    if (!std::isnan(this->internal_boost_)) {
      wire -= this->internal_boost_ / 2.0f;
      wire = std::floor(wire * 2.0f) / 2.0f;  // B/2 can land off-grid (0.25 steps); floor re-grids for exact write-skip/echo
    }
  }
  wire = std::min(30.0f, std::max(wire, 16.0f));

  // Log every calculation with its inputs (desired in display units, room
  // raw sensor, intake raw packet, boost (b13-b3)/2, wire = actual byte).
  if (!law_valid) {
    ESP_LOGD(TAG, "Setpoint calc (fallback, sensors stale): desired=%.1f offset=%d -> wire=%.1f",
             this->desired_setpoint_, this->current_temperature_offset_, wire);
  } else if (std::isnan(this->internal_boost_)) {
    ESP_LOGD(TAG, "Setpoint calc: desired=%.1f room=%.1f intake=%.1f boost=? -> wire=%.1f (no cancellation yet)",
             this->desired_setpoint_, this->room_temperature_, this->intake_temperature_, wire);
  } else {
    ESP_LOGD(TAG, "Setpoint calc: desired=%.1f room=%.1f intake=%.1f boost=%.1f (cancel %.1f = B/2) -> wire=%.1f",
             this->desired_setpoint_, this->room_temperature_, this->intake_temperature_,
             this->internal_boost_, this->internal_boost_ / 2.0f, wire);
  }
  return wire;
}

void PanasonicAC::maybe_restore_desired_setpoint() {
  if (!this->compensation_active())
    return;

  // Unique per entity so multiple units on one ESP don't collide
  uint32_t hash = this->get_object_id_hash() ^ fnv1_hash("desired_setpoint");
  this->desired_setpoint_pref_ = global_preferences->make_preference<float>(hash);

  float restored;
  if (this->desired_setpoint_pref_.load(&restored) && !std::isnan(restored)) {
    this->desired_setpoint_ = restored;
    ESP_LOGI(TAG, "Restored desired setpoint %.1f", restored);
  }
}

void PanasonicAC::save_desired_setpoint(float desired) {
  this->desired_setpoint_pref_.save(&desired);  // no-op if preference never bound
}

/*
 * Sensor handling
 */

void PanasonicAC::set_outside_temperature_sensor(sensor::Sensor *outside_temperature_sensor) {
  this->outside_temperature_sensor_ = outside_temperature_sensor;
}

void PanasonicAC::set_outside_temperature_offset(int8_t outside_temperature_offset) {
  ESP_LOGV(TAG, "Outside temperature offset %d", outside_temperature_offset);
  this->outside_temperature_offset_ = outside_temperature_offset;

  if (this->outside_temperature_sensor_) {
    ESP_LOGV(TAG, "Corrected outside temperature: %d", this->outside_temperature_sensor_->state + outside_temperature_offset);
  }
}

void PanasonicAC::set_current_temperature_offset(int8_t current_temperature_offset)
{
  ESP_LOGV(TAG, "Current temperature offset %d", current_temperature_offset);
  this->current_temperature_offset_ = current_temperature_offset;

  if (this->current_temperature_sensor_) {
    ESP_LOGV(TAG, "Corrected current temperature: %d", this->current_temperature_sensor_->state + current_temperature_offset);
  }
}

void PanasonicAC::set_current_temperature_sensor(sensor::Sensor *current_temperature_sensor)
{
  this->current_temperature_sensor_ = current_temperature_sensor;
  this->current_temperature_sensor_->add_on_state_callback([this](float state)
                                                           {
                                                             this->room_temperature_ = state;  // RAW for the law
                                                             this->current_temperature = state + this->current_temperature_offset_;
                                                             this->publish_state();
                                                             this->on_room_change();
                                                           });
}

void PanasonicAC::set_vertical_swing_select(select::Select *vertical_swing_select) {
  this->vertical_swing_select_ = vertical_swing_select;
  this->vertical_swing_select_->add_on_state_callback([this](size_t index) {
    if (index == this->vertical_swing_state_)
      return;
    this->on_vertical_swing_change(this->vertical_swing_select_->current_option());
  });
}

void PanasonicAC::set_horizontal_swing_select(select::Select *horizontal_swing_select) {
  this->horizontal_swing_select_ = horizontal_swing_select;
  this->horizontal_swing_select_->add_on_state_callback([this](size_t index) {
    if (index == this->horizontal_swing_state_)
      return;
    this->on_horizontal_swing_change(this->horizontal_swing_select_->current_option());
  });
}

void PanasonicAC::set_nanoex_switch(switch_::Switch *nanoex_switch) {
  this->nanoex_switch_ = nanoex_switch;
  this->nanoex_switch_->add_on_state_callback([this](bool state) {
    if (state == this->nanoex_state_)
      return;
    this->on_nanoex_change(state);
  });
}

void PanasonicAC::set_eco_switch(switch_::Switch *eco_switch) {
  this->eco_switch_ = eco_switch;
  this->eco_switch_->add_on_state_callback([this](bool state) {
    if (state == this->eco_state_)
      return;
    this->on_eco_change(state);
  });
}

void PanasonicAC::set_econavi_switch(switch_::Switch *econavi_switch) {
  this->econavi_switch_ = econavi_switch;
  this->econavi_switch_->add_on_state_callback([this](bool state) {
    if (state == this->econavi_state_)
      return;
    this->on_econavi_change(state);
  });
}

void PanasonicAC::set_mild_dry_switch(switch_::Switch *mild_dry_switch) {
  this->mild_dry_switch_ = mild_dry_switch;
  this->mild_dry_switch_->add_on_state_callback([this](bool state) {
    if (state == this->mild_dry_state_)
      return;
    this->on_mild_dry_change(state);
  });
}

void PanasonicAC::set_current_power_consumption_sensor(sensor::Sensor *current_power_consumption_sensor) {
  this->current_power_consumption_sensor_ = current_power_consumption_sensor;
}

void PanasonicAC::set_internal_boost_sensor(sensor::Sensor *internal_boost_sensor) {
  this->internal_boost_sensor_ = internal_boost_sensor;
}

void PanasonicAC::set_defrost_sensor(binary_sensor::BinarySensor *defrost_sensor) {
  this->defrost_sensor_ = defrost_sensor;
}

/*
 * Debugging
 */

void PanasonicAC::log_packet(std::vector<uint8_t> data, bool outgoing) {
  if (outgoing) {
    ESP_LOGV(TAG, "TX: %s", format_hex_pretty(data).c_str());
  } else {
    ESP_LOGV(TAG, "RX: %s", format_hex_pretty(data).c_str());
  }
}

}  // namespace panasonic_ac
}  // namespace esphome
