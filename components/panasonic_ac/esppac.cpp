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

void PanasonicAC::note_unit_idle() {
  if (std::isnan(this->desired_setpoint_))
    return;
  bool room_valid = !std::isnan(this->room_temperature_);
  if (room_valid && !this->overshoot_ && this->room_temperature_ > this->desired_setpoint_) {
    this->overshoot_ = true;
    ESP_LOGI(TAG, "Overshoot latch ON: unit idle with room %.1f over desired %.1f - holding off",
             this->room_temperature_, this->desired_setpoint_);
  }
}

void PanasonicAC::note_user_change() {
  if (this->overshoot_) {
    this->overshoot_ = false;
    ESP_LOGI(TAG, "Overshoot latch cleared: user setpoint/mode change");
  }
}

float PanasonicAC::compute_wire_setpoint() {
  if (std::isnan(this->desired_setpoint_))
    return NAN;

  // Effective desired: the overshoot latch hard-holds the unit OFF by pinning
  // the wire at the unit's minimum (16) - the law is bypassed so no clamp can
  // lift the wire back up (a mere desired-sink saturates at E_MIN and still
  // allows D=+2.5 cruise margins). Cleared when the room gives back 0.5 K or
  // the user intervenes. Passthrough experiment builds are exempt.
  float desired = this->desired_setpoint_;
  if (this->overshoot_ && COMPENSATION_ENABLED) {
    if (!std::isnan(this->room_temperature_) &&
        this->room_temperature_ <= desired - 0.5f) {
      this->overshoot_ = false;
      ESP_LOGI(TAG, "Overshoot latch cleared: room dipped to %.1f (desired %.1f)",
               this->room_temperature_, desired);
    } else {
      // Room sensor stale -> latch stays (conservative: hold off).
      float wire = 16.0f;  // unit minimum; b13 = 16 + B stays far below intake
      ESP_LOGD(TAG, "Setpoint calc (overshoot hold): desired=%.1f room=%.1f intake=%.1f -> wire=%.1f "
                    "(state=0x%02X D=%.1f pw=%.0fW)",
               desired, this->room_temperature_, this->intake_temperature_,
               wire, this->last_state_byte_, this->last_reported_gap_, this->last_power_w_);
      return wire;
    }
  }

  if (!COMPENSATION_ENABLED) {
    // Law disabled in code: upstream-plain byte - the climate target goes to
    // the unit unchanged. Telemetry keeps flowing so external tooling can
    // observe state/D/pw while steering the wire via the climate target.
    float wire = std::floor((desired - this->current_temperature_offset_) * 2.0f) / 2.0f;
    wire = std::min(30.0f, std::max(wire, 16.0f));
    ESP_LOGD(TAG, "Setpoint calc (passthrough): desired=%.1f room=%.1f intake=%.1f boost=%.1f -> wire=%.1f "
                  "(state=0x%02X D=%.1f pw=%.0fW)",
             this->desired_setpoint_, this->room_temperature_, this->intake_temperature_, this->internal_boost_,
             wire, this->last_state_byte_, this->last_reported_gap_, this->last_power_w_);
    return wire;
  }

  bool law_valid = !std::isnan(this->room_temperature_) && !std::isnan(this->intake_temperature_);
  bool law_heat = law_valid && this->mode == climate::CLIMATE_MODE_HEAT && !std::isnan(this->internal_boost_);
  float wire;
  float gap_ss = NAN, e_star = NAN;  // v7 intermediates (logged)
  if (!law_valid) {
    // Fallback = plain-mode / upstream logic. Deliberately NO boost
    // subtraction - the default path must byte-match what a
    // non-compensating setup sends.
    wire = std::floor((desired - this->current_temperature_offset_) * 2.0f) / 2.0f;
  } else if (law_heat) {
    // v7 cruise law (HEAT only until cooling has field data).
    // E* = clamp(desired - room + gap_ss - LEAD, -2.5, +2.5); b13 = intake + E*;
    // wire cancels the FULL boost so realized b13 tracks E* through B transients.
    // gap_ss = 0 (GAP_WARM): realized D = -E* = room - desired + LEAD, so the
    // running unit cruises at D = +0.5 while room approaches desired - just
    // under the measured stop threshold (~+0.5..+1.0) - and crosses 0 /
    // restart once room falls ~0.5 K below the target. Starts too deep
    // (D > -2.5) are too rare to matter and self-correct via the clamps.
    // Nearest-grid quantization (v7.1): floor() ate up to 0.4 K of commanded
    // heat (field: E*=+0.40 -> realized D=0.0, room stalled 0.9 K under
    // target). Nearest caps the law's D error at +/-0.25 K (bias-free); the
    // plain/fallback paths below keep floor() to stay byte-compatible with
    // upstream. The overshoot latch sinks `desired` upstream of E*.
    float boost = std::min(HEAT_BOOST_SCALE, std::max(0.0f, this->internal_boost_));
    gap_ss = HEAT_GAP_WARM * (1.0f - boost / HEAT_BOOST_SCALE);
    e_star = desired - this->room_temperature_ + gap_ss - HEAT_LEAD;
    e_star = std::min(HEAT_E_MAX, std::max(HEAT_E_MIN, e_star));
    wire = std::roundf((this->intake_temperature_ + e_star - this->internal_boost_) * 2.0f) / 2.0f;
  } else {
    // Legacy law (cool / heat-cool / boost not yet known): full proportional
    // on the raw error - 2026-10-20: the earlier (desired - room)/2 half-step
    // left too much deadband and produced over/undershoot.
    wire = this->intake_temperature_ + (desired - this->room_temperature_);
    // Never let the unit thermostat-off: the service manual turns the
    // compressor off when intake - b13 > +2.0 K (3-min restart below), so
    // b13 >= intake - 1 holds D <= +1, a full K below OFF. Floor applies to
    // the INTENDED value - the boost subtraction below shifts realized b13
    // back up, making D even smaller.
    wire = std::min(30.0f, std::max(wire, std::max(this->intake_temperature_ - 1.0f, 16.0f)));
    wire = std::floor(wire * 2.0f) / 2.0f;

    // Feedforward-cancel half the unit's internal boost: the compressor tracks
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

  // Log every calculation with its inputs (desired = effective staged value,
  // i.e. pinned to the 16 hold while the overshoot latch is active; room raw sensor; intake
  // raw packet; boost (b13-b3)/2; wire = actual byte) plus last-poll
  // diagnostics: operating state byte, realized thermostat gap D = intake -
  // b13, instantaneous power.
  if (!law_valid) {
    ESP_LOGD(TAG, "Setpoint calc (fallback, sensors stale): desired=%.1f offset=%d -> wire=%.1f "
                  "(state=0x%02X D=%.1f pw=%.0fW)",
             desired, this->current_temperature_offset_, wire, this->last_state_byte_,
             this->last_reported_gap_, this->last_power_w_);
  } else if (law_heat) {
    ESP_LOGD(TAG, "Setpoint calc: desired=%.1f room=%.1f intake=%.1f boost=%.1f -> wire=%.1f "
                  "(gap_ss=%.2f E*=%.2f state=0x%02X D=%.1f pw=%.0fW)",
             desired, this->room_temperature_, this->intake_temperature_, this->internal_boost_,
             wire, gap_ss, e_star, this->last_state_byte_, this->last_reported_gap_, this->last_power_w_);
  } else if (std::isnan(this->internal_boost_)) {
    ESP_LOGD(TAG, "Setpoint calc: desired=%.1f room=%.1f intake=%.1f boost=? -> wire=%.1f (no cancellation yet) "
                  "(state=0x%02X D=%.1f pw=%.0fW)",
             desired, this->room_temperature_, this->intake_temperature_, wire, this->last_state_byte_,
             this->last_reported_gap_, this->last_power_w_);
  } else {
    ESP_LOGD(TAG, "Setpoint calc: desired=%.1f room=%.1f intake=%.1f boost=%.1f (cancel %.1f = B/2) -> wire=%.1f "
                  "(state=0x%02X D=%.1f pw=%.0fW)",
             desired, this->room_temperature_, this->intake_temperature_, this->internal_boost_,
             this->internal_boost_ / 2.0f, wire, this->last_state_byte_, this->last_reported_gap_,
             this->last_power_w_);
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
