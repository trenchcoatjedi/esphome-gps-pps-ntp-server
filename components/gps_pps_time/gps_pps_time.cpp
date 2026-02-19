#include "gps_pps_time.h"
#include "esphome/core/log.h"
#include <sys/time.h>
#include <cstdlib>

namespace esphome {
namespace gps_pps_time {

static const char *const TAG = "gps_pps_time";

void IRAM_ATTR GPSPPSTime::pps_isr(GPSPPSTime *self) {
  self->last_pps_micros_ = micros();
  self->pps_flag_ = true;
}

void GPSPPSTime::setup() {
  ESP_LOGI(TAG, "Setting up GPS PPS time source...");
  this->pps_pin_->setup();
  this->pps_pin_->attach_interrupt(&GPSPPSTime::pps_isr, this, gpio::INTERRUPT_RISING_EDGE);
}

void GPSPPSTime::loop() {
  if (this->pps_flag_) {
    this->pps_flag_ = false;
    this->apply_pps_correction_();
  }
}

void GPSPPSTime::set_pps_time_(time_t epoch, int32_t compensation_us) {
  // Account for time elapsed since PPS edge, minus drift pre-compensation
  int32_t elapsed_us = static_cast<int32_t>(micros() - this->last_pps_micros_);
  int32_t adjusted_us = elapsed_us - compensation_us;
  struct timeval tv;
  tv.tv_sec = epoch;
  tv.tv_usec = (adjusted_us > 0) ? adjusted_us : 0;
  struct timezone tz = {0, 0};
  settimeofday(&tv, &tz);
}

void GPSPPSTime::apply_pps_correction_() {
  if (!this->gps_time_valid_) {
    ESP_LOGD(TAG, "PPS pulse received but no valid GPS time yet");
    return;
  }

  // Guard: if pps_flag_ blocked NMEA from setting the epoch, last_gps_epoch_ is still 0.
  // Skip this PPS and wait for the next NMEA + PPS cycle.
  if (this->last_gps_epoch_ == 0) {
    ESP_LOGD(TAG, "PPS pulse skipped, waiting for NMEA epoch");
    return;
  }

  this->pps_count_++;
  this->last_pps_millis_ = millis();

  // Detect ISR latency from PPS interval deviation.
  // If UART ISR delays the PPS ISR, micros() captures a late timestamp.
  // The interval between consecutive PPS ISRs reveals this delay.
  int32_t isr_latency_us = 0;
  if (this->pps_count_ > 1) {
    uint32_t pps_interval = this->last_pps_micros_ - this->prev_pps_micros_;
    int32_t deviation = static_cast<int32_t>(pps_interval) - 1000000;
    // Only compensate for plausible ISR delays (500µs–10ms)
    if (deviation > 500 && deviation < 10000) {
      isr_latency_us = deviation;
      ESP_LOGD(TAG, "PPS ISR latency detected: %d us", isr_latency_us);
    }
  }
  this->prev_pps_micros_ = this->last_pps_micros_;

  // PPS marks the start of the next second after the last GPS epoch
  time_t corrected_epoch = this->last_gps_epoch_ + 1;
  // Advance epoch so the next PPS pulse computes the correct next second
  this->last_gps_epoch_ = corrected_epoch;

  // Measure drift: compensate for ISR-to-loop delay AND ISR latency
  uint32_t now_us = micros();
  uint32_t isr_delay_us = now_us - this->last_pps_micros_;

  struct timeval current_tv;
  gettimeofday(&current_tv, nullptr);
  // System time at the actual PPS edge = current time minus all delays
  int64_t system_us_at_pps = static_cast<int64_t>(current_tv.tv_sec) * 1000000LL
                             + current_tv.tv_usec - isr_delay_us - isr_latency_us;
  // Expected time at PPS edge: corrected_epoch seconds, 0 microseconds
  int64_t expected_us = static_cast<int64_t>(corrected_epoch) * 1000000LL;
  this->last_drift_us_ = system_us_at_pps - expected_us;

  if (!this->pps_synced_) {
    // First PPS: hard-sync the clock, accounting for elapsed time since PPS edge
    this->set_pps_time_(corrected_epoch);
    this->last_drift_us_ = 0;

    ESP_LOGI(TAG, "PPS-disciplined time synchronized");
    this->pps_synced_ = true;
    this->time_sync_callback_.call();
  } else if (this->last_drift_us_ > 500000 || this->last_drift_us_ < -500000) {
    // PPS epoch counter has diverged (e.g. PPS gap, GPS reconfiguration).
    // Reset sync state so NMEA re-establishes the correct epoch before next PPS.
    ESP_LOGW(TAG, "Large drift detected (%lld us), re-syncing from NMEA+PPS",
             (long long) this->last_drift_us_);
    this->pps_synced_ = false;
    this->gps_time_valid_ = false;
    this->has_gps_time_ = false;
    this->last_gps_epoch_ = 0;
    this->last_drift_us_ = 0;
    this->drift_mean_x256_ = 0;
  } else {
    // Normal operation: correct every PPS pulse
#ifdef USE_ESP_IDF
    // adjtime() gradually slews the clock without jumping — ideal for NTP serving
    // Filter measurement spikes (e.g. from sensor publishing stalling the loop)
    // to prevent adjtime from applying wrong corrections that cause oscillation
    if (this->last_drift_us_ > -500 && this->last_drift_us_ < 500) {
      // Overcorrect by the running mean (≈ crystal drift rate / 2) to center the
      // sawtooth error around zero. Without this, the clock drifts from 0 to +D µs
      // between PPS pulses. With overcorrection, it swings from -D/2 to +D/2,
      // halving the peak error that NTP clients see.
      int64_t overcorrection = this->drift_mean_x256_ / 256;
      int64_t correction_us = -(this->last_drift_us_ + overcorrection);
      struct timeval delta;
      delta.tv_sec = correction_us / 1000000LL;
      delta.tv_usec = correction_us % 1000000LL;
      adjtime(&delta, nullptr);
      // Track running mean of measured drift (fixed-point x256, alpha 1/64)
      this->drift_mean_x256_ += (this->last_drift_us_ * 256 - this->drift_mean_x256_) / 64;
      this->last_clock_offset_us_ = this->last_drift_us_;
    } else {
      ESP_LOGD(TAG, "Drift spike filtered from adjtime: %lld us",
               (long long) this->last_drift_us_);
      // Spike filtered — clock free-runs on previous correction, estimate extra drift
      this->last_clock_offset_us_ += this->drift_mean_x256_ / 256;
    }
#else
    // Platforms without adjtime(): always correct, protect EMA from spikes
    // Subtract ISR latency from compensation so set_pps_time_ adds it to elapsed time
    this->set_pps_time_(corrected_epoch, this->drift_compensation_us_ - isr_latency_us);
    // Only update drift estimate when measurement is clean (no ISR latency, small drift)
    if (isr_latency_us == 0 && this->last_drift_us_ > -50 && this->last_drift_us_ < 50) {
      this->drift_compensation_us_ += static_cast<int32_t>(this->last_drift_us_) / 4;
    } else {
      ESP_LOGD(TAG, "PPS drift spike filtered from EMA: %lld us",
               (long long) this->last_drift_us_);
    }
#endif
  }

  ESP_LOGD(TAG, "PPS #%lu, epoch: %ld, adj: %lld us",
           (unsigned long) this->pps_count_, (long) corrected_epoch,
           (long long) this->last_drift_us_);
}

void GPSPPSTime::on_update(TinyGPSPlus &tiny_gps) {
  // Register TinyGPSCustom extractors on first call (needs TinyGPSPlus reference)
  if (this->gp_gsv_sats_ == nullptr) {
    this->gp_gsv_sats_ = new TinyGPSCustom(tiny_gps, "GPGSV", 3);
    this->gl_gsv_sats_ = new TinyGPSCustom(tiny_gps, "GLGSV", 3);
    this->ga_gsv_sats_ = new TinyGPSCustom(tiny_gps, "GAGSV", 3);
  }

  if (tiny_gps.satellites.isValid()) {
    this->last_satellite_count_ = tiny_gps.satellites.value();
  }

  // Update per-constellation satellite counts from GSV sentences
  if (this->gp_gsv_sats_->isUpdated())
    this->last_gps_sat_count_ = atoi(this->gp_gsv_sats_->value());
  if (this->gl_gsv_sats_->isUpdated())
    this->last_glonass_sat_count_ = atoi(this->gl_gsv_sats_->value());
  if (this->ga_gsv_sats_->isUpdated())
    this->last_galileo_sat_count_ = atoi(this->ga_gsv_sats_->value());

  if (!tiny_gps.time.isValid() || !tiny_gps.date.isValid() ||
      !tiny_gps.time.isUpdated() || !tiny_gps.date.isUpdated() ||
      tiny_gps.date.year() < 2025) {
    return;
  }

  ESPTime val{};
  val.year = tiny_gps.date.year();
  val.month = tiny_gps.date.month();
  val.day_of_month = tiny_gps.date.day();
  val.day_of_week = 1;
  val.day_of_year = 1;
  val.hour = tiny_gps.time.hour();
  val.minute = tiny_gps.time.minute();
  val.second = tiny_gps.time.second();
  val.recalc_timestamp_utc(false);

  this->gps_time_valid_ = true;

  // Once PPS is synced, it manages the epoch counter via incrementing.
  // NMEA must not overwrite it, as stale timestamps would reset the counter backward.
  // Skip update when a PPS ISR is pending — prevents race where NMEA for the next
  // second arrives between the PPS ISR and loop() processing (would cause +1s error).
  if (!this->pps_synced_ && !this->pps_flag_) {
    this->last_gps_epoch_ = val.timestamp;
  }

  // Set coarse GPS time once as fallback until PPS takes over
  if (!this->pps_synced_ && !this->has_gps_time_) {
    this->synchronize_epoch_(val.timestamp);
    this->has_gps_time_ = true;
    ESP_LOGI(TAG, "GPS time set (coarse, waiting for PPS): %04d-%02d-%02d %02d:%02d:%02d",
             val.year, val.month, val.day_of_month, val.hour, val.minute, val.second);
  }
}

void GPSPPSTime::update() {
  if (this->satellites_sensor_ != nullptr) {
    // Sum per-constellation GSV counts for consistent "in view" total
    uint16_t total = this->last_gps_sat_count_ + this->last_glonass_sat_count_
                     + this->last_galileo_sat_count_;
    this->satellites_sensor_->publish_state(total);
  }

  if (this->gps_satellites_sensor_ != nullptr && this->gp_gsv_sats_ != nullptr && this->gp_gsv_sats_->isValid()) {
    this->gps_satellites_sensor_->publish_state(this->last_gps_sat_count_);
  }
  if (this->glonass_satellites_sensor_ != nullptr && this->gl_gsv_sats_ != nullptr && this->gl_gsv_sats_->isValid()) {
    this->glonass_satellites_sensor_->publish_state(this->last_glonass_sat_count_);
  }
  if (this->galileo_satellites_sensor_ != nullptr && this->ga_gsv_sats_ != nullptr && this->ga_gsv_sats_->isValid()) {
    this->galileo_satellites_sensor_->publish_state(this->last_galileo_sat_count_);
  }

  if (this->clock_offset_sensor_ != nullptr && this->pps_synced_) {
    // Filtered clock offset: actual error NTP clients see. Only updated on clean measurements;
    // on spikes, estimates drift from crystal rate (since adjtime skipped the correction).
    this->clock_offset_sensor_->publish_state(static_cast<float>(this->last_clock_offset_us_));
  }
  if (this->pps_drift_sensor_ != nullptr && this->pps_synced_) {
    // Raw PPS drift: unfiltered measurement at each PPS edge.
    // Shows ISR contention and measurement anomalies for diagnostics.
    this->pps_drift_sensor_->publish_state(static_cast<float>(this->last_drift_us_));
  }

  if (this->pps_synced_) {
    auto time = this->now();
    ESP_LOGD(TAG, "PPS-disciplined time: %04d-%02d-%02d %02d:%02d:%02d",
             time.year, time.month, time.day_of_month, time.hour, time.minute, time.second);

    if (this->gps_time_sensor_ != nullptr) {
      char buf[20];
      snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
               time.year, time.month, time.day_of_month, time.hour, time.minute, time.second);
      this->gps_time_sensor_->publish_state(buf);
    }
  }
}

bool GPSPPSTime::is_synchronized() const {
  if (!this->pps_synced_)
    return false;
  return (millis() - this->last_pps_millis_) < PPS_TIMEOUT_MS;
}

void GPSPPSTime::dump_config() {
  ESP_LOGCONFIG(TAG, "GPS PPS Time:");
  LOG_PIN("  PPS Pin: ", this->pps_pin_);
  ESP_LOGCONFIG(TAG, "  PPS Synced: %s", YESNO(this->pps_synced_));
}

}  // namespace gps_pps_time
}  // namespace esphome
