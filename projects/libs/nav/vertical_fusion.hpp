#pragma once

#include <math.h>
#include <stdint.h>

namespace avionics::nav {

struct VerticalFusionConfig {
    float baro_alt_gain = 0.10f;
    float baro_vel_gain = 0.015f;
    float gps_alt_gain = 0.04f;
    float gps_vel_gain = 0.20f;
    float max_dt_s = 0.10f;
    float accel_limit_mss = 280.0f;
};

class VerticalFusion {
public:
    explicit VerticalFusion(VerticalFusionConfig cfg = {}) : cfg_(cfg) {}

    void reset(float altitude_m, float vel_down_mps, uint32_t now_ms)
    {
        altitude_m_ = altitude_m;
        vel_down_mps_ = vel_down_mps;
        last_ms_ = now_ms;
        initialized_ = true;
    }

    bool initialized() const { return initialized_; }
    float altitude_m() const { return altitude_m_; }
    float vel_down_mps() const { return vel_down_mps_; }

    float update(uint32_t now_ms,
                 float baro_alt_m,
                 bool have_gps,
                 float gps_alt_m,
                 float gps_vel_down_mps,
                 bool have_accel,
                 float accel_down_mss)
    {
        if (!initialized_) {
            reset(have_gps ? gps_alt_m : baro_alt_m,
                  have_gps ? gps_vel_down_mps : 0.0f,
                  now_ms);
            return 0.0f;
        }

        float dt_s = static_cast<float>(now_ms - last_ms_) * 0.001f;
        if (!isfinite(dt_s) || dt_s <= 0.0f) {
            dt_s = 0.0f;
        } else if (dt_s > cfg_.max_dt_s) {
            dt_s = cfg_.max_dt_s;
        }
        last_ms_ = now_ms;

        float accel = 0.0f;
        if (have_accel && isfinite(accel_down_mss)) {
            accel = clamp(accel_down_mss, -cfg_.accel_limit_mss, cfg_.accel_limit_mss);
        }

        altitude_m_ += (-vel_down_mps_ * dt_s) - (0.5f * accel * dt_s * dt_s);
        vel_down_mps_ += accel * dt_s;

        correct_altitude(baro_alt_m, cfg_.baro_alt_gain, cfg_.baro_vel_gain, dt_s);

        if (have_gps) {
            correct_altitude(gps_alt_m, cfg_.gps_alt_gain, cfg_.baro_vel_gain, dt_s);
            if (isfinite(gps_vel_down_mps)) {
                vel_down_mps_ += cfg_.gps_vel_gain * (gps_vel_down_mps - vel_down_mps_);
            }
        }

        return dt_s;
    }

private:
    static float clamp(float value, float min_value, float max_value)
    {
        if (value < min_value) return min_value;
        if (value > max_value) return max_value;
        return value;
    }

    void correct_altitude(float measured_alt_m, float alt_gain, float vel_gain, float dt_s)
    {
        if (!isfinite(measured_alt_m)) return;
        const float residual = measured_alt_m - altitude_m_;
        altitude_m_ += alt_gain * residual;
        const float vel_dt = (dt_s > 0.005f) ? dt_s : 0.02f;
        vel_down_mps_ -= vel_gain * residual / vel_dt;
    }

    VerticalFusionConfig cfg_;
    bool initialized_ = false;
    uint32_t last_ms_ = 0;
    float altitude_m_ = 0.0f;
    float vel_down_mps_ = 0.0f;
};

} // namespace avionics::nav
