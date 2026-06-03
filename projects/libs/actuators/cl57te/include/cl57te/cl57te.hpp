#pragma once

#include "hardware/gpio.h"
#include "pico/time.h"
#include "pico/types.h"
#include <stdint.h>

// Pass INVALID_PIN for ena / alm in Cl57te::Config if those signals are unwired.
static constexpr uint INVALID_PIN = 0xFFu;

// -----------------------------------------------------------------------------
// Cl57te  –  CL57TE closed-loop stepper drive hardware abstraction
//
// Supports both single-ended and full differential wiring.
//
// Single-ended (pul_n / dir_n = INVALID_PIN):
//   PUL+  ← Pico GPIO (step pulses)         PUL-/COM- → GND
//   DIR+  ← Pico GPIO (direction)            DIR-      → GND
//
// Differential (pul_n / dir_n assigned):
//   PUL+  ← Pico GPIO   PUL-  ← Pico GPIO (complement)
//   DIR+  ← Pico GPIO   DIR-  ← Pico GPIO (complement)
//   Idle: PUL+=LOW/PUL-=HIGH, DIR± set to current direction
//   Step: PUL+=HIGH/PUL-=LOW for 2 µs, then back to idle
//
//   ENA+  ← Pico GPIO (HIGH = disabled)      ENA-      → GND
//   ALM   → Pico GPIO (open-collector, pulled-up; LOW = fault)
//
// The drive clocks a step on every rising edge of (PUL+ − PUL-).
// Step resolution (pulses/rev) is set by the drive's DIP switches.
//
// Timing requirements:
//   • PUL high/low ≥ 1 µs   – driver generates 2 µs pulses
//   • DIR setup ≥ 5 µs before first PUL after a direction change
//   • After ENA goes inactive (motor enabled): ≥ 200 ms before first pulse
//
// Thread safety:
//   isr_tick()  is called from the hardware-timer ISR.
//   All other methods must be called only from the owning FreeRTOS task.
// -----------------------------------------------------------------------------

class Cl57te {
public:
    // -- Hardware configuration ------------------------------------------------
    struct Config {
        uint    pul;            // step pulse GPIO (PUL+ normally; PUL- if pul_active_low)
        uint    pul_n;          // complement GPIO (INVALID_PIN = single-ended or hardwired)
        uint    dir;            // direction GPIO  (DIR+ normally; DIR- if dir_active_low)
        uint    dir_n;          // complement GPIO (INVALID_PIN = single-ended or hardwired)
        uint    ena;            // ENA+ GPIO – HIGH disables the drive
        uint    alm;            // ALM  GPIO – input, LOW = fault (open-collector)

        uint32_t pulses_per_rev;   // pulses/rev as programmed in drive DIP switches
        float    gear_ratio;       // reduction ratio = motor_revs / output_shaft_revs
                                   // (e.g. 10.0 for a 10:1 planetary gearbox, 1.0 = direct drive)

        float    max_speed_dps;    // hard ceiling on degrees/sec
        float    default_speed_dps;

        // Set true when the + rail is hardwired to 5V and `pul`/`dir` are the
        // active-low complement pins. Idle = HIGH, pulse/forward = LOW.
        bool    pul_active_low = false;
        bool    dir_active_low = false;
    };

    // -- Lifecycle -------------------------------------------------------------
    void init( const Config& cfg );

    // Enable: drives ENA LOW (motor powered, holding).
    // The caller must vTaskDelay(200 ms) before calling start_move() —
    // the CL57TE needs time to initialise after power-on of the coils.
    void enable();

    // Disable: drives ENA HIGH (motor de-energised, freely rotating).
    void disable();

    bool is_enabled() const  { return enabled_; }

    // ALM pin: LOW = fault (overcurrent, over-voltage, position following error).
    // Returns false (no fault assumed) when alm pin is INVALID_PIN.
    bool is_faulted() const  { return cfg_.alm != INVALID_PIN && !gpio_get( cfg_.alm ); }

    // -- Motion ----------------------------------------------------------------
    // start_move: begin stepping toward |n_steps| from current position.
    //   n_steps > 0  -> positive direction (DIR HIGH)
    //   n_steps < 0  -> negative direction (DIR LOW)
    //   step_hz      -> step pulse rate; 0 = use default from Config
    //
    // Starts a repeating hardware timer that generates step pulses via isr_tick().
    // Returns false if faulted or not enabled.
    bool start_move( int32_t n_steps, uint32_t step_hz = 0 );

    // Abort the current move (cancels the timer). Position remains at the last
    // emitted step.
    void stop();

    bool is_moving() const  { return remaining_ > 0; }

    // -- ISR entry point -------------------------------------------------------
    // Returns true to keep the repeating timer alive; false to cancel it.
    // Called from hardware-timer IRQ — keep it short, no FreeRTOS calls.
    bool isr_tick();

    // -- Position --------------------------------------------------------------
    // pos_steps_ is advanced by the timer ISR for each emitted step. It is the
    // commanded mechanical position, not an encoder measurement.
    int32_t  pos_steps() const  { return pos_steps_; }
    float    angle_deg()  const;

    // Used by calibration/home procedures to define the current mechanical angle.
    void commit_position( int32_t pos ) { pos_steps_ = pos; }

    float steps_per_deg() const { return steps_per_deg_; }

    // -- Public timer handle (needed for add_repeating_timer_us user_data) ----
    repeating_timer_t timer;

private:
    Config   cfg_{};
    bool     enabled_  = false;
    volatile int32_t  pos_steps_ = 0;
    volatile int8_t   step_sign_ = 1;

    float    steps_per_deg_ = 1.0f;   // cached at init

    // Modified by ISR, read by task — volatile is sufficient here because
    // the task only reads (never writes while the timer is running).
    volatile int32_t  remaining_ = 0;
};
