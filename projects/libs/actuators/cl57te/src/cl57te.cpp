#include "cl57te/cl57te.hpp"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/time.h"

// ---------------------------------------------------------------------------
// Hardware timer ISR  –  runs in IRQ context, no FreeRTOS calls allowed
// ---------------------------------------------------------------------------

static bool step_isr_cb( repeating_timer_t* rt )
{
    auto* d = static_cast<Cl57te*>( rt->user_data );
    return d->isr_tick();
}

bool Cl57te::isr_tick()
{
    if ( remaining_ <= 0 ) return false;

    // Step pulse: assert active level for >= 2 µs, then return to idle.
    // Active level is HIGH for PUL+ GPIO, LOW when pul_active_low (PUL- GPIO).
    const bool pul_active = !cfg_.pul_active_low;
    gpio_put( cfg_.pul, pul_active );
    if ( cfg_.pul_n != INVALID_PIN ) gpio_put( cfg_.pul_n, !pul_active );
    busy_wait_us_32( 5 );   // spec minimum 2.5 µs; 5 µs gives 2× margin
    gpio_put( cfg_.pul, !pul_active );
    if ( cfg_.pul_n != INVALID_PIN ) gpio_put( cfg_.pul_n, pul_active );

    pos_steps_ = pos_steps_ + step_sign_;
    remaining_ = remaining_ - 1;
    return remaining_ > 0;      // false -> timer self-cancels
}

// ---------------------------------------------------------------------------
// Cl57te::init
// ---------------------------------------------------------------------------

void Cl57te::init( const Config& cfg )
{
    cfg_ = cfg;

    // PUL – output, idle at inactive level
    gpio_init( cfg_.pul );
    gpio_set_dir( cfg_.pul, GPIO_OUT );
    gpio_put( cfg_.pul, cfg_.pul_active_low );   // active_low->idle HIGH; normal->idle LOW

    // PUL complement – output, idle at opposite level (skipped if hardwired/unused)
    if ( cfg_.pul_n != INVALID_PIN ) {
        gpio_init( cfg_.pul_n );
        gpio_set_dir( cfg_.pul_n, GPIO_OUT );
        gpio_put( cfg_.pul_n, !cfg_.pul_active_low );
    }

    // DIR – output, idle = positive direction (forward)
    gpio_init( cfg_.dir );
    gpio_set_dir( cfg_.dir, GPIO_OUT );
    gpio_put( cfg_.dir, cfg_.dir_active_low ? true : false );

    // DIR complement – output (skipped if hardwired/unused)
    if ( cfg_.dir_n != INVALID_PIN ) {
        gpio_init( cfg_.dir_n );
        gpio_set_dir( cfg_.dir_n, GPIO_OUT );
        gpio_put( cfg_.dir_n, cfg_.dir_active_low ? false : true );
    }

    // ENA – output, HIGH disables the drive; start disabled (safe)
    if ( cfg_.ena != INVALID_PIN ) {
        gpio_init( cfg_.ena );
        gpio_set_dir( cfg_.ena, GPIO_OUT );
        gpio_put( cfg_.ena, true );     // HIGH = disabled
    }
    enabled_ = false;

    // ALM – input with pull-up; drive pulls LOW on fault
    if ( cfg_.alm != INVALID_PIN ) {
        gpio_init( cfg_.alm );
        gpio_set_dir( cfg_.alm, GPIO_IN );
        gpio_pull_up( cfg_.alm );
    }

    // Pre-compute steps/degree for angle conversions
    float output_steps_per_rev = static_cast<float>( cfg_.pulses_per_rev )
                                 * cfg_.gear_ratio;
    steps_per_deg_ = output_steps_per_rev / 360.0f;

    remaining_ = 0;
    pos_steps_ = 0;
}

// ---------------------------------------------------------------------------
// Cl57te::enable / disable
// ---------------------------------------------------------------------------

void Cl57te::enable()
{
    if ( cfg_.ena != INVALID_PIN )
        gpio_put( cfg_.ena, false );    // LOW = ENA inactive = drive ENABLED
    enabled_ = true;
    // Caller is responsible for waiting >= 200 ms (vTaskDelay) before the
    // first start_move() call.
}

void Cl57te::disable()
{
    stop();
    if ( cfg_.ena != INVALID_PIN )
        gpio_put( cfg_.ena, true );     // HIGH = ENA active = drive DISABLED
    enabled_ = false;
}

// ---------------------------------------------------------------------------
// Cl57te::start_move
// ---------------------------------------------------------------------------

bool Cl57te::start_move( int32_t n_steps, uint32_t step_hz )
{
    if ( !enabled_ || is_faulted() ) return false;
    if ( n_steps == 0 )              return true;

    // Abort any in-progress move
    if ( remaining_ > 0 ) stop();

    // Set direction first, then observe DIR setup time (>= 5 µs).
    // With active-low DIR (DIR- GPIO, DIR+ hardwired 5V): LOW=forward, HIGH=reverse.
    bool forward = ( n_steps > 0 );
    bool dir_level = cfg_.dir_active_low ? !forward : forward;
    gpio_put( cfg_.dir, dir_level );
    if ( cfg_.dir_n != INVALID_PIN ) gpio_put( cfg_.dir_n, !dir_level );
    busy_wait_us_32( 10 );  // 10 µs >= the 5 µs minimum

    // Choose step rate
    if ( step_hz == 0 ) {
        step_hz = static_cast<uint32_t>( cfg_.default_speed_dps * steps_per_deg_ );
    }
    if ( step_hz < 1 ) step_hz = 1;
    float max_hz = cfg_.max_speed_dps * steps_per_deg_;
    if ( step_hz > static_cast<uint32_t>( max_hz ) )
        step_hz = static_cast<uint32_t>( max_hz );

    int64_t period_us = -( static_cast<int64_t>( 1'000'000 ) / step_hz );

    step_sign_ = forward ? 1 : -1;
    remaining_ = ( n_steps > 0 ) ? n_steps : -n_steps;
    add_repeating_timer_us( period_us, step_isr_cb, this, &timer );
    return true;
}

// ---------------------------------------------------------------------------
// Cl57te::stop
// ---------------------------------------------------------------------------

void Cl57te::stop()
{
    if ( remaining_ > 0 ) {
        cancel_repeating_timer( &timer );
        remaining_ = 0;
    }
}

// ---------------------------------------------------------------------------
// Cl57te::angle_deg
// ---------------------------------------------------------------------------

float Cl57te::angle_deg() const
{
    return static_cast<float>( pos_steps_ ) / steps_per_deg_;
}
