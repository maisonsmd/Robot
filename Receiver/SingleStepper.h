#pragma once

#include "fast_io.h"
#include "Logger.h"

#ifndef STEPPER_PULSE_DURATION
#define STEPPER_PULSE_DURATION 5 // us
#endif

#define STEPPER_INVERT_PUL

enum class VProfile {
    CONSTANT,
    CHANGING
};

double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class single_stepper {
public:
    struct profile_segment {
        // 0: start, 1: end
        double t0, t1;
        double v0, v1;
        int32 p0, p1;
        void print() {
            DEBUGF("t: %4.1f %4.1f v: %4.0f %4.0f p: %5ld %5ld", t0, t1, v0, v1, p0, p1);
        }
    };
    profile_segment segment;
    VProfile profile{ VProfile::CONSTANT };

    double profile_seg_pos(double time) {
        auto delta_t = time - segment.t0;
        auto temp_accel = segment.v1 > segment.v0 ? accel : -accel;

        switch (profile)
        {
        case VProfile::CONSTANT:
            return segment.p0 + segment.v0 * delta_t;
        case VProfile::CHANGING:
            return segment.p0 + segment.v0 * delta_t + 0.5 * temp_accel * delta_t * delta_t;
        }
        return 0;
    }
    double profile_seg_vel(double time) {
        auto delta_t = time - segment.t0;
        auto temp_accel = segment.v1 > segment.v0 ? accel : -accel;

        if (profile == VProfile::CONSTANT)
            return target_velocity;

        return segment.v0 + temp_accel * delta_t;
    }

public:
    single_stepper(const fast_io &  pul, const fast_io &  dir, HardwareTimer * timer)
        : pul_pin(pul), dir_pin(dir), timer_on(timer) {
    }

    void init(bool _flip_dir = false) {
        flip_dir = _flip_dir;
        pul_pin.set_mode(OUTPUT);
        dir_pin.set_mode(OUTPUT);
    }

    void interpolate(double time) {
        if (profile == VProfile::CHANGING)
            if (time >= segment.t1) {
                profile = VProfile::CONSTANT;
                segment.t0 = time;
                segment.v0 = target_velocity;
                segment.p0 = current_step;
            }
        current_velocity = profile_seg_vel(time);
        temp_target_step = profile_seg_pos(time);
    }

    void update(const uint32 current_us, bool external_timing = false) {
        constexpr int32 interval_us = 5000;
        //const uint32_t ms = millis();

        SCHEDULER_GUARD(current_us, last_interpolate_us);
        SCHEDULER_GUARD(current_us, step_trigger_us);
        SCHEDULER_GUARD(current_us, last_step_finish_us);

        if (!external_timing) {
            if (last_interpolate_us > current_us)
                last_interpolate_us = current_us;
            if (current_us < last_interpolate_us + interval_us)
                return;

            last_interpolate_us = current_us;
        }

        if (accel == 0)
            return;

        double interval_sec = double(interval_us) / 1000000.0;
        auto increment = accel * interval_sec;

        if (abs(current_velocity - target_velocity) < 2 * increment) {
            current_velocity = target_velocity;
        }
        else {
            if (current_velocity < target_velocity)
                current_velocity += increment;
            else
                current_velocity -= increment;
        }
        temp_target_step_double += current_velocity * interval_sec;
        temp_target_step = temp_target_step_double;

        // refine tune velocity
        double practical_d = abs(temp_target_step - current_step);
        constexpr double increment_factor = 1000000.0 / interval_us;

        double main_v = current_velocity;
        double theory_d = main_v / increment_factor;
        if (practical_d > theory_d
            && practical_d > 0
            && theory_d > 0) {
            double r = practical_d / theory_d;
            r = constrain(r, 1, 2);
            main_v *= r;
        }

        set_instant_velocity(main_v);
    }

    void fast_stop() {
    }

    void set_accel(double a) {
        accel = a;
    }

    void set_peak_velocity(double v) {
        if (v == target_velocity)
            return;
        target_velocity = v;
    }

public:
    const fast_io pul_pin, dir_pin;
    HardwareTimer * timer_on;

    int32 current_step = 0, temp_target_step{ 0 };
    double temp_target_step_double{ 0.0 };
    bool flip_dir = false;

    bool timer_enabled = false;
    uint32 step_trigger_us = 0, last_step_finish_us = 0, last_interpolate_us = 0;
    bool full_stepped = true;
    bool waiting_new_pulse = false;

    double accel = 0;
    double current_velocity = 0;
    uint32 period_us = 0;
    int32 isr_velocity = 0;
    double target_velocity = 0;

    // ISR interlocks
    volatile bool isr_firing = false;

    void set_instant_velocity(int32 steps_per_sec) {
        isr_velocity = steps_per_sec;
        steps_per_sec = abs(steps_per_sec);

        uint32 period = 1000000;
        if (steps_per_sec != 0) period = 1000000 / steps_per_sec;
        period_us = period;

        if (steps_per_sec != 0) {
            set_timer_period(period, true);
        }
        else { // already reached target
            isr_velocity = 0;
            set_timer_period(1000000, false);
        }
    }

    // set time interval for main timer
    void set_timer_period(uint32 period, bool resume_after) {
        timer_enabled = false;
        // this method cause an immediately interrupt, so timer_enabled is used to prevent that!
        timer_on->pause();
        timer_on->setPeriod(period);
        if (resume_after) {
            timer_on->resume();
            timer_on->refresh(); // without this, the interrupt right next will not update right after setting
            timer_enabled = true;
        }
    }

    void change_step() {
        if (temp_target_step > current_step) {
            if (flip_dir)
                dir_pin.low();
            else
                dir_pin.high();
            current_step++;
#ifdef STEPPER_INVERT_PUL
            pul_pin.low();
#else
            pul_pin.high();
#endif
            full_stepped = false;
        }
        else if (temp_target_step < current_step) {
            if (flip_dir)
                dir_pin.high();
            else
                dir_pin.low();
            current_step--;
#ifdef STEPPER_INVERT_PUL
            pul_pin.low();
#else
            pul_pin.high();
#endif
            full_stepped = false;
        }
    }

    // make a step electrically
    void isr_on() {
        if (!timer_enabled || isr_firing)
            return;
        isr_firing = true;

        const uint32 current_us = micros();

        if (full_stepped
            && current_us >= last_step_finish_us + STEPPER_PULSE_DURATION) {
            // start a new step
            change_step();
            step_trigger_us = current_us;
        }
        else {
            // start a new wait command, isr_off will proceed to start a new step
            waiting_new_pulse = true;
        }
        isr_firing = false;
    }
    // flip step back to inactive state
    void isr_off(const uint32 current_us) {
        if (isr_firing)
            return;
        isr_firing = true;

        if (full_stepped
            && waiting_new_pulse
            && current_us >= last_step_finish_us + STEPPER_PULSE_DURATION) {
            waiting_new_pulse = false;
            // start a new step
            change_step();
            step_trigger_us = current_us;
        }
        if (!full_stepped
            && (current_us >= step_trigger_us + STEPPER_PULSE_DURATION)) {
            // end the step
            full_stepped = true;
#ifdef STEPPER_INVERT_PUL
            pul_pin.high();
#else
            pul_pin.low();
#endif
            last_step_finish_us = current_us;
        }
        isr_firing = false;
    }
};