#pragma once

#include "fast_io.h"
#include "Schedule.h"

#ifndef STEPPER_PULSE_DURATION
#define STEPPER_PULSE_DURATION 10 // us
#endif

#ifndef STEPPER_ACTIVE_LEVEL
#define STEPPER_ACTIVE_LEVEL HIGH
#endif

class SingleStepper {
public:
  SingleStepper(const fast_io & pul_pin, const fast_io & dir_pin, HardwareTimer * const timer, const bool invert_dir = false)
	: m_pul_pin(pul_pin), m_dir_pin(dir_pin), m_timer_on(timer) {
	m_invert_direction = invert_dir;
  }

  void init() {
	m_pul_pin.set_mode(OUTPUT);
	m_dir_pin.set_mode(OUTPUT);
  }

  void setAccel(const float & accel) {
	m_accel = accel;
  }

  void setVelocity(const float & velocity) {
	m_max_velocity = velocity;
  }

  void fastStop() {
	m_current_velocity = 0;
	setInstantVelocity(0);
  }

  void update() {
	const uint32_t current_us = micros();
	constexpr uint32_t interval_us = 10000;
	constexpr double interval_sec = double(interval_us) / 1000000.0;

	SCHEDULER_GUARD(current_us, m_last_update_us);
	if (!onSchedule(current_us, m_last_update_us, 10000))
	  return;

	auto increment = m_accel * interval_sec;

	if (m_current_velocity <= m_max_velocity - increment) {
	  m_current_velocity += increment;
	}
	else if (m_current_velocity >= m_max_velocity + increment) {
	  m_current_velocity -= increment;
	}
	if (abs(m_current_velocity - m_max_velocity) < increment * 2) {
	  m_current_velocity = m_max_velocity;
	}

	if (m_current_velocity != m_last_velocity) {
	  m_last_velocity = m_current_velocity;
	  setInstantVelocity(2 * m_current_velocity);
	}
  }

  // make a step electrically
  void isr_on() {
	if (!m_timer_enabled)
	  return;

	if (m_isr_velocity > 0) {
	  m_dir_pin.write(m_invert_direction);
	  m_pul_pin.toggle();
	}
	else if (m_isr_velocity < 0) {
	  m_dir_pin.write(!m_invert_direction);
	  m_pul_pin.toggle();
	}
  }
  // flip step back to inactive state
  void isr_off(const uint32 current_us) {
  }

  //private:
  void setInstantVelocity(int32_t steps_per_sec) {
	uint32 period = 1000000;
	m_isr_velocity = steps_per_sec;
	steps_per_sec = abs(steps_per_sec);
	if (steps_per_sec != 0) period = 1000000 / steps_per_sec;

	setTimerPeriod(period, steps_per_sec != 0);
  }

  // set time interval for main timer
  void setTimerPeriod(uint32 period, bool resume_after) {
	m_timer_enabled = false;
	// this method cause an immediately interrupt, so timer_enabled is used to prevent that!
	m_timer_on->pause();
	m_timer_on->setPeriod(period);
	if (resume_after) {
	  m_timer_on->resume();
	  m_timer_on->refresh(); // without this, the interrupt right next will not update right after setting
	  m_timer_enabled = true;
	}
  }

  const fast_io m_pul_pin;
  const fast_io m_dir_pin;
  bool m_invert_direction{ false };
  uint32_t m_last_update_us{ 0 };

  volatile bool m_timer_enabled{ false };

  volatile int32 m_isr_velocity{ 0 };
  double m_current_velocity{ 0.0 };
  double m_last_velocity{ 0.0 };
  double m_max_velocity{ 0.0 };
  double m_accel{ 0.0 };

  volatile bool m_isr_firing{ false };

public:
  HardwareTimer * const m_timer_on;
};
