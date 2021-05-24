#pragma once

#ifndef BUTTONS_ACTIVE_STATE
#define BUTTONS_ACTIVE_STATE			LOW
#endif
#ifndef BUTTONS_LONG_PRESS_INTERVAL
#define BUTTONS_LONG_PRESS_INTERVAL		700000
#endif
#ifndef BUTTONS_DEBOUNCE_INTERVAL
#define BUTTONS_DEBOUNCE_INTERVAL		35000
#endif
#ifndef BUTTONS_ACCEPT_INTERVAL
#define BUTTONS_ACCEPT_INTERVAL			1000
#endif
#ifndef BUTTONS_MIN_KEYSTROKES_INTERVAL
#define BUTTONS_MIN_KEYSTROKES_INTERVAL 10000
#endif
#ifndef BUTTONS_CONTINOUS_PRESS_INTERVAL
#define BUTTONS_CONTINOUS_PRESS_INTERVAL 70000
#endif

#define ACCCEPT_PRESS_AFTER_LONG_PRESS true

enum class ButtonState
{
	Holding,
	Released,
	WaitingSensorAccept,
	LongPressed
};

class Button
{
private:
	uint8_t m_pin;
	ButtonState state = ButtonState::Released;
	boolean buttonPressEnabled = true;
	boolean skipThisStroke = false;
	uint32_t continuousPressCount = 0;

	uint32_t lastActive_us = 0;
	uint32_t pressStart_us = 0;
	uint32_t lastKeyUp_us = 0;
	uint32_t lastConinousPress_us = 0;

	const uint8_t m_activeState;
	const uint32_t m_debounceInterval;
	const uint32_t m_longPressInterval;
	const uint32_t m_minAcceptInterval;
	const uint32_t m_minKeystrokesInterval;
	const uint32_t m_continousPressInterval;
public:
	void(*onSensorTriggered)() = nullptr;
	void(*onSensorReleased)() = nullptr;
	void(*onButtonPressed)() = nullptr;
	void(*onButtonContinousPress)(uint32_t) = nullptr;
	boolean(*onButtonLongPress)() = nullptr;

	Button(const uint8_t activeState = BUTTONS_ACTIVE_STATE,
		const uint32_t debounceInterval = BUTTONS_DEBOUNCE_INTERVAL,
		const uint32_t longPressInterval = BUTTONS_LONG_PRESS_INTERVAL,
		const uint32_t minAcceptInterval = BUTTONS_ACCEPT_INTERVAL,
		const uint32_t keystrokesInterval = BUTTONS_MIN_KEYSTROKES_INTERVAL,
		const uint32_t continousPressInterval = BUTTONS_CONTINOUS_PRESS_INTERVAL)
		: m_activeState(activeState),
		m_debounceInterval(debounceInterval),
		m_longPressInterval(longPressInterval),
		m_minAcceptInterval(minAcceptInterval),
		m_minKeystrokesInterval(keystrokesInterval),
		m_continousPressInterval(continousPressInterval) {	}

	void attach(const uint8_t& pin, bool pullup = false) volatile {
		m_pin = pin;
		pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
	}

	void update(uint8_t externalState = 255) volatile {
		const uint32_t current_us = micros();
		if (externalState == 255 && m_pin == 255)
		  return;
		uint8_t digitalState = (externalState != 255) ? externalState : digitalRead(m_pin);

		// timer guard
		if (current_us < lastActive_us) lastActive_us = current_us;
		if (current_us < pressStart_us) pressStart_us = current_us;
		if (current_us < lastKeyUp_us) lastKeyUp_us = current_us;
		if (current_us < lastConinousPress_us) lastConinousPress_us = current_us;

		if (state == ButtonState::LongPressed
			&& current_us >= lastConinousPress_us + m_continousPressInterval) {
			lastConinousPress_us = current_us;

			if (onButtonContinousPress)
				onButtonContinousPress(continuousPressCount++);
		}

		if (digitalState == m_activeState) {
			lastActive_us = current_us;

			if (state == ButtonState::Released) {
				pressStart_us = current_us;
				if (current_us < lastKeyUp_us + m_minKeystrokesInterval)
					skipThisStroke = true;
				state = ButtonState::WaitingSensorAccept;
			}

			if (state == ButtonState::WaitingSensorAccept
				&& current_us > pressStart_us + m_minAcceptInterval) {
				state = ButtonState::Holding;

				buttonPressEnabled = true;
				continuousPressCount = 0;

				if (onSensorTriggered && !skipThisStroke)
					onSensorTriggered();
				if (onButtonContinousPress && !skipThisStroke)
					onButtonContinousPress(continuousPressCount++);
			}
		}
		else if (state == ButtonState::WaitingSensorAccept)
			state = ButtonState::Released;

		if (current_us > pressStart_us + m_longPressInterval
			&& state == ButtonState::Holding) {
			state = ButtonState::LongPressed;
			if (!skipThisStroke)
				buttonPressEnabled = onButtonLongPress ? onButtonLongPress() : false;
		}

		if (current_us > lastActive_us + m_debounceInterval
			&& (state == ButtonState::Holding
				|| state == ButtonState::LongPressed)) {
			state = ButtonState::Released;

			if (onSensorReleased && !skipThisStroke)
				onSensorReleased();
			if (onButtonPressed && buttonPressEnabled && !skipThisStroke)
				onButtonPressed();

			lastKeyUp_us = current_us;
			skipThisStroke = false;
		}
	}
	boolean isTriggered() volatile {
		return (state == ButtonState::Holding
			|| state == ButtonState::LongPressed);
	}
};
