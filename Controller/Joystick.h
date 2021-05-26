#pragma once
#include "Schedule.h"

class Joystick {
public:
    Joystick(const int32_t max_adc, const float minimum_threshold, const uint8_t pin_x, const uint8_t pin_y)
        : m_max_adc(max_adc),
        m_minimum_threshold(minimum_threshold),
        m_pin_x(pin_x),
        m_pin_y(pin_y) {
    }

    void update() {
        SCHEDULER_GUARD(millis(), m_last_update_ms);
        if (!onSchedule(millis(), m_last_update_ms, 20))
            return;

        // do a simple RC filter
        constexpr float alpha = 0.7;
        if (m_pin_x != 255) {
            float x = float(analogRead(m_pin_x)) * 2.0f / m_max_adc - 1.0f;
            if (abs(x) < m_minimum_threshold) x = 0.0f;
            m_last_x = alpha * m_last_x + (1.0f - alpha) * x;

            if (m_last_x > 0.99f)
                m_last_x = 1.0f;

            if (m_last_x < -0.99f)
                m_last_x = -1.0f;
        }

        if (m_pin_y != 255) {
            float y = float(analogRead(m_pin_y)) * 2.0f / m_max_adc - 1.0f;
            if (abs(y) < m_minimum_threshold) y = 0.0f;
            m_last_y = alpha * m_last_y + (1.0f - alpha) * y;

            if (m_last_y > 0.99f)
                m_last_y = 1.0f;

            if (m_last_y < -0.99f)
                m_last_y = -1.0f;
        }
    }

    float getX() {
        return m_last_x;
    }

    float getY() {
        return m_last_y;
    }
private:
    const int32_t m_max_adc;
    const float m_minimum_threshold;
    uint8_t m_pin_x, m_pin_y;
    float m_last_x{ 0.0f }, m_last_y{ 0.0f };
    uint32_t m_last_update_ms{ 0 };
};
