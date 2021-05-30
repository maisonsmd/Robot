#pragma once

class LedFlasher {
public:
    LedFlasher() {}
    LedFlasher(LedFlasher &) = delete;
    LedFlasher(LedFlasher &&) = delete;
    LedFlasher(const uint8_t & pin = 255, const uint8_t & active_level = HIGH) {
        attach(pin, active_level);
    }

    ~LedFlasher() {
        if (m_pointer_mutable && m_pattern)
            delete[] m_pattern;
    }

    void attach(const uint8_t & pin, const uint8_t & active_level = HIGH) {
        m_pin = pin;
        m_active_level = active_level;
        pinMode(m_pin, OUTPUT);
        digitalWrite(m_pin, !m_active_level);
    }

    void setRunning(const bool & running) {
        // light up for the first duration
        if (!m_running && running && m_pattern && m_pin != 255) {
            digitalWrite(m_pin, m_active_level);
            m_next_ms = millis() + m_pattern[0];
        }

        m_running = running;
    }

    void setLoop(const bool & loop) {
        m_loop = loop;
    }

    void setPattern(uint32_t * const pattern, const uint8_t & length, const bool & is_array_mutable) {
        if (m_pointer_mutable && m_pattern && m_pattern != pattern)
            delete[] m_pattern;

        m_current_index = 0;
        m_next_ms = 0;

        m_pattern = pattern;
        m_length = length;
        m_pointer_mutable = is_array_mutable;

        if (pattern) {
            if (m_pin != 255)
                digitalWrite(m_pin, m_active_level);
            m_next_ms = millis() + pattern[0];
        }
    }

    void on() {
        setPattern(nullptr, 0, false);
        digitalWrite(m_pin, m_active_level);
    }

    void off() {
        setPattern(nullptr, 0, false);
        digitalWrite(m_pin, !m_active_level);
    }

    void update() {
        // falling edge update (timeout)
        const uint32_t current_ms = millis();
        // overflow guard
        if (m_next_ms < current_ms)
            m_next_ms = current_ms;

        if (current_ms < m_next_ms)
            return;

        if (!m_pattern || !m_running || m_pin == 255)
            return;

        m_current_index++;

        if (m_current_index >= m_length) {
            if (!m_loop)
                setRunning(false);
            m_current_index = 0;
        }

        digitalWrite(m_pin, m_current_index % 2 == 0 ? m_active_level : !m_active_level);
        m_next_ms = current_ms + m_pattern[m_current_index];
    }

private:
    uint8_t m_pin{ 255 };
    uint8_t m_active_level{ HIGH };

    bool m_loop{ true };
    bool m_running{ true };

    bool m_pointer_mutable{ false };
    uint8_t m_current_index{ 0 };
    uint32_t * m_pattern{ nullptr };
    uint8_t m_length{ 0 };
    uint32_t m_next_ms{ 0 };
};