#pragma once

#include <libmaple/gpio.h>

// WARNING: undefined behavior if bank and mask is not set

//#define FAST_IO_DEBUG

class fast_io {
private:
    gpio_dev * bank;
    uint32 mask;
    uint8 pin;
public:

    fast_io() {}

    fast_io(const fast_io & rhs)
        :bank(rhs.bank),
        mask(rhs.mask),
        pin(rhs.pin)
    {	}
    fast_io(fast_io & rhs)
        :bank(rhs.bank),
        mask(rhs.mask),
        pin(rhs.pin)
    {	}

    fast_io(uint8 _pin)
        : bank(digitalPinToPort(_pin)),
        mask(digitalPinToBitMask(_pin)),
        pin(_pin)
    {	}

    void set_mode(const WiringPinMode mode) const {
        pinMode(pin, mode);
    }
    void set_mode(const WiringPinMode mode) {
        const_cast<const fast_io*>(this)->set_mode(mode);
    }

    inline __always_inline uint8 read() const {
#ifdef FAST_IO_DEBUG
        return digitalRead(pin);
#else
        return (bank->regs->IDR & mask) ? HIGH : LOW;
#endif
    }

    inline __always_inline void low() const {
#ifdef FAST_IO_DEBUG
        digitalWrite(pin, LOW);
#else
        bank->regs->BRR = mask;
#endif
    }

    inline __always_inline void high() const {
#ifdef FAST_IO_DEBUG
        digitalWrite(pin, HIGH);
#else
        bank->regs->BSRR = mask;
#endif
    }

    inline __always_inline void toggle() const {
#ifdef FAST_IO_DEBUG
        digitalWrite(pin, !digitalRead(pin));
#else
        bank->regs->ODR = bank->regs->ODR ^ mask;
#endif
    }

    inline __always_inline void write(const uint8 val) const {
#ifdef FAST_IO_DEBUG
        digitalWrite(pin, val);
#else
        if (val)
            high();
        else
            low();
#endif
    }
};