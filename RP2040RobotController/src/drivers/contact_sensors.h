#pragma once

#include <array>

#include "locomotion/contact_adaptation.h"

namespace hexapod {

class ContactSensors {
public:
    void init();
    void update(float dt_s);
    const ContactDebouncer& debouncer() const { return debouncer_; }
    std::array<bool, kLegCount> contacts() const { return debouncer_.contacts(); }

private:
    std::array<bool, kLegCount> readRaw() const;

    ContactDebouncer debouncer_{};
    bool initialized_ = false;
};

}  // namespace hexapod
