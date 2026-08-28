#pragma once

#include <array>

#include "locomotion/contact_adaptation.h"

namespace hexapod {

class ContactSensors {
public:
    void init();
    void update(float dt_s);
    const ContactManager& manager() const { return manager_; }
    const std::array<ContactState, kLegCount>& states() const { return manager_.states(); }
    std::array<bool, kLegCount> contacts() const { return manager_.stableContacts(); }
    std::array<bool, kLegCount> rawContacts() const { return manager_.rawContacts(); }

private:
    std::array<bool, kLegCount> readRaw() const;

    ContactManager manager_{};
    bool initialized_ = false;
};

}  // namespace hexapod
