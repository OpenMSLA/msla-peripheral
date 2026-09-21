#pragma once

/// @brief Init endstop and attach interrupt
void initEndstop();

class Endstop {
public:
    volatile bool triggered = false;

    /// Trigger manual update of endstop state
    bool manualUpdate();
};

extern Endstop end;