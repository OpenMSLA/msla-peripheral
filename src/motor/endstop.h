#pragma once

/// @brief Init endstop and attach interrupt
void initEndstop();

class Endstop {
  public:
    volatile bool triggered = false;
};

extern Endstop end;