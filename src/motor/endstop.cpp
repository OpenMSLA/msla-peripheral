#include "endstop.h"
#include "config.h"
#include "driver/gpio.h"
#include <Arduino.h>

Endstop end;

void IRAM_ATTR handleInterrupt() { end.triggered = true; }

void initEndstop() {
    pinMode(Z_ENDSTOP_PIN, INPUT_PULLUP);
    attachInterrupt(
        digitalPinToInterrupt(Z_ENDSTOP_PIN), handleInterrupt, FALLING);
}