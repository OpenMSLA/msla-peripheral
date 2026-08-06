#include "endstop.h"
#include "config.h"
#include "driver/gpio.h"
#include <Arduino.h>

bool Endstop::manualUpdate() {
    triggered = digitalRead(Z_ENDSTOP_PIN) ^ Z_ENDSTOP_INVERSE;
    return triggered;
};

Endstop end;

void IRAM_ATTR handleInterrupt() { end.triggered = true; }

void initEndstop() {
    pinMode(Z_ENDSTOP_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Z_ENDSTOP_PIN), handleInterrupt,
        Z_ENDSTOP_INVERSE ? FALLING : RISING);
}