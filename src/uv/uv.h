#pragma once

#include "uart/packet.h"

#include <Arduino.h>

struct UVTask {
    bool state;
};

/// @brief Parse packet reader to a uv task
UVTask parseUVTask(PacketReader* pr);

/// @brief Execute uv task in hw
void executeUVTask(UVTask uv);

/// Initialize UV light
void initUV();

/// @brief Get current uv state
/// @return UV state
bool getUVState();