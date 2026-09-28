#pragma once

#include "motor/motor.h"
#include "uv/uv.h"

#include <Arduino.h>

/// @brief A task that runs on core 0 and performs communication functions
/// @param pvParameters
void communicationTask(void* pvParameters);

/// Intercore communication queue
extern QueueHandle_t intercoreQueue;

/// @brief Bind communication task to a core 0 and init internal vars
void startCommunication();

union CommunicationUnion {
    UVTask ut;          // 40
    MotorTargetTask mt; // 50

    uint16_t current;    // 52
    uint16_t microsteps; // 54

    // <HOME>            // 56
    // <DISABLE STEPPER> // 58
    // <ENABLE STEPPER>  // 60

    // <GET STEPPER POS> // 62

    // System fan control
    u_int8_t sys_fan_power; // 70
};

/// @brief Communication command structure containing a command identifier and a payload.
struct CommunicationStruct {
    size_t field_id;
    CommunicationUnion u;
};

bool receiveQueue(CommunicationStruct* cs);