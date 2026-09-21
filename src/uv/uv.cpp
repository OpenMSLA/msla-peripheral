#include "uv.h"

#include "config.h"
#include "uart/communication.h"

void initUV() {
    pinMode(UV_RELAY_PIN, OUTPUT);
}

UVTask parseUVTask(PacketReader* pr) {
    UVTask uv;

    uv.state = pr->readUInt8() == 1;
    return uv;
}

void executeUVTask(UVTask uv) {
    digitalWrite(UV_RELAY_PIN, uv.state);
}