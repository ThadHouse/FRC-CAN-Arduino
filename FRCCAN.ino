
#include "mcp2515.h"

#include "frc/CAN.h"

#define CAN_CS 10
#define CAN_INTERRUPT 2

MCP2515 mcp2515{CAN_CS};
frc::CAN frcCANDevice{1};

void CANCallback(frc::CAN* can, int apiId) {

}


void setup() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS);
    mcp2515.setNormalMode();
    pinMode(CAN_INTERRUPT, OUTPUT);
    frc::CAN::SetCANImpl(&mcp2515, CAN_INTERRUPT, CANCallback);
}

unsigned long long lastSend20Ms = 0;
int count = 0;

void loop() {
    frc::CAN::Update();

    uint8_t data[8];

    auto now = millis();
    if (now - lastSend20Ms > 20) {
        // 20 ms periodic
        memset(data, 0, 8);
        data[0] = count;
        count++;

        frcCANDevice.WritePacket(data, 1, 1);
    }
}