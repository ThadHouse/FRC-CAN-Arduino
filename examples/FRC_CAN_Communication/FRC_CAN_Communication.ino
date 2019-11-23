
#include "frc/mcp2515.h"
#include "frc/CAN.h"

// Define the CS pin and the interrupt pin
#define CAN_CS 10
#define CAN_INTERRUPT 2

// Create an MCP2515 device. Only need to create 1 of these
frc::MCP2515 mcp2515{CAN_CS};

// Create an FRC CAN Device. You can create up to 16 of these in 1 progam
// Any more will overflow a global array
frc::CAN frcCANDevice{1};

// Callback function. This will be called any time a new message is received
// Matching one of the enabled devices.
void CANCallback(frc::CAN* can, int apiId, bool rtr, const frc::CANData& data) {
    Serial.println(apiId, HEX);
}

// Callback function for any messages not matching a known device.
// This would still have flags for RTR and Extended set, its a raw ID
void UnknownMessageCallback(uint32_t id, const frc::CANData& data) {

}


void setup() {
    // Initialize the MCP2515. If any error values are set, initialization failed
    auto err = mcp2515.reset();
    // CAN rate must be 1000KBPS to work with the FRC Ecosystem
    // Clock rate must match clock rate of CAN Board.
    err = mcp2515.setBitrate(frc::CAN_1000KBPS, frc::CAN_CLOCK::MCP_8MHZ);

    // Set up to normal CAN mode
    err = mcp2515.setNormalMode();

    // Prepare our interrupt pin
    pinMode(CAN_INTERRUPT, INPUT);
    
    // Set up FRC CAN to be able to use the CAN Impl and callbacks
    // Last parameter can be set to nullptr if unknown messages should be skipped
    frc::CAN::SetCANImpl(&mcp2515, CAN_INTERRUPT, CANCallback, UnknownMessageCallback);

    // All CAN Devices must be added to the read list. Otherwise they will not be handled correctly.
    frcCANDevice.AddToReadList();
}

unsigned long long lastSend20Ms = 0;
int count = 0;

void loop() {
    // Update must be called every loop in order to receive messages
    frc::CAN::Update();

    uint8_t data[8];

    // Writes can happen any time, this uses a periodic send
    auto now = millis();
    if (now - lastSend20Ms > 20) {
        // 20 ms periodic
        memset(data, 0, 8);
        data[0] = count;
        count++;

        frcCANDevice.WritePacket(data, 1, 1);
    }
}
