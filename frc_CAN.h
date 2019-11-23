#pragma once

#include <stdint.h>

namespace frc
{
class MCP2515;
struct CANData
{
  uint8_t data[8];
  uint8_t length;
  uint32_t timestamp;
};

enum class CANDeviceType
{
  kBroadcast = 0,
  kRobotController = 1,
  kMotorController = 2,
  kRelayController = 3,
  kGyroSensor = 4,
  kAccelerometer = 5,
  kUltrasonicSensor = 6,
  kGearToothSensor = 7,
  kPowerDistribution = 8,
  kPneumatics = 9,
  kMiscellaneous = 10,
  kFirmwareUpdate = 31
};

enum class CANManufacturer
{
  kBroadcast = 0,
  kNI = 1,
  kLM = 2,
  kDEKA = 3,
  kCTRE = 4,
  kREV = 5,
  kGrapple = 6,
  kMS = 7,
  kTeamUse = 8,
  kKauaiLabs = 9,
  kCopperforge = 10,
  kPWF = 11,
  kStudica = 12
};

constexpr uint32_t createCANId(uint16_t apiId, uint8_t deviceId, uint8_t manufacturer, uint8_t deviceType)
{
  return (static_cast<uint32_t>(deviceType) & 0x1F) << 24 | (static_cast<uint32_t>(manufacturer) & 0xFF) << 16 | (apiId & 0x3FF) << 6 | (deviceId & 0x3F);
}

class CAN
{
public:
  typedef void (*MessageCallback)(CAN *can, int apiId, bool rtr, const CANData& message);
  typedef void (*UnknownMessageCallback)(uint32_t id, const CANData& message);
  static void SetCANImpl(MCP2515 *mcp2515, uint8_t interruptPin, MessageCallback messageCallback, UnknownMessageCallback unknownCallback);

  static void Update();

  constexpr CAN(int deviceId, CANManufacturer deviceManufacturer, CANDeviceType deviceType) 
    : m_messageMask{createCANId(0, deviceId, static_cast<uint8_t>(deviceManufacturer), static_cast<uint8_t>(deviceType))}
  {
  }

  constexpr explicit CAN(int deviceId) : CAN{deviceId, CANManufacturer::kTeamUse, CANDeviceType::kMiscellaneous}
  {
  }

  void AddToReadList();

  void RemoveFromReadList();

  bool WritePacket(const uint8_t *data, uint8_t length, int apiId);

  bool WriteRTRFrame(uint8_t length, int apiId);

private:
  static MCP2515* m_mcp2515;
  static uint8_t m_interruptPin;
  static CAN::MessageCallback m_messageCb; 
  static CAN::UnknownMessageCallback m_unknownMessageCb; 
  static CAN* m_canClasses[16];
  static uint8_t m_canCount;

  int16_t matchesId(int32_t id) {
    constexpr int32_t mask = 0x1FFF003F;
    constexpr int32_t filter = 0x0000FFC0;
    if ((id & mask) == m_messageMask) {
      return (id & filter) >> 6;
    }
    return -1;
  }
  uint32_t m_messageMask = 0;
};
} // namespace frc