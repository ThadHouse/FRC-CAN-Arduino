#include "frc/CAN.h"
#include "frc/mcp2515.h"

using namespace frc;

frc::MCP2515 *CAN::m_mcp2515 = nullptr;
uint8_t CAN::m_interruptPin = 0;
CAN::MessageCallback CAN::m_messageCb = nullptr;
CAN::UnknownMessageCallback CAN::m_unknownMessageCb = nullptr;
CAN *CAN::m_canClasses[16];
uint8_t CAN::m_canCount = 0;

void CAN::SetCANImpl(MCP2515 *mcp, uint8_t iPin, CAN::MessageCallback cb, CAN::UnknownMessageCallback ucb)
{
    m_mcp2515 = mcp;
    m_interruptPin = iPin;
    m_messageCb = cb;
    m_unknownMessageCb = ucb;
}

void CAN::Update()
{
    can_frame frame;
    if (!digitalRead(m_interruptPin))
    {
        auto err = m_mcp2515->readMessage(&frame);
        if (err == MCP2515::ERROR::ERROR_OK)
        {
            auto masked = frame.can_id & CAN_EFF_MASK;
            for (uint8_t i = 0; i < m_canCount; i++)
            {
                auto canClass = m_canClasses[i];
                if (canClass == nullptr)
                    return;
                auto matchesId = canClass->matchesId(masked);
                if (matchesId >= 0)
                {
                    CANData message;
                    message.length = frame.can_dlc;
                    message.timestamp = millis();
                    memcpy(message.data, frame.data, message.length);
                    m_messageCb(canClass, matchesId, (frame.can_id & CAN_RTR_FLAG) != 0, message);
                    return;
                }
            }
            if (m_unknownMessageCb)
            {
                CANData message;
                message.length = frame.can_dlc;
                message.timestamp = millis();
                memcpy(message.data, frame.data, message.length);
                m_unknownMessageCb(frame.can_id, message);
            }
        }
    }
}

void CAN::AddToReadList()
{
    m_canClasses[m_canCount] = this;
    m_canCount++;
}

void CAN::RemoveFromReadList()
{
    for (int i = 0; i < m_canCount; i++)
    {
        if (m_canClasses[i] == this)
        {
            m_canClasses[i] == nullptr;
        }
    }
}

bool CAN::WritePacket(const uint8_t *data, uint8_t length, int apiId)
{
    can_frame frame;
    memcpy(frame.data, data, length);
    frame.can_dlc = length;
    frame.can_id = m_messageMask;
    frame.can_id |= apiId << 6;
    frame.can_id |= CAN_EFF_FLAG;

    return m_mcp2515->sendMessage(&frame) == MCP2515::ERROR::ERROR_OK;
}

bool CAN::WriteRTRFrame(uint8_t length, int apiId)
{
    can_frame frame;
    memset(frame.data, 0, 8);
    frame.can_dlc = length;
    frame.can_id = m_messageMask;
    frame.can_id |= apiId << 6;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_id |= CAN_RTR_FLAG;

    return m_mcp2515->sendMessage(&frame) == MCP2515::ERROR::ERROR_OK;
}
