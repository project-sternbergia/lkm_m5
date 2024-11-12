#include "lkm_can_interface_mcp2515.hh"

#include "RingBuf.h"
#ifdef USE_ARDUINO_MCP2515
#include "mcp_can.h"

using namespace lkm_m5::can;

CanInterfaceMcp2515::CanInterfaceMcp2515() : CanInterface(), p_can_() {}

CanInterfaceMcp2515::~CanInterfaceMcp2515() {}

bool CanInterfaceMcp2515::init(int cs_pin, int int_pin, uint64_t baudrate)
{
  uint8_t baud = CAN_1000KBPS;
  switch (baudrate) {
    case (uint64_t)125E3:
      baud = CAN_125KBPS;
      break;
    case (uint64_t)200E3:
      baud = CAN_200KBPS;
      break;
    case (uint64_t)250E3:
      baud = CAN_250KBPS;
      break;
    case (uint64_t)500E3:
      baud = CAN_500KBPS;
      break;
    case (uint64_t)1000E3:
      baud = CAN_1000KBPS;
      break;
  }

  p_can_ = new MCP_CAN(cs_pin);
  if (p_can_->begin(MCP_ANY, baud, MCP_8MHZ) != CAN_OK) {
    LKM_DEBUG_PRINTLN("Failed to begin can controller.");
    return false;
  }

  LKM_DEBUG_PRINTLN("Succeeded to open can controller.");
  p_can_->setMode(MCP_NORMAL);
  pinMode(int_pin, INPUT);
  return true;
}

size_t CanInterfaceMcp2515::write(uint64_t id, const Frame & data, bool is_ext)
{
  return write(id, data.data(), data.size(), is_ext);
}

size_t CanInterfaceMcp2515::write(uint64_t id, const uint8_t * buffer, size_t size, bool is_ext)
{
  memcpy(send_buffer_, buffer, size);
  uint8_t ret = p_can_->sendMsgBuf(id, is_ext, size, send_buffer_);
  if (ret != CAN_OK) {
    LKM_DEBUG_PRINTF("Failed to send can data. error=[0x%x]\n", ret);
    return 0;
  }

  LKM_DEBUG_PRINTLN("Succeeded to send can data.");
  return size;
}

size_t CanInterfaceMcp2515::read(CanPacket & packet)
{
  unsigned long id;
  uint8_t len;
  uint8_t ext;
  uint8_t ret = p_can_->readMsgBuf(&id, &ext, &len, receive_buffer_);
  if (ret == CAN_OK) {
    packet.is_extended = (ext != 0);
    packet.is_rtr = false;
    packet.rx_id = id;
    packet.tx_id = 0x00;
    packet.dlc = len;
    packet.data = Frame(receive_buffer_, receive_buffer_ + len);
    packet.stamp = micros();
    LKM_DEBUG_PRINTF("Receive can msg. id=[%x] dlc=[%x].\n", id, len);
    return 1;  // MCP2515 does not provide api for getting number of data
  }

  return 0;
}

size_t CanInterfaceMcp2515::available()
{
  uint8_t ret = p_can_->checkReceive();
  return (ret == CAN_MSGAVAIL) ? 1 : 0;
}

#endif  // ARDUINO_MCP2515
