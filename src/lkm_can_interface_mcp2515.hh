#ifndef LKM_CAN_INTERFACE_MCP2515
#define LKM_CAN_INTERFACE_MCP2515
#include "lkm_can_interface.hh"

class MCP_CAN;

namespace lkm_m5::can
{
class CanInterfaceMcp2515 : public CanInterface
{
public:
  CanInterfaceMcp2515();
  virtual ~CanInterfaceMcp2515();
  bool init(int cs_pin, int int_pin, uint64_t baudrate);
  virtual size_t write(uint64_t id, const Frame & data, bool is_ext = false);
  virtual size_t write(uint64_t id, const uint8_t * buffer, size_t size, bool is_ext = false);
  virtual size_t read(CanPacket & packet);
  virtual size_t available();

private:
  MCP_CAN * p_can_;
  uint8_t receive_buffer_[32];
  uint8_t send_buffer_[32];
};
};  // namespace lkm_m5::can

#endif  // LKM_CAN_INTERFACE_MCP2515
