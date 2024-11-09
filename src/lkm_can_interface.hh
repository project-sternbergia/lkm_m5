#ifndef LKM_CAN_INTERFACE
#define LKM_CAN_INTERFACE

#include "lkm_driver_common.hh"

namespace lkm_m5::can
{
struct CanPacket
{
  unsigned long stamp;  //!< timestamp
  uint64_t tx_id;       //!< ID
  uint64_t rx_id;       //!< ID
  bool is_extended;     //!< is extended packet
  bool is_rtr;          //!< is rtr packet
  uint8_t dlc;          //!< data size
  Frame data;           //!< data
};

class CanInterface
{
public:
  CanInterface() {}

  virtual ~CanInterface() {}

  virtual size_t write(uint64_t id, const Frame & data, bool is_ext = false) { return 0; }

  virtual size_t write(uint64_t id, const uint8_t * buffer, size_t size, bool is_ext = false)
  {
    return 0;
  }

  virtual size_t read(CanPacket & packet) { return 0; }

  virtual size_t available() { return 0; }
};
};  // namespace lkm_m5::can

#endif  // LKM_CAN_INTERFACE
