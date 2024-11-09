#ifndef LKM_CAN_INTERFACE_TWAI_HH
#define LKM_CAN_INTERFACE_TWAI_HH

#include "lkm_can_interface.hh"

#define M5_ESP32_DEFAULT_RX_PIN 1
#define M5_ESP32_DEFAULT_TX_PIN 2

namespace lkm_m5::can
{
class CanInterfaceTwai : public CanInterface
{
public:
  CanInterfaceTwai();
  virtual ~CanInterfaceTwai();
  bool init(uint8_t rx_pin = M5_ESP32_DEFAULT_RX_PIN, uint8_t tx_pin = M5_ESP32_DEFAULT_TX_PIN);

  virtual size_t write(uint64_t id, const Frame & data, bool is_ext = false);
  virtual size_t write(uint64_t id, const uint8_t * buffer, size_t size, bool is_ext = false);
  virtual size_t read(CanPacket & packet);
  virtual size_t available();
};
}  // namespace lkm_m5::can

#endif  // LKM_CAN_INTERFACE_TWAI_HH
