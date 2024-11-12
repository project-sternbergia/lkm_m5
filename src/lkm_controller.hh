#ifndef LKM_CONTROLLER_HH
#define LKM_CONTROLLER_HH

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "lkm_can_driver.hh"
#include "lkm_can_interface.hh"

namespace lkm_m5
{
struct MotorConfig
{
  uint8_t id;
  uint8_t motor_type;
  uint8_t encoder_type;
};

typedef std::shared_ptr<lkm_m5::can::Driver> DriverPtr;

class Controller
{
public:
  explicit Controller(uint8_t master_can_id);
  virtual ~Controller();

  bool init(
    can::CanInterface * p_can, const std::vector<MotorConfig> & configs,
    uint16_t wait_response_time_usec = 0);
  bool motor_on();
  bool motor_off();
  bool motor_stop();
  DriverPtr driver(uint8_t id);
  bool process_packet();

private:
  uint8_t master_can_id_;
  lkm_m5::can::CanInterface * p_can_;
  uint8_t receive_buffer_[64];  //!< receive buffer
  std::vector<MotorConfig> configs_;
  std::unordered_map<uint8_t, DriverPtr> drivers_;
};

};  // end namespace lkm_m5

#endif  // !LKM_CONTROLLER_HH
