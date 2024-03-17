#include "lkm_driver.hh"
#include "lkm_driver_defs.hh"
#include "lkm_can_packet.hh"
#include <Arduino.h>

using namespace lkm_m5;

Driver::Driver(uint8_t host_id, uint8_t target_id, uint8_t motor_type, uint8_t encoder_type)
  : host_id_(host_id)
  , target_id_(target_id)
  , motor_type_(motor_type)
  , encoder_type_(encoder_type)
{}

Driver::~Driver()
{}

void Driver::init()
{
  // NOT IMPLEMENTED
}

bool Driver::motor_on()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::motor_off()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::stop_motor()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::open_loop_control(int16_t power)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::torque_closed_loop_control(float iq_control)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::speed_closed_loop_control(float speed)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::multi_loop_angle_control(float angle)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::multi_loop_angle_with_speed_control(float angle, float speed)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::single_loop_angle_control(uint8_t spin_dir, float angle)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::single_loop_angle_with_speed_control(uint8_t spin_dir, float angle, float speed)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::increment_angle_control(float angle)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::increment_angle_with_speed_control(float angle, float speed)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::read_pid_parameter()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::write_pid_parameter_to_ram(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::write_pid_parameter_to_rom(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::read_acceleration()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::write_acceleration_to_ram(float acc)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::read_encoder()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::write_encoder_value_to_rom_as_zero_point(uint16_t encoder_offset)
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::write_current_position_to_rom_as_zero_point()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::read_multi_angle_loop()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::read_single_angle_loop()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::clear_motor_angle_loop()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::read_motor_state1()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::clear_motor_error_state()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::read_motor_state2()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::read_motor_state3()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::process_packet()
{
  // NOT IMPLEMENTED
  return false;
}

bool Driver::process_response_packet(const Frame& frame)
{
  // NOT IMPLEMENTED
  return false;
}
