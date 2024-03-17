#include "lkm_rs485_driver.hh"
#include "lkm_driver.hh"
#include "lkm_driver_defs.hh"
#include "lkm_driver_common.hh"
#include "lkm_rs485_packet.hh"
#include "lkm_rs485_packet_defs.hh"
#include <Arduino.h>

using namespace lkm_m5;

namespace lkm_m5::rs485
{

Driver::Driver(uint8_t master_id, uint8_t target_id, uint8_t motor_type, uint8_t encoder_type)
  : lkm_m5::Driver(master_id, target_id, motor_type, encoder_type)
  , send_count_(0)
{}

Driver::~Driver()
{}

void Driver::init(Stream* p_stream)
{
  LKM_DEBUG_FUNC
  stream_ = p_stream;
}

bool Driver::motor_on()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_MOTOR_ON, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::motor_off()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_MOTOR_OFF, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::stop_motor()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_MOTOR_STOP, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::open_loop_control(int16_t power)
{
  LKM_DEBUG_FUNC
  auto packet = OpenLoopControlRequestPacket(target_id(), power);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::torque_closed_loop_control(float iq_control)
{
  LKM_DEBUG_FUNC
  auto packet = TorqueClosedLoopControlRequestPacket(target_id(), iq_control, motor_type());
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::speed_closed_loop_control(float speed)
{
  LKM_DEBUG_FUNC
  auto packet = SpeedClosedLoopControlRequestPacket(target_id(), speed);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::multi_loop_angle_control(float angle)
{
  LKM_DEBUG_FUNC
  auto packet = MultiLoopAngleControlRequestPacket(target_id(), angle);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::multi_loop_angle_with_speed_control(float angle, float speed)
{
  LKM_DEBUG_FUNC
  auto packet = MultiLoopAngleControlWithSpeedRequestPacket(target_id(), angle, speed);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::single_loop_angle_control(uint8_t spin_dir, float angle)
{
  LKM_DEBUG_FUNC
  auto packet = SingleLoopAngleControlRequestPacket(target_id(), spin_dir, angle);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::single_loop_angle_with_speed_control(uint8_t spin_dir, float angle, float speed)
{
  LKM_DEBUG_FUNC
  auto packet = SingleLoopAngleControlWithSpeedRequestPacket(target_id(), spin_dir, angle, speed);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::increment_angle_control(float angle)
{
  LKM_DEBUG_FUNC
  auto packet = IncrementAngleControlRequestPacket(target_id(), angle);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::increment_angle_with_speed_control(float angle, float speed)
{
  LKM_DEBUG_FUNC
  auto packet = IncrementAngleControlWithSpeedRequestPacket(target_id(), angle, speed);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_pid_parameter()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_READ_PID_PARAMTER, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::write_pid_parameter_to_ram(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki)
{
  LKM_DEBUG_FUNC
  auto packet = WritePIDParameterToRAMRequestPacket(target_id(), angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::write_pid_parameter_to_rom(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki)
{
  LKM_DEBUG_FUNC
  auto packet = WritePIDParameterToROMRequestPacket(target_id(), angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_acceleration()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_READ_ACCELERATION, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::write_acceleration_to_ram(float acc)
{
  LKM_DEBUG_FUNC
  auto packet = WriteAccerelationToRAMRequestPacket(target_id(), acc);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_encoder()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_READ_ENCODER, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::write_encoder_value_to_rom_as_zero_point(uint16_t encoder_offset)
{
  LKM_DEBUG_FUNC
  auto packet = WriteEncoderValueToROMAsZeroPointRequestPacket(target_id(), encoder_offset);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::write_current_position_to_rom_as_zero_point()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_WRITE_CURRENT_POSITION_TO_RON_AS_THE_MOTOR_ZERO_POINT, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_multi_angle_loop()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_READ_MULTI_ANGLE_LOOP, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_single_angle_loop()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_READ_SINGLE_ANGLE_LOOP, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::clear_motor_angle_loop()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_CLEAR_MOTOR_ANGLE_LOOP, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_motor_state1()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_READ_MOTOR_STATE_1_AND_ERROR_STATE, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::clear_motor_error_state()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_CLEAR_MOTOR_ERROR_STATE, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_motor_state2()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_READ_MOTOR_STATE_2, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_motor_state3()
{
  LKM_DEBUG_FUNC
  auto packet = RequestPacket(CMD_READ_MOTOR_STATE_3, target_id(), 0);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::send_motor_request(const Frame& packet)
{
  LKM_DEBUG_FUNC
  stream_->write(packet.data(), packet.size());

  for (uint8_t idx = 0; idx < packet.size(); ++idx) {
    Serial.printf("%x ", packet[idx]);
  }
  Serial.println("");

  ++send_count_;
  return true;
}

bool Driver::process_packet()
{
  LKM_DEBUG_FUNC

  uint16_t cnt = 0;
  while (stream_->available())
  {
    receive_buffer_[cnt] = stream_->read();
    ++cnt;
  }

  if (cnt < CMD_FRAME_LENGTH) return false;

  // move to vector
  Frame receive_data(receive_buffer_, receive_buffer_ + cnt);
  process_response_packet(receive_data);

  return true;
}

bool Driver::process_response_packet(const Frame& frame)
{
  LKM_DEBUG_FUNC

  // check header
  if (frame[CMD_FRAME_HEADER_BYTE] != CMD_FRAME_HEADER) {
    return false;
  }

  uint8_t cmd = frame[CMD_FRAME_COMMAND_BYTE];
  if (cmd == CMD_OPEN_LOOP_CONTROL || cmd == CMD_TORQUE_CLOSED_LOOP_CONTROL ||
      cmd == CMD_SPEED_CLOSED_LOOP_CONTROL || cmd == CMD_MULTI_LOOP_ANGLE_CONTROL_1 ||
      cmd == CMD_MULTI_LOOP_ANGLE_CONTROL_2 || cmd == CMD_SINGLE_LOOP_ANGLE_CONTROL_1 ||
      cmd == CMD_SINGLE_LOOP_ANGLE_CONTROL_2 || cmd == CMD_INCREMENT_ANGLE_CONTROL_1 ||
      cmd == CMD_INCREMENT_ANGLE_CONTROL_2)
  {
    MotorStateResponsePacket packet(cmd, frame, motor_type(), encoder_type());
    if (packet.unpack()) {
      motor_state_.effort = packet.torque_current();
      motor_state_.position = packet.position();
      motor_state_.velocity = packet.speed();
      motor_state_.tempareture = packet.tempareture();
    }

  } else if (cmd == CMD_MOTOR_OFF || cmd == CMD_MOTOR_ON || cmd == CMD_MOTOR_STOP || cmd == CMD_CLEAR_MOTOR_ANGLE_LOOP) {
    // DO NOTHING

  } else if (cmd == CMD_READ_PID_PARAMTER || cmd == CMD_WRITE_PID_PARAMTER_TO_RAM || cmd == CMD_WRITE_PID_PARAMTER_TO_ROM) {
    PIDResponsePacket packet(cmd, frame);
    if (packet.unpack()) {
      motor_parameter_.angle_kp = packet.angle_kp();
      motor_parameter_.angle_ki = packet.angle_ki();
      motor_parameter_.speed_kp = packet.speed_kp();
      motor_parameter_.speed_ki = packet.speed_ki();
      motor_parameter_.iq_kp = packet.iq_kp();
      motor_parameter_.iq_ki = packet.iq_ki();
    }

  } else if (cmd == CMD_READ_ENCODER) {
    ReadEncoderResponsePacket packet(frame, encoder_type());
    if (packet.unpack()) {
      encoder_state_.position = packet.encoder_position();
      encoder_state_.raw_position = packet.encoder_raw_position();
      encoder_state_.offset = packet.encoder_offset();
    }

  } else if (cmd == CMD_READ_ACCELERATION || cmd == CMD_WRITE_ACCELERATION ||
            cmd == CMD_WRITE_ENCODER_VALUE_TO_ROM_AS_THE_MOTOR_ZERO_POINT || cmd == CMD_WRITE_CURRENT_POSITION_TO_RON_AS_THE_MOTOR_ZERO_POINT ||
            cmd == CMD_READ_MULTI_ANGLE_LOOP || cmd == CMD_READ_SINGLE_ANGLE_LOOP || cmd == CMD_READ_MOTOR_STATE_1_AND_ERROR_STATE ||
            cmd == CMD_CLEAR_MOTOR_ERROR_STATE || cmd == CMD_READ_MOTOR_STATE_2 || cmd == CMD_READ_MOTOR_STATE_3) {
    // NOT IMPLEMENTED
  }

  return true;
}

}
