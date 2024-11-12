#include "lkm_can_driver.hh"

#include <Arduino.h>

#include "lkm_can_packet.hh"
#include "lkm_driver_defs.hh"

using namespace lkm_m5;

namespace lkm_m5::can
{
Driver::Driver(
  uint8_t host_id, uint8_t target_id, uint8_t motor_type, uint8_t encoder_type,
  uint16_t wait_response_time_usec)
: lkm_m5::Driver(host_id, target_id, motor_type, encoder_type),
  wait_response_time_usec_(wait_response_time_usec)
{
}

Driver::~Driver() {}

void Driver::init(CanInterface * p_can) { can_ = p_can; }

bool Driver::motor_on()
{
  auto packet = RequestPacket(CMD_MOTOR_ON, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::motor_off()
{
  auto packet = RequestPacket(CMD_MOTOR_OFF, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::stop_motor()
{
  auto packet = RequestPacket(CMD_MOTOR_STOP, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::open_loop_control(int16_t power)
{
  auto packet = OpenLoopControlRequestPacket(power);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::torque_closed_loop_control(float iq_control)
{
  auto packet = TorqueClosedLoopControlRequestPacket(iq_control, motor_type());
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::speed_closed_loop_control(float speed)
{
  auto packet = SpeedClosedLoopControlRequestPacket(speed);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::multi_loop_angle_control(float angle)
{
  auto packet = MultiLoopAngleControlRequestPacket(angle);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::multi_loop_angle_with_speed_control(float angle, float speed)
{
  auto packet = MultiLoopAngleControlWithSpeedRequestPacket(angle, speed);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::single_loop_angle_control(uint8_t spin_dir, float angle)
{
  auto packet = SingleLoopAngleControlRequestPacket(spin_dir, angle);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::single_loop_angle_with_speed_control(uint8_t spin_dir, float angle, float speed)
{
  auto packet = SingleLoopAngleControlWithSpeedRequestPacket(spin_dir, angle, speed);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::increment_angle_control(float angle)
{
  auto packet = IncrementAngleControlRequestPacket(angle);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::increment_angle_with_speed_control(float angle, float speed)
{
  auto packet = IncrementAngleControlWithSpeedRequestPacket(angle, speed);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_pid_parameter()
{
  auto packet = RequestPacket(CMD_READ_PID_PARAMTER, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::write_pid_parameter_to_ram(
  uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp,
  uint8_t iq_ki)
{
  auto packet =
    WritePIDParameterToRAMRequestPacket(angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::write_pid_parameter_to_rom(
  uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp,
  uint8_t iq_ki)
{
  auto packet =
    WritePIDParameterToROMRequestPacket(angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_acceleration()
{
  auto packet = RequestPacket(CMD_READ_ACCELERATION, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::write_acceleration_to_ram(float acc)
{
  auto packet = WriteAccerelationToRAMRequestPacket(acc);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_encoder()
{
  auto packet = RequestPacket(CMD_READ_ENCODER, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::write_encoder_value_to_rom_as_zero_point(uint16_t encoder_offset)
{
  auto packet = WriteEncoderValueToROMAsZeroPointRequestPacket(encoder_offset);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::write_current_position_to_rom_as_zero_point()
{
  auto packet = RequestPacket(CMD_WRITE_CURRENT_POSITION_TO_RON_AS_THE_MOTOR_ZERO_POINT, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_multi_angle_loop()
{
  auto packet = RequestPacket(CMD_READ_MULTI_ANGLE_LOOP, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_single_angle_loop()
{
  auto packet = RequestPacket(CMD_READ_SINGLE_ANGLE_LOOP, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::clear_motor_angle_loop()
{
  auto packet = RequestPacket(CMD_CLEAR_MOTOR_ANGLE_LOOP, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_motor_state1()
{
  auto packet = RequestPacket(CMD_READ_MOTOR_STATE_1_AND_ERROR_STATE, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::clear_motor_error_state()
{
  auto packet = RequestPacket(CMD_CLEAR_MOTOR_ERROR_STATE, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_motor_state2()
{
  auto packet = RequestPacket(CMD_READ_MOTOR_STATE_2, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::read_motor_state3()
{
  auto packet = RequestPacket(CMD_READ_MOTOR_STATE_3, 8);
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::send_motor_request(const Frame & packet)
{
  uint32_t id = 0x140 + target_id();
  Frame send_data = packet;
  can_->write(id, send_data.data(), send_data.size(), false);
  delayMicroseconds(wait_response_time_usec_);
  ++send_count_;
  return true;
}

bool Driver::process_packet()
{
  if (!can_->available()) {
    return false;
  }

  // receive data
  unsigned long id;
  uint8_t len;
  CanPacket packet;
  can_->read(packet);
  process_response_packet(packet.data);
  return true;
}

bool Driver::process_response_packet(const Frame & frame)
{
  uint8_t cmd = frame[0];
  if (
    cmd == CMD_OPEN_LOOP_CONTROL || cmd == CMD_TORQUE_CLOSED_LOOP_CONTROL ||
    cmd == CMD_SPEED_CLOSED_LOOP_CONTROL || cmd == CMD_MULTI_LOOP_ANGLE_CONTROL_1 ||
    cmd == CMD_MULTI_LOOP_ANGLE_CONTROL_2 || cmd == CMD_SINGLE_LOOP_ANGLE_CONTROL_1 ||
    cmd == CMD_SINGLE_LOOP_ANGLE_CONTROL_2 || cmd == CMD_INCREMENT_ANGLE_CONTROL_1 ||
    cmd == CMD_INCREMENT_ANGLE_CONTROL_2) {
    MotorStateResponsePacket packet(cmd, frame, motor_type(), encoder_type());
    if (packet.unpack()) {
      motor_state_.stamp = micros();
      motor_state_.effort = packet.torque_current();
      motor_state_.position = packet.position();
      motor_state_.velocity = packet.speed();
      motor_state_.tempareture = packet.tempareture();
    }
  } else if (
    cmd == CMD_MOTOR_OFF || cmd == CMD_MOTOR_ON || cmd == CMD_MOTOR_STOP ||
    cmd == CMD_CLEAR_MOTOR_ANGLE_LOOP) {
    // DO NOTHING
  } else if (
    cmd == CMD_READ_PID_PARAMTER || cmd == CMD_WRITE_PID_PARAMTER_TO_RAM ||
    cmd == CMD_WRITE_PID_PARAMTER_TO_ROM) {
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
  } else if (
    cmd == CMD_READ_ACCELERATION || cmd == CMD_WRITE_ACCELERATION ||
    cmd == CMD_WRITE_ENCODER_VALUE_TO_ROM_AS_THE_MOTOR_ZERO_POINT ||
    cmd == CMD_WRITE_CURRENT_POSITION_TO_RON_AS_THE_MOTOR_ZERO_POINT ||
    cmd == CMD_READ_MULTI_ANGLE_LOOP || cmd == CMD_READ_SINGLE_ANGLE_LOOP ||
    cmd == CMD_READ_MOTOR_STATE_1_AND_ERROR_STATE || cmd == CMD_CLEAR_MOTOR_ERROR_STATE ||
    cmd == CMD_READ_MOTOR_STATE_2 || cmd == CMD_READ_MOTOR_STATE_3) {
    // NOT IMPLEMENTED
  }

  return true;
}

void Driver::print_can_packet(uint32_t id, uint8_t * data, uint8_t len)
{
  Serial.print("Id : ");
  Serial.print(id, HEX);

  Serial.print(" Data : ");
  for (byte i = 0; i < len; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\n");
}

}  // namespace lkm_m5::can
