#include "lkm_driver.hh"
#include "lkm_driver_defs.hh"
#include "lkm_packet.hh"
#include <Arduino.h>

using namespace lkm_m5;

Driver::Driver(uint8_t master_can_id, uint8_t target_can_id, uint8_t motor_type, uint8_t encoder_type)
  : master_can_id_(master_can_id)
  , target_can_id_(target_can_id)
  , motor_type_(motor_type)
  , encoder_type_(encoder_type)
  , send_count_(0)
{}

Driver::~Driver()
{}

void Driver::init(MCP_CAN* p_can)
{
  can_ = p_can;
}

bool Driver::motor_on()
{
  auto packet = MotorOnRequestPacket();
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::motor_off()
{
  auto packet = MotorOffRequestPacket();
  if (!packet.pack()) return false;
  return send_motor_request(packet.frame());
}

bool Driver::stop_motor()
{
  auto packet = MotorStopRequestPacket();
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
  auto packet = TorqueClosedLoopControlRequestPacket(iq_control, motor_type_);
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

bool Driver::increment_angle_control(){ return true; }
bool Driver::increment_angle_with_speed_control(){ return true; }
bool Driver::read_pid_parameter(){ return true; }
bool Driver::write_pid_parameter_to_ram(){ return true; }
bool Driver::write_pid_parameter_to_rom(){ return true; }
bool Driver::read_acceleration(){ return true; }
bool Driver::read_encoder(){ return true; }
bool Driver::write_encoder_value_to_rom_as_zero_point(){ return true; }
bool Driver::write_current_position_to_rom_as_zero_point(){ return true; }
bool Driver::read_multi_angle_loop(){ return true; }
bool Driver::read_single_angle_loop(){ return true; }
bool Driver::clear_motor_angle_loop(){ return true; }
bool Driver::read_motor_state1(){ return true; }
bool Driver::clear_motor_error_state(){ return true; }
bool Driver::read_motor_state2(){ return true; }
bool Driver::read_motor_state3(){ return true; }

bool Driver::send_motor_request(const Frame& packet)
{
  uint32_t id = 0x140 + target_can_id_;
  Frame send_data = packet;
  can_->sendMsgBuf(id, send_data.size(), send_data.data());
  ++send_count_;
  return true;
}

bool Driver::process_can_packet()
{
  if (can_->checkReceive() != CAN_MSGAVAIL) {
    return false;
  }

  // receive data
  unsigned long id;
  uint8_t len;
  can_->readMsgBuf(&id, &len, receive_buffer_);
  // print_can_packet(id, receive_buffer_, len);

  // parse packet --------------
  Frame frame(receive_buffer_, receive_buffer_ + len);
  process_response_packet(frame);
  return true;
}

bool Driver::process_response_packet(const Frame& frame)
{
  uint8_t cmd = frame[0];
  if (cmd == CMD_OPEN_LOOP_CONTROL || cmd == CMD_TORQUE_CLOSED_LOOP_CONTROL ||
      cmd == CMD_SPEED_CLOSED_LOOP_CONTROL || cmd == CMD_MULTI_LOOP_ANGLE_CONTROL_1 ||
      cmd == CMD_MULTI_LOOP_ANGLE_CONTROL_2 || cmd == CMD_SINGLE_LOOP_ANGLE_CONTROL_1 ||
      cmd == CMD_SINGLE_LOOP_ANGLE_CONTROL_2 || cmd == CMD_INCREMENT_ANGLE_CONTROL_1 ||
      cmd == CMD_INCREMENT_ANGLE_CONTROL_2)
  {
    MotorStateResponsePacket packet(cmd, frame, motor_type_, encoder_type_);
    if (packet.unpack()) {
      status_.effort = packet.torque_current();
      status_.position = packet.position();
      status_.velocity = packet.speed();
      status_.tempareture = packet.tempareture();
    }

  } else if (cmd == CMD_MOTOR_OFF || cmd == CMD_MOTOR_ON || cmd == CMD_MOTOR_STOP ||
            cmd == CMD_WRITE_PID_PARAMTER_TO_RAM || cmd == CMD_WRITE_PID_PARAMTER_TO_ROM) {
    // DO NOTHING
  }

  return true;
}

void Driver::print_can_packet(uint32_t id, uint8_t *data, uint8_t len)
{
  Serial.print("Id : ");
  Serial.print(id, HEX);

  Serial.print(" Data : ");
  for(byte i = 0; i<len; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\n");
}

void Driver::print_motor_status(const MotorStatus& status)
{
  Serial.printf("%f %f %f %f\n", status.tempareture, status.position, status.velocity, status.effort);
}
