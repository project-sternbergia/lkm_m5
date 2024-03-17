#include "lkm_rs485_packet.hh"
#include "lkm_driver_common.hh"
#include "lkm_rs485_packet_defs.hh"

#include <M5Stack.h>
#undef min
#undef max

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <sys/select.h>

namespace lkm_m5::rs485
{

Packet::Packet(uint8_t type, uint8_t id, uint8_t data_length)
  : header_(CMD_FRAME_HEADER), type_(type), id_(id), frame_(), data_length_(data_length)
{
  if (data_length > 0) {
    frame_.resize(CMD_FRAME_LENGTH + data_length + 1, 0);
  }
  else {
    frame_.resize(CMD_FRAME_LENGTH, 0);
  }
}

Packet::Packet(uint8_t type, uint8_t data_length, const Frame& frame)
  : type_(type), frame_(frame), id_(0xFF), header_(CMD_FRAME_HEADER), data_length_(data_length)
{}

Packet::~Packet()
{}

uint8_t Packet::calc_checksum(Frame::const_iterator start, Frame::const_iterator end)
{
  uint8_t chk = 0x00;
  for (Frame::const_iterator it = start; it < end; ++it)
  {
    chk += (*it & 0xFF);
  }
  return chk;
}

Frame::iterator Packet::data_frame_head()
{
  if (data_length() > 0) {
    return frame_.begin() + CMD_FRAME_LENGTH;
  }
  return frame_.end();
}

RequestPacket::RequestPacket(uint8_t type, uint8_t id, uint8_t data_len) : Packet(type, id, data_len)
{}

RequestPacket::~RequestPacket()
{}

bool RequestPacket::pack()
{
  frame_[CMD_FRAME_HEADER_BYTE] = header();
  frame_[CMD_FRAME_COMMAND_BYTE] = type();
  frame_[CMD_FRAME_ID_BYTE] = id();
  frame_[CMD_FRAME_DATA_LENGTH_BYTE] = data_length();
  frame_[CMD_FRAME_CHECK_BYTE] = calc_checksum(frame_.begin(), frame_.begin() + CMD_FRAME_LENGTH - 1);
  if (data_length() > 0) {
    frame_.back() = calc_checksum(frame_.begin() + CMD_FRAME_LENGTH, frame_.begin() + CMD_FRAME_LENGTH + data_length() - 1);
  }
  return true;
}

ResponsePacket::ResponsePacket(uint8_t type, uint8_t data_length, const Frame & frame)
  : Packet(type, data_length, frame)
{}

ResponsePacket::~ResponsePacket()
{}

bool ResponsePacket::unpack()
{
  if (frame_[CMD_FRAME_HEADER_BYTE] != header()){
    LKM_DEBUG_PRINTF("Invalid header type. except=[%x] actual=[%x]\n", header(), frame_[CMD_FRAME_HEADER_BYTE]);
    return false;
  }

  if (frame_[CMD_FRAME_COMMAND_BYTE] != type()) {
    LKM_DEBUG_PRINTF("Invalid command type. except=[%x] actual=[%x]\n", type(), frame_[CMD_FRAME_COMMAND_BYTE]);
    return false;
  }

  if (frame_[CMD_FRAME_DATA_LENGTH_BYTE] != data_length()) {
    LKM_DEBUG_PRINTF("Invalid data length. except=[%x] actual=[%x]\n", data_length(), frame_[CMD_FRAME_DATA_LENGTH_BYTE]);
    return false;
  }

  // uint8_t cmd_chk = calc_checksum(frame_.begin(), frame_.begin() + CMD_FRAME_LENGTH - 1);
  // if (cmd_chk != frame_[CMD_FRAME_CHECK_BYTE]) {
  //   LKM_DEBUG_PRINTF("Invalid cmd frame checksum. except=[%x] actual=[%x]\n", cmd_chk, frame_[CMD_FRAME_CHECK_BYTE]);
  //   return false;
  // }

  // check data frame length
  if (data_length() > 0) {
    // check frame size
    if (frame_.size() != CMD_FRAME_LENGTH + data_length() + 1) {
      LKM_DEBUG_PRINTF("Invalid received data length. except=[%d] actual=[%u]\n", CMD_FRAME_LENGTH + data_length() + 1, frame_.size());
      return false;
    }

    // check data frame parity
    uint8_t data_chk = calc_checksum(frame_.begin() + CMD_FRAME_LENGTH, frame_.begin() + CMD_FRAME_LENGTH + data_length());
    if (data_chk != frame_.back()) {
      LKM_DEBUG_PRINTF("Invalid data frame checksum. except=[%x] actual=[%x]\n", data_chk, frame_.back());
      return false;
    }
  }

  return true;
}

MotorStateResponsePacket::MotorStateResponsePacket(uint8_t type, const Frame& frame, uint8_t motor_type, uint8_t encoder_type)
  : ResponsePacket(type, 7, frame)
  , motor_type_(motor_type)
  , encoder_type_(encoder_type)
  , tempareture_(0.0f)
  , torque_current_(0.0f)
  , speed_(0.0f)
  , position_(0.0f)
{}

MotorStateResponsePacket::~MotorStateResponsePacket()
{}

bool MotorStateResponsePacket::unpack()
{
  if (!ResponsePacket::unpack()) return false;
  tempareture_ = frame_[0];

  // set torque current
  int16_t current = 0.0f;
  std::memcpy(&current, frame_.data() + 1, sizeof(current));
  if (motor_type_ == MOTOR_SERIES_MS) {
    torque_current_ = current;
  } else if (motor_type_ == MOTOR_SERIES_MF) {
    torque_current_ = MF_SERIES_MAX_TORQUE * current / 2048.0f;
  } else if (motor_type_ == MOTOR_SERIES_MG) {
    torque_current_ = MG_SERIES_MAX_TORQUE * current / 2048.0f;
  }

  // set speed
  int16_t speed_dps = 0;
  std::memcpy(&speed_dps, frame_.data() + 3, sizeof(speed_dps));
  speed_ = speed_dps / 180.0f * M_PI;

  // set position
  uint16_t position_step = 0;
  std::memcpy(&position_step, frame_.data() + 5, sizeof(position_step));

  if (encoder_type_ == ENCODER_TYPE_14_BIT) {
    position_ = position_step / 16383.0f * 2.0f * M_PI;
  } else if (encoder_type_ == ENCODER_TYPE_16_BIT) {
    position_ = position_step / 32767.0f * 2.0f * M_PI;
  } else if (encoder_type_ == ENCODER_TYPE_18_BIT) {
    position_ = position_step / 65535.0f * 2.0f * M_PI;
  } else {
    // error
    position_ = 0.0f;
  }

  return true;
}

PIDResponsePacket::PIDResponsePacket(uint8_t type, const Frame& frame)
  : ResponsePacket(type, 0, frame)
{}

PIDResponsePacket::~PIDResponsePacket() {}

bool PIDResponsePacket::unpack() {
  if (!ResponsePacket::unpack()) return false;
  angle_kp_ = frame_[2];
  angle_ki_ = frame_[3];
  speed_kp_ = frame_[4];
  speed_ki_ = frame_[5];
  iq_kp_ = frame_[6];
  iq_ki_ = frame_[7];
  return true;
}

ReadEncoderResponsePacket::ReadEncoderResponsePacket(const Frame& frame, uint8_t encoder_type)
  : ResponsePacket(CMD_READ_ENCODER, 0, frame)
  , encoder_type_(encoder_type)
{}

ReadEncoderResponsePacket::~ReadEncoderResponsePacket() {}

bool ReadEncoderResponsePacket::unpack() {
  if (!ResponsePacket::unpack()) return false;

  uint16_t position_step;
  std::memcpy(&position_step, frame_.data() + 2, sizeof(position_step));

  uint16_t raw_position_step;
  std::memcpy(&raw_position_step, frame_.data() + 4, sizeof(raw_position_step));

  uint16_t offset;
  std::memcpy(&offset, frame_.data() + 6, sizeof(offset));

  if (encoder_type_ == ENCODER_TYPE_14_BIT) {
    encoder_position_ = position_step / 16383.0f * 2.0f * M_PI;
    encoder_raw_position_ = raw_position_step / 16383.0f * 2.0f * M_PI;
    encoder_offset_ = offset / 16383.0f * 2.0f * M_PI;
  } else if (encoder_type_ == ENCODER_TYPE_16_BIT) {
    encoder_position_ = position_step / 32767.0f * 2.0f * M_PI;
    encoder_raw_position_ = raw_position_step / 32767.0f * 2.0f * M_PI;
    encoder_offset_ = offset / 32767.0f * 2.0f * M_PI;
  } else if (encoder_type_ == ENCODER_TYPE_18_BIT) {
    encoder_position_ = position_step / 65535.0f * 2.0f * M_PI;
    encoder_raw_position_ = raw_position_step / 65535.0f * 2.0f * M_PI;
    encoder_offset_ = offset / 65535.0f * 2.0f * M_PI;
  } else {
    // error
    encoder_position_ = 0.0f;
    encoder_raw_position_ = 0.0f;
    encoder_offset_ = 0.0f;
  }
  return true;
}


OpenLoopControlRequestPacket::OpenLoopControlRequestPacket(uint8_t id, int16_t power_control)
  : RequestPacket(CMD_OPEN_LOOP_CONTROL, id, 2)
{
  power_control_ = std::max(std::min(power_control, static_cast<int16_t>(MS_SERIES_MAX_POWER)), static_cast<int16_t>(-MS_SERIES_MAX_POWER));
}

bool OpenLoopControlRequestPacket::pack() {
  if (!RequestPacket::pack()) return false;
  std::memcpy(frame_.data() + 4, &power_control_, sizeof(power_control_));
  return true;
}


TorqueClosedLoopControlRequestPacket::TorqueClosedLoopControlRequestPacket(uint8_t id, float iq_control, uint8_t motor_type)
  : RequestPacket(CMD_TORQUE_CLOSED_LOOP_CONTROL, id, 2)
  , motor_type_(motor_type)
{
  if (motor_type == MOTOR_SERIES_MF) {
    iq_control_ = std::max(std::min(iq_control, MF_SERIES_MAX_TORQUE), -MF_SERIES_MAX_TORQUE);

  } else if (motor_type == MOTOR_SERIES_MG) {
    iq_control_ = std::max(std::min(iq_control, MG_SERIES_MAX_TORQUE), -MG_SERIES_MAX_TORQUE);

  } else {
    // unknown motor type
    iq_control_ = 0.0f;
  }
}

bool TorqueClosedLoopControlRequestPacket::pack() {
  int16_t control = 0;
  if (motor_type_ == MOTOR_SERIES_MF) {
    control = static_cast<int16_t>(iq_control_ / MF_SERIES_MAX_TORQUE * 2048);

  } else if (motor_type_ == MOTOR_SERIES_MG) {
    control = static_cast<int16_t>(iq_control_ / MG_SERIES_MAX_TORQUE * 2048);
  }
  std::memcpy(frame_.data() + CMD_FRAME_LENGTH, &control, sizeof(control));
  return RequestPacket::pack();
}


SpeedClosedLoopControlRequestPacket::SpeedClosedLoopControlRequestPacket(uint8_t id, float speed)
  : RequestPacket(CMD_SPEED_CLOSED_LOOP_CONTROL, id, 8)
  , speed_(speed)
{}

bool SpeedClosedLoopControlRequestPacket::pack() {
  if (!RequestPacket::pack()) return false;
  int32_t speed_dps = static_cast<int32_t>(speed_ * 180.0f / M_PI * 100.0f);
  std::memcpy(frame_.data() + 4, &speed_dps, sizeof(int32_t));
  return true;
}


MultiLoopAngleControlRequestPacket::MultiLoopAngleControlRequestPacket(uint8_t id, float angle)
  : RequestPacket(CMD_MULTI_LOOP_ANGLE_CONTROL_1, id, 8)
  , angle_(angle)
{}

bool MultiLoopAngleControlRequestPacket::pack() {
  if (!RequestPacket::pack()) return false;
  int32_t angle_deg = static_cast<int32_t>(angle_ * 180.0f / M_PI * 100.0);
  std::memcpy(frame_.data() + 4, &angle_deg, sizeof(int32_t));
  return true;
}


MultiLoopAngleControlWithSpeedRequestPacket::MultiLoopAngleControlWithSpeedRequestPacket(uint8_t id, float angle, float speed)
  : RequestPacket(CMD_MULTI_LOOP_ANGLE_CONTROL_2, id, 8)
  , angle_(angle)
  , speed_(speed)
{}

bool MultiLoopAngleControlWithSpeedRequestPacket::pack() {
  if (!RequestPacket::pack()) return false;
  uint16_t speed_dps = static_cast<uint16_t>(std::abs(speed_dps) * 180.0f / M_PI * 100.0);
  std::memcpy(frame_.data() + 2, &speed_dps, sizeof(speed_dps));

  int32_t angle_deg = static_cast<int32_t>(angle_ * 180.0f / M_PI * 100.0);
  std::memcpy(frame_.data() + 4, &angle_deg, sizeof(int32_t));
  return true;
}


SingleLoopAngleControlRequestPacket::SingleLoopAngleControlRequestPacket(uint8_t id, uint8_t spin_dir, float angle)
  : RequestPacket(CMD_SINGLE_LOOP_ANGLE_CONTROL_1, id, 8)
  , spin_dir_(spin_dir)
  , angle_(angle)
{}

bool SingleLoopAngleControlRequestPacket::pack() {
  if (!RequestPacket::pack()) return false;
  frame_[1] = spin_dir_;
  uint32_t angle_deg = static_cast<uint32_t>(angle_ * 180.0f / M_PI * 100.0);
  std::memcpy(frame_.data() + 4, &angle_deg, sizeof(uint32_t));
  return true;
}


SingleLoopAngleControlWithSpeedRequestPacket::SingleLoopAngleControlWithSpeedRequestPacket(uint8_t id, uint8_t spin_dir, float angle, float speed)
  : RequestPacket(CMD_SINGLE_LOOP_ANGLE_CONTROL_2, id, 8)
  , spin_dir_(spin_dir)
  , angle_(angle)
  , speed_(speed)
{}

bool SingleLoopAngleControlWithSpeedRequestPacket::pack() {
  if (!RequestPacket::pack()) return false;
  frame_[1] = spin_dir_;
  uint16_t speed_dps = static_cast<uint16_t>(speed_ * 180.0f / M_PI);
  std::memcpy(frame_.data() + 2, &speed_dps, sizeof(uint16_t));

  uint32_t angle_deg = static_cast<uint32_t>(angle_ * 180.0f / M_PI * 100.0f);
  std::memcpy(frame_.data() + 4, &angle_deg, sizeof(uint32_t));
  return true;
}

IncrementAngleControlRequestPacket::IncrementAngleControlRequestPacket(uint8_t id, float angle)
  : RequestPacket(CMD_INCREMENT_ANGLE_CONTROL_1, id, 8)
  , angle_(angle)
{}

bool IncrementAngleControlRequestPacket::pack() {
  if (!RequestPacket::pack()) return false;
  int32_t angle_deg = static_cast<int32_t>(angle_ * 180.0f / M_PI * 100.0f);
  std::memcpy(frame_.data() + 4, &angle_deg, sizeof(int32_t));
  return true;
}


IncrementAngleControlWithSpeedRequestPacket::IncrementAngleControlWithSpeedRequestPacket(uint8_t id, float angle, float speed)
  : RequestPacket(CMD_INCREMENT_ANGLE_CONTROL_2, id, 8)
  , angle_(angle)
  , speed_(speed)
{}

bool IncrementAngleControlWithSpeedRequestPacket::pack() {
  if (!RequestPacket::pack()) return false;
  uint16_t speed_dps = static_cast<uint16_t>(speed_ * 180.0f / M_PI);
  std::memcpy(frame_.data() + 2, &speed_dps, sizeof(uint16_t));

  int32_t angle_deg = static_cast<int32_t>(angle_ * 180.0f / M_PI * 100.0f);
  std::memcpy(frame_.data() + 4, &angle_deg, sizeof(int32_t));
  return true;
}

WritePIDParameterToRAMRequestPacket::WritePIDParameterToRAMRequestPacket(uint8_t id, uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki)
  : RequestPacket(CMD_WRITE_PID_PARAMTER_TO_RAM, id, 8)
  , angle_kp_(angle_kp), angle_ki_(angle_ki), speed_kp_(speed_kp), speed_ki_(speed_ki), iq_kp_(iq_kp), iq_ki_(iq_ki)
{}

bool WritePIDParameterToRAMRequestPacket::pack() {
  if (!RequestPacket::pack()) return false;
  frame_[2] = angle_kp_;
  frame_[3] = angle_ki_;
  frame_[4] = speed_kp_;
  frame_[5] = speed_ki_;
  frame_[6] = iq_kp_;
  frame_[7] = iq_ki_;
  return true;
}


WritePIDParameterToROMRequestPacket::WritePIDParameterToROMRequestPacket(uint8_t id, uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki)
  : RequestPacket(CMD_WRITE_PID_PARAMTER_TO_ROM, id, 8)
  , angle_kp_(angle_kp), angle_ki_(angle_ki), speed_kp_(speed_kp), speed_ki_(speed_ki), iq_kp_(iq_kp), iq_ki_(iq_ki)
{}

bool WritePIDParameterToROMRequestPacket::pack() {
  if (!RequestPacket::pack()) return false;
  frame_[2] = angle_kp_;
  frame_[3] = angle_ki_;
  frame_[4] = speed_kp_;
  frame_[5] = speed_ki_;
  frame_[6] = iq_kp_;
  frame_[7] = iq_ki_;
  return true;
}

WriteAccerelationToRAMRequestPacket::WriteAccerelationToRAMRequestPacket(uint8_t id, float acc)
  : RequestPacket(CMD_WRITE_ACCELERATION, id, 8)
  , accerelation_(acc)
{}


WriteEncoderValueToROMAsZeroPointRequestPacket::WriteEncoderValueToROMAsZeroPointRequestPacket(uint8_t id, uint16_t encoder_offset)
  : RequestPacket(CMD_WRITE_ACCELERATION, id, 8)
  , encoder_offset_(encoder_offset)
{}

bool WriteEncoderValueToROMAsZeroPointRequestPacket::pack() {
  if (!RequestPacket::pack()) return false;
  std::memcpy(frame_.data() + 6, &encoder_offset_, sizeof(encoder_offset_));
  return true;
}

}; // llkm_m5::can
