#ifndef LKM_CAN_PACKET_HH
#define LKM_CAN_PACKET_HH

#include "lkm_driver_common.hh"
#include "lkm_driver_defs.hh"

#include <M5Stack.h>
#undef min
#undef max

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <sys/select.h>

namespace lkm_m5::can {

  class Packet
  {
  public:
    Packet(uint8_t type, uint8_t len);
    Packet(uint8_t type, const Frame& frame);
    virtual ~Packet();
    uint8_t type() const { return type_; }
    const Frame & frame() const {return frame_;}

  protected:
    Frame frame_;

  private:
    uint8_t type_;
  };

  class RequestPacket : public Packet
  {
  public:
    RequestPacket(uint8_t type, uint8_t len);
    virtual ~RequestPacket();
    virtual bool pack();
  };

  class ResponsePacket : public Packet
  {
  public:
    ResponsePacket(uint8_t type, const Frame & frame);
    virtual ~ResponsePacket();
    virtual bool unpack();
  };

  class MotorStateResponsePacket : public ResponsePacket
  {
  public:
    MotorStateResponsePacket(uint8_t type, const Frame& frame, uint8_t motor_type, uint8_t encoder_type);
    virtual ~MotorStateResponsePacket();
    virtual bool unpack();

    float tempareture() const { return tempareture_; }
    float torque_current() const { return torque_current_; }
    float speed() const { return speed_; }
    float position() const { return position_; }

  private:
    uint8_t motor_type_;
    uint8_t encoder_type_;
    float tempareture_;
    float torque_current_;
    float speed_;
    float position_;
  };

  class PIDResponsePacket : public ResponsePacket
  {
  public:
    PIDResponsePacket(uint8_t type, const Frame& frame);
    virtual ~PIDResponsePacket();
    virtual bool unpack();

    uint8_t angle_kp() const { return angle_kp_; }
    uint8_t angle_ki() const { return angle_ki_; }
    uint8_t speed_kp() const { return speed_kp_; }
    uint8_t speed_ki() const { return speed_ki_; }
    uint8_t iq_kp() const { return iq_kp_; }
    uint8_t iq_ki() const { return iq_ki_; }

  private:
    uint8_t angle_kp_;
    uint8_t angle_ki_;
    uint8_t speed_kp_;
    uint8_t speed_ki_;
    uint8_t iq_kp_;
    uint8_t iq_ki_;
  };

  class ReadEncoderResponsePacket: public ResponsePacket
  {
  public:
    ReadEncoderResponsePacket(const Frame& frame, uint8_t encoder_type);
    virtual ~ReadEncoderResponsePacket();
    virtual bool unpack();
    float encoder_position() const { return encoder_position_; }
    float encoder_raw_position() const { return encoder_raw_position_; }
    float encoder_offset() const { return encoder_offset_; }

  private:
    uint8_t encoder_type_;
    float encoder_position_;
    float encoder_raw_position_;
    float encoder_offset_;
  };

  class OpenLoopControlRequestPacket : public RequestPacket
  {
  public:
    explicit OpenLoopControlRequestPacket(int16_t power_control);
    int16_t power_control() const { return power_control_; };
    virtual bool pack();

  private:
    int16_t power_control_;
  };

  class TorqueClosedLoopControlRequestPacket : public RequestPacket
  {
  public:
    TorqueClosedLoopControlRequestPacket(float iq_control, uint8_t motor_type);
    virtual bool pack();

  private:
    float iq_control_;
    uint8_t motor_type_;
  };

  class SpeedClosedLoopControlRequestPacket: public RequestPacket
  {
  public:
    explicit SpeedClosedLoopControlRequestPacket(float speed);
    virtual bool pack();

  private:
    float speed_;
  };

  class MultiLoopAngleControlRequestPacket: public RequestPacket
  {
  public:
    explicit MultiLoopAngleControlRequestPacket(float angle);
    virtual bool pack();

  private:
    float angle_;
  };

  class MultiLoopAngleControlWithSpeedRequestPacket: public RequestPacket
  {
  public:
    MultiLoopAngleControlWithSpeedRequestPacket(float angle, float speed);
    virtual bool pack();

  private:
    float angle_;
    float speed_;
  };

  class SingleLoopAngleControlRequestPacket: public RequestPacket
  {
  public:
    SingleLoopAngleControlRequestPacket(uint8_t spin_dir, float angle);
    virtual bool pack();

  private:
    uint8_t spin_dir_;
    float angle_;
  };

  class SingleLoopAngleControlWithSpeedRequestPacket: public RequestPacket
  {
  public:
    SingleLoopAngleControlWithSpeedRequestPacket(uint8_t spin_dir, float angle, float speed);
    virtual bool pack();

  private:
    uint8_t spin_dir_;
    float angle_;
    float speed_;
  };

  class IncrementAngleControlRequestPacket: public RequestPacket
  {
  public:
    IncrementAngleControlRequestPacket(float angle);
    virtual bool pack();

  private:
    float angle_;
  };

  class IncrementAngleControlWithSpeedRequestPacket: public RequestPacket
  {
  public:
    IncrementAngleControlWithSpeedRequestPacket(float angle, float speed);
    virtual bool pack();

  private:
    float angle_;
    float speed_;
  };

  class WritePIDParameterToRAMRequestPacket: public RequestPacket
  {
  public:
    WritePIDParameterToRAMRequestPacket(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki);
    virtual bool pack();

  private:
    uint8_t angle_kp_;
    uint8_t angle_ki_;
    uint8_t speed_kp_;
    uint8_t speed_ki_;
    uint8_t iq_kp_;
    uint8_t iq_ki_;
  };

  class WritePIDParameterToROMRequestPacket: public RequestPacket
  {
  public:
    WritePIDParameterToROMRequestPacket(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki);
    virtual bool pack();

  private:
    uint8_t angle_kp_;
    uint8_t angle_ki_;
    uint8_t speed_kp_;
    uint8_t speed_ki_;
    uint8_t iq_kp_;
    uint8_t iq_ki_;
  };

  class WriteAccerelationToRAMRequestPacket: public RequestPacket
  {
  public:
    WriteAccerelationToRAMRequestPacket(float acc);

  private:
    float accerelation_;
  };

  class WriteEncoderValueToROMAsZeroPointRequestPacket: public RequestPacket
  {
  public:
    WriteEncoderValueToROMAsZeroPointRequestPacket(uint16_t encoder_offset);
    virtual bool pack();

  private:
    uint16_t encoder_offset_;
  };
}

#endif // !LKM_PACKET_HH
