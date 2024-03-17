#ifndef LKM_RS485_DRIVER_HH
#define LKM_RS485_DRIVER_HH

#include <cstdint>
#include <sys/types.h>
#include "lkm_driver_common.hh"
#include "lkm_driver.hh"

namespace lkm_m5::rs485
{
  class Driver : public lkm_m5::Driver
  {
  public:
    Driver(uint8_t master_id, uint8_t target_id, uint8_t motor_type, uint8_t encoder_type);
    virtual ~Driver();
    void init(Stream* p_stream);
    virtual bool motor_on();
    virtual bool motor_off();
    virtual bool stop_motor();
    virtual bool open_loop_control(int16_t power);
    virtual bool torque_closed_loop_control(float iq_control);
    virtual bool speed_closed_loop_control(float speed);
    virtual bool multi_loop_angle_control(float angle);
    virtual bool multi_loop_angle_with_speed_control(float angle, float speed);
    virtual bool single_loop_angle_control(uint8_t spin_dir, float angle);
    virtual bool single_loop_angle_with_speed_control(uint8_t spin_dir, float angle, float speed);
    virtual bool increment_angle_control(float angle);
    virtual bool increment_angle_with_speed_control(float angle, float speed);
    virtual bool read_pid_parameter();
    virtual bool write_pid_parameter_to_ram(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki);
    virtual bool write_pid_parameter_to_rom(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki);
    virtual bool read_acceleration();
    virtual bool write_acceleration_to_ram(float acc);
    virtual bool read_encoder();
    virtual bool write_encoder_value_to_rom_as_zero_point(uint16_t encoder_offset);
    virtual bool write_current_position_to_rom_as_zero_point();
    virtual bool read_multi_angle_loop();
    virtual bool read_single_angle_loop();
    virtual bool clear_motor_angle_loop();
    virtual bool read_motor_state1();
    virtual bool clear_motor_error_state();
    virtual bool read_motor_state2();
    virtual bool read_motor_state3();
    virtual bool process_packet();
    virtual bool process_response_packet(const Frame& frame);

  protected:
    bool send_motor_request(const Frame& packet);

  private:
    Stream* stream_;              //!< can connection instance
    uint8_t receive_buffer_[64];  //!< receive buffer
    unsigned long send_count_;
  };
};

#endif // !LKM_RS485_DRIVER_HH
