#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
#include "lkm_rs485_driver.hh"
#include "lkm_rs485_packet_defs.hh"

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID = 0x01;

// init cybergeardriver
lkm_m5::rs485::Driver driver = lkm_m5::rs485::Driver(MASTER_CAN_ID, MOT_CAN_ID, MOTOR_SERIES_MF, ENCODER_TYPE_16_BIT);
lkm_m5::MotorState motor_status;

void setup()
{
  M5.begin();

  // Serial2.begin(115200, SERIAL_8N1, 16, 15);
  // Serial2.begin(115200, SERIAL_8N1, 15, 16);

  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  driver.init(&Serial2);

  driver.motor_on();
  driver.process_packet();
}

void loop()
{
  M5.update();
  driver.torque_closed_loop_control(0.0f);
  driver.process_packet();

  motor_status = driver.motor_state();
  Serial.printf("%f %f %f %f\n", motor_status.position, motor_status.velocity, motor_status.effort, motor_status.tempareture);
  delay(1000);
}
