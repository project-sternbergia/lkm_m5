#include "lkm_can_interface_twai.hh"

#include "ESP32-TWAI-CAN.hpp"

using namespace lkm_m5::can;

CanInterfaceTwai::CanInterfaceTwai() : CanInterface() {}

CanInterfaceTwai::~CanInterfaceTwai() {}

bool CanInterfaceTwai::init(uint8_t rx_pin, uint8_t tx_pin)
{
  return ESP32Can.begin(ESP32Can.convertSpeed(1000), tx_pin, rx_pin, 10, 10);
}

size_t CanInterfaceTwai::write(uint64_t id, const Frame & data, bool is_ext)
{
  return write(id, data.data(), data.size(), is_ext);
}

size_t CanInterfaceTwai::write(uint64_t id, const uint8_t * buffer, size_t size, bool is_ext)
{
  CanFrame frame = {0};
  frame.identifier = id;
  frame.extd = (is_ext) ? 1 : 0;
  frame.data_length_code = size;
  memcpy(frame.data, buffer, size);
  bool ret = ESP32Can.writeFrame(frame);
  return (ret) ? size : 0;
}

size_t CanInterfaceTwai::read(CanPacket & packet)
{
  CanFrame frame = {0};
  if (ESP32Can.readFrame(frame, 1) == false) {
    return 0;
  }

  packet.stamp = micros();
  packet.rx_id = frame.identifier;
  packet.tx_id = frame.identifier;
  packet.is_extended = frame.extd;
  packet.is_rtr = frame.rtr;
  packet.dlc = frame.data_length_code;
  packet.data.resize(frame.data_length_code);
  for (uint8_t idx = 0; idx < frame.data_length_code; ++idx) {
    packet.data[idx] = frame.data[idx];
  }

  return frame.data_length_code;
}

size_t CanInterfaceTwai::available() { return ESP32Can.inRxQueue(); }
