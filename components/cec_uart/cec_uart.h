#pragma once

#include <vector>
#include <queue>
#include "esphome/core/hal.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
 
namespace esphome {
namespace cec_uart_ns {

enum CecStd : uint8_t {
  MESSAGE_MAX_BLOCKS: 16,
  MESSAGE_BLOCK_BITS: 10,
  MAX_POSEDGES:       1 + MESSAGE_MAX_BLOCKS * MESSAGE_BLOCK_BITS
}

class Message: public std::vector<uint8_t> {
  public:
  bool is_broadcast() const;
  uint8_t expected_edges() const;
}

class CecUart : public Component, public uart::UartDevice {
  public:
  bool send_message(const Message& msg);
  bool flush();
  bool is_busy();
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;

  protected:
  void issue_message_from_queue();
  void send_start_bit();
  void send_byte(uint8_t data, bool is_address, bool is_eom);
  bool check_initiator_collision(uint32_t start_time, uint8_t initiator_adress) const;
  void posedge_isr()
  void loop() override;
   void gpio_posedge(CecUart *self);

  bool collision_{false};
  bool acks_ok_{true};
  uint8_t posedge_count_{0};
  uint8_t message_send_attempts_{0};  // 0 means uart is idle, 1 means first send attempt busy on uart, 2 is resend attempt, ...
  uint32_t allow_next_message_us_{0};
  ISRInternalGPIOPin isr_pin_;
  std::array<uint32_t, MAX_POSEDGES> posedge_timestamps_;
  std::queue<Message> message_queue_;
  Mutex send_mutex_;
}

}  // namespace cec_uart_ns
}  // namespace esphome
