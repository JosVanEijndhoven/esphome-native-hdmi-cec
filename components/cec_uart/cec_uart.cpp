#include "cec_uart.h"

namespace esphome {
namespace cec_uart_ns {

bool Message::is_broadcast() const {
  uint8_t address_byte = data[0];
  const static uint8_t destination_mask = 0xf0;
  return (address_byte & destination_mask) == destination_mask;
}

uint8_t Message::expected_edges() const {
  return 1 + MESSAGE_BLOCK_BITS * size();
}

void CecUart::queue_send_message(const Message& msg) {
  LockGuard send_lock(send_mutex_);
  message_queue_.push(msg);
}

bool CecUart::flush() {
  return false;
}

bool CecUart::is_busy() {. 
  return false;
}

void CecUart::send_start_bit() {
  // Keep 0.6ms bit-time for both start-bit and data-words, but send start-bit with
  // '5 data bits' mode in stead of the '6 data bits' mode in data-words.
  // this allows to send a 6*'0' bit and a last '1'-bit, ending after 7*0.6=4.2ms
  // Keep uart setting of 8 datab-bite per uart-frame, but change its baudrate
  // to conform to a 3.7ms low start-bit period:
  // 3.7ms / (9 uart bits) = 0.41111 ms/bit = 2432 bits/sec = 2432 baud.
  const static uint32_t START_BIT_PERIOD_US = 4500;
  const static uint32_t START_LOW_MIN_US    = 3500;
  const static uint32_t START_LOW_NOM_US    = 3700;
  const static uint32_t START_LOW_MAX_US    = 3900;
  const static uint32_t CEC_BITPERIOD_US    = 2400;
  this->parent_->set_baud_rate(2432);  // = 1000000 * 9 / START_LOW_NOM_US
  this->parent->load_settings(false);
  posedge_count_ = 0;
  uint32_t start_time = micros();
  write_byte(0);  // send uart start-bit and 8 more '0' data bits (together 9 * 0.4111 = 3.7msec), then trailing '1' stop-bit.
  flush();
  this->parent_->set_baud_rate(2083);  // = 1000000 * 5 / CEC_BITPERIOD_US
  this->parent->load_settings(false);
  int32_t waittime = START_BIT_PERIOD_US + starttime - micros();  // expected 4500 - 4200 = 300. Should be > 0
  if (waittime > 0) {
    delay_microseconds_safe(waittime);
  }
  // Expected an interrupt on the rising edge.
  // The rising edge could be delayed (or missing) on a bus conflict, where some other initiator also 'start's
  bool ok = (posedge_count_ == 1   // received exactly 1 interrupt
             && (posedge_timestamps_[0] - start_time) >= START_LOW_MIN_US
             && (posedge_timestamps_[0] - start_time) <= START_LOW_MAX_US);
  collision_ = !ok;
  return;
}

void CecUart::send_byte(uint8_t data, bool is_address, bool is_eom) {
  // the cec bit is created by the uart with 0.48ms per uart-bit is 
  // 5 uart-bits create the nominal 2.4ms cec bit period
  // 10 uart-bits are made with an (always-0) uart start bit, then 8 data bit, and an (always 1) uart stop bit.
  // transmitting a '0' data bit gets translated to a uart 3xlow, 2xhigh on the cec line
  // transmitting a '1' data bit gets translated to a uart 1xlow, 4xhigh on the cec line
  static const std::array<uint8_t, 4> dualbit_to_uart_word = {0x8c, 0x8f, 0xec, 0xef};
  std::array<uint8_t, 5> uart_bytes;
  uint16_t cec_frame = data;
  cec_frame |= ((uint16_t)(is_eom & 1) << 8);  // add eom (end-of-message) bit
  cec_frame |= 1u << 9;  // add ack bit, is always 1
  for (int i = 0; i < 5; i++, cec_frame >>= 2) {
    uint8_t twobits = cec_frame & 0x3;
    uart_bytes[i] = dualbit_to_uart_word[twobits];
  }
  uint32_t start_time = micros();
  posedge_count_ = 0;
  if (is_address) {
    // first byte of cec message starts with initiator (is me) address.
    // these 4 bits are written as 2 bytes by the uart.
    // need to check for bus conflict
    const uint32_t initiator_byte_cnt = 2;
    write_array(uart_bytes.data(), initiator_byte_cnt);
    flush();
    collision_ = check_initiator_collision( start_time, data & 0x0f);
    if (collision_) {
      return;  // immediatly abort transmitting, allow a different initiator on the bus to continue
    }
    write_array(uart_bytes.data() + initiator_byte_cnt, uart_bytes.size() - initiator_byte_cnt);  // send 3 remaining bytes
  } else {
    write_array(uart_bytes.data(), uart_bytes.size());
  }
  // do NOT wait here with 'flush' to finish sending the byte: that would occupy this 'loop()' for too long
  return;
}

bool CecUart::check_initiator_collision(uint32_t start_time, uint8_t initiator_adress) const {
  return false;
}

void CecUart::issue_message_from_queue() {
  const Message& msg = message_queue_.front();
  collision_ = false;
  // Sending data shall stop when a bus collision is detected.
  // This collision detect only occurs during the start_bit and the first half of the first byte.
  // The byte acknowledges on the bus (by the target component) are at the end of each transmitted byte.
  // We don't wait for that: the uart queues the bytes to send. The acknowledges are checked later,
  // so that we can return early and not keep the 'loop()' method busy for too long.
  send_start_bit();
  for (int i = 0; !collision_ && i < msg.size(); i++){
    send_byte( msg[i], (i == 0), (i == (msg.size() - 1)));
  }
}

void CecUart::loop() override {
  {
    LockGuard send_lock(send_mutex_);
    if (message_queue_.empty()) {
      // not busy and no new work to do
      return;
    }
  }

  if (allow_next_message_us_ > micros()) {
    // there is work in the queue, but it is too early to issue a message on the bus
    return;
  }

  const Message &msg = message_queue_.front();  // this msg is either new to send, or already busy
  if (message_send_attempts_ > 0 && posedge_count_ < msg.expected_edges() && !collision_) {
    // uart is still busy, wait until it is finished with sending the current message
    return;
  }

  acks_ok_ = false;
  if (message_send_attempts_ > 0 && posedge_count_ == expected_edges) {  // this implies !collision
    // current message got transmitted by uart
    acks_ok_ = check_message_acks(msg.size(), msg.is_broadcast());
  }
  // if '!acks_ok' or 'collision_' the current send attempt was unsuccessful, maybe try next attempt?

  if (message_send_attempts_ >= 5 || acks_ok_) {
    // current message is done, either OK or unsuccessful
    // log termination of the current message...
    uint8_t edge_cnt = collision_ ? 4 : msg.expected_edges();
    uint32_t time_of_last_ack = posedge_timestamps_[edge_cnt];
    uint8_t wait_bit_periods = (collision_ || !acks_ok_) ? 3 : 7;  // 3 or 7 periods idle as specified in CEC standard
    allow_next_message_us_ = time_of_last_ack + wait_bit_periods * CEC_BITPERIOD_US;
    message_send_attempts_ = 0;
    {
      LockGuard send_lock(send_mutex_);
      message_queue_.pop();
      // there might be a next message on the queue, that will be picked up on the next call to 'loop()'.
      // A next message would require some waiting time anyhow.
      return;
    }
  }
  
  // issue message on the queue front onto the cec bus
  message_send_attempts_++;
  issue_message_from_queue();
}

void IRAM_ATTR CecUart::gpio_posedge(CecUart *self) {
  if (posedge_count_ < posedge_timestamps_.size()) {
    posedge_timestamps_[posedge_count_] = micros();
  }
  posedge_count_++;
  isr_pin_.clear_interrupt();
}

void CecUart::setup() override {
  // set pin_mode( flags)
  // tx_pin_->pin_mode( ..)
  isr_pin_ = tx_pin_->to_isr();
  tx_pin_->attach_interrupt(isr, this, gpio::INTERRUPT_RISING_EDGE);
}

}  // namespace cec_uart_ns
}  // namespace esphome
    