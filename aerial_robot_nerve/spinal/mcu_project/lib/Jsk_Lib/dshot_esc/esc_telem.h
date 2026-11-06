//
// Created by jinjie on 24/01/18.
//

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef ESC_TELEM_H
#define ESC_TELEM_H

#include "util/ring_buffer.h"
#include "math/AP_Math.h"
#include "config.h"
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <spinal/ESCTelemetry.h>
#include <spinal/ESCTelemetryArray.h>

#define ESC_BUFFER_SIZE 512
#define ESC_TELEM_FRAME_SIZE 10  // KISS telemetry frame: 9 data bytes + crc
#define ESC_TELEM_FRAME_SIZE_RPM_RATE 12  // KISS extended telemetry frame: 9 data bytes + 2 bytes eRPM rate (AM32 fork, branch measure_rate) + crc
#define ESC_TELEM_ERPM_PER_COUNT 10  // eRPM per count of the eRPM field
#define ESC_TELEM_NO_FRAME 0xFF  // Value of crc_error when a slot had no valid frame (values held from the last one)
#define ESC_TELEM_WAIT_US 200  // Longest wait for the last byte and the idle character after it (86.8 us each at 115200 baud)
#define ESC_TELEM_MAX_VOLTAGE 6000  // Sanity boundary [1=10mV]
#define ESC_TELEM_MAX_RPM 50000  // Sanity boundary [RPM]

namespace
{
#ifdef STM32H7
  uint8_t esc_telem_rx_buf_[ESC_BUFFER_SIZE] __attribute__((section(".EscRxBufferSection")));
#else
  uint8_t esc_telem_rx_buf_[ESC_BUFFER_SIZE];
#endif
  uint32_t esc_telem_rd_ptr_ = 0;
}

class ESCReader
{
public:
  ESCReader(){};
  ~ESCReader(){};

  UART_HandleTypeDef *huart_;

  spinal::ESCTelemetry esc_msg_1_;
  spinal::ESCTelemetry esc_msg_2_;
  spinal::ESCTelemetry esc_msg_3_;
  spinal::ESCTelemetry esc_msg_4_;

  void init(UART_HandleTypeDef* huart);
  bool update(spinal::ESCTelemetry& esc_msg);
  uint32_t bytesAvailable();
  void flush();

  bool is_update_all_msg_ = false;

  int num_motor_mag_pole_; // conversion from erpm to rpm

private:
  bool deferred_ = false;  // Current reply already got one more slot, so the next one is deferred to the next round

  uint8_t step_ = 0;
  uint8_t msg_id_ = 0;
  uint16_t payload_length_ = 0;
  uint16_t payload_counter_ = 0;
  uint8_t ck_a_ = 0;
  uint8_t ck_b_ = 0;
  uint8_t class_ = 0;
};

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);

#endif  // ESC_TELEM_H
