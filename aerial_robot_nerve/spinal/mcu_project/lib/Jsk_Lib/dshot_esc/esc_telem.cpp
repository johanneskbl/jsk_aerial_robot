//
// Created by jinjie on 24/01/18.
//

#include "esc_telem.h"

/* Core clock cycles since the SysTick sample `start`. SysTick is the FreeRTOS tick timer: it counts down from LOAD
   at the core clock and wraps every 1 ms, so this only holds for intervals shorter than that */
static uint32_t cycles_since(uint32_t start)
{
  uint32_t now = SysTick->VAL;
  return (start >= now) ? start - now : start + SysTick->LOAD + 1 - now;
}

void ESCReader::init(UART_HandleTypeDef* huart)
{
  huart_ = huart;

  // use DMA for UART RX
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

  memset(esc_telem_rx_buf_, 0, ESC_BUFFER_SIZE);
  esc_telem_rd_ptr_ = 0;

  HAL_UART_Receive_DMA(huart, esc_telem_rx_buf_, ESC_BUFFER_SIZE);
}

bool ESCReader::update(spinal::ESCTelemetry& esc_msg)
{
  /* Returns true if update is finished and false if read is still expected to be ongoing */
  /* An overrun makes the HAL restart the DMA at index 0 while the read pointer
     stays where it was, which desynchronises the stream for good. Reset both. */
  if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_NEF | UART_CLEAR_OREF | UART_FLAG_RXNE | UART_FLAG_ORE);
    HAL_UART_AbortReceive(huart_);
    HAL_UART_Receive_DMA(huart_, esc_telem_rx_buf_, ESC_BUFFER_SIZE);
    esc_telem_rd_ptr_ = 0;
    esc_msg.crc_error = ESC_TELEM_NO_FRAME;  // no valid frame this round
    deferred_ = false;  // The restart dropped any reply that was still pending
    return true;
  }

  /* Consume nothing until the whole frame has landed. The frames carry no delimiter, so every slot must hold the
     reply of exactly one ESC (flush() at each slot boundary), defined by the number of expected bytes.
     Issue: a reply still arriving at the end of its slot could be flushed and its last arriving byte then
     shifts the next ESC's frame by one byte.
     Fix: a reply is complete once the line went idle after it. IDLE flag: set one character after the last byte and
     cleared by flush(). A reply missing at most its last byte is waited for (<= 174 us at 115200 baud), a later one
     keeps the line for one more slot: update() returns false, DShot::write() neither flushes nor requests the next
     ESC, and the next call reads the reply in full. */
  uint32_t available = bytesAvailable();
  // Check if the request landed up to its last byte, and if so wait for that byte and the idle character after it
  if (!deferred_ && available >= ESC_TELEM_FRAME_SIZE_RPM_RATE - 1)
  {
    const uint32_t start = SysTick->VAL;
    const uint32_t timeout = ESC_TELEM_WAIT_US * (SystemCoreClock / 1000000U);
    while (!__HAL_UART_GET_FLAG(huart_, UART_FLAG_IDLE) && cycles_since(start) < timeout) {}
  }
  const bool idle = __HAL_UART_GET_FLAG(huart_, UART_FLAG_IDLE);
  available = bytesAvailable();  // Read after the flag: no byte follows once the line is idle
  // Still arriving: keep the line for one more slot
  if (available > 0 && !idle && !deferred_)
  {
    deferred_ = true;
    return false;
  }
  deferred_ = false;  // On the second try, the number of bytes alone decides

  int frame_size;
  if (available == ESC_TELEM_FRAME_SIZE_RPM_RATE || available == ESC_TELEM_FRAME_SIZE)
    frame_size = available;
  else
  {
    esc_msg.crc_error = ESC_TELEM_NO_FRAME;  // No reply, a truncated one, or more bytes than one frame
    return true;
  }

  // Byte 0: Temperature
  // Byte 1: Voltage high byte
  // Byte 2: Voltage low byte
  // Byte 3: Current high byte
  // Byte 4: Current low byte
  // Byte 5: Consumption high byte
  // Byte 6: Consumption low byte
  // Byte 7: eRPM / 10 high byte
  // Byte 8: eRPM / 10 low byte
  // Byte 9: 8-bit CRC (if 10-byte frame is sent) ! OR ! RPM rate high byte (IF extended frame is sent)
  // Byte 10: RPM rate low byte (extended frame)
  // Byte 11: 8-bit CRC (extended frame)

  uint8_t buffer[ESC_TELEM_FRAME_SIZE_RPM_RATE];  // buffer for KISS esc telemetry data

  for (int i = 0; i < frame_size; i++)
  {
    buffer[i] = esc_telem_rx_buf_[esc_telem_rd_ptr_];
    esc_telem_rd_ptr_ = (esc_telem_rd_ptr_ + 1) % ESC_BUFFER_SIZE;
  }

  /* check crc */
  uint8_t crc = get_crc8(buffer, frame_size - 1);
  esc_msg.crc_error = crc - buffer[frame_size - 1];

  if (crc != buffer[frame_size - 1])
    return true;  // Corrupt frame: keep the previous values, crc_error flags them as stale

  // Read data
  uint16_t voltage = buffer[1] << 8 | buffer[2];
  uint16_t erpm_field = buffer[7] << 8 | buffer[8];
  uint32_t erpm = (uint32_t)erpm_field * ESC_TELEM_ERPM_PER_COUNT;  // Decoded eRPM from the telemetry frame
  int pole_pairs = num_motor_mag_pole_ / 2;
  // Rounded to whole rpm: truncating would read 0.5 rpm low on average
  uint32_t rpm = (pole_pairs > 0) ? (erpm + pole_pairs / 2) / pole_pairs : 0;

  /* Sanity check */
  if (voltage > ESC_TELEM_MAX_VOLTAGE || rpm > ESC_TELEM_MAX_RPM)
  {
    esc_msg.crc_error = ESC_TELEM_NO_FRAME;
    return true;
  }

  /* Save data in esc_msg_1_ */
  esc_msg.temperature = buffer[0];
  esc_msg.voltage = voltage;
  esc_msg.current = buffer[3] << 8 | buffer[4];
  esc_msg.consumption = buffer[5] << 8 | buffer[6];
  esc_msg.rpm = rpm;

  /* eRPM rate / 100 (two's complement), estimated on the ESC */
  if (frame_size == ESC_TELEM_FRAME_SIZE_RPM_RATE && pole_pairs > 0)
  {
    // Decode the two's complement value of the eRPM rate from the extended telemetry frame
    // Latency Note: AM32 already averages each period over about one electrical revolution before the
    // filter gets it. That adds 1.1 ms at hover and 12 ms at 785 rpm.
    // Including Spinal's 8 ms polling, the rate reaches Spinal's controller about
    // 18 ms after the rotor at hover.
    int16_t erpm_rate = (int16_t)(buffer[9] << 8 | buffer[10]);
    esc_msg.rpm_rate = (int32_t)erpm_rate * 100 / pole_pairs;
  }
  else
    // The 10-byte frame carries no rate
    esc_msg.rpm_rate = 0;

  return true;
}

/* number of bytes the DMA has landed but we have not consumed yet */
uint32_t ESCReader::bytesAvailable()
{
  uint32_t dma_write_ptr = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (ESC_BUFFER_SIZE);
  return (dma_write_ptr + ESC_BUFFER_SIZE - esc_telem_rd_ptr_) % (ESC_BUFFER_SIZE);
}

/* Re-align the read pointer to the DMA write pointer, discarding whatever was
 * not consumed, and clear IDLE so that it only reports the end of the next reply.
 * Call this at every telemetry slot boundary: the four ESCs share
 * one line and the frames carry no delimiter, so a leftover byte would shift
 * every following frame forever. */
void ESCReader::flush()
{
  __HAL_UART_CLEAR_IDLEFLAG(huart_);
  esc_telem_rd_ptr_ = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (ESC_BUFFER_SIZE);
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}
