//
// Created by jinjie on 24/01/18.
//

#include "esc_telem.h"

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

void ESCReader::update(spinal::ESCTelemetry& esc_msg)
{
  /* An overrun makes the HAL restart the DMA at index 0 while the read pointer
     stays where it was, which desynchronises the stream for good. Reset both. */
  if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_NEF | UART_CLEAR_OREF | UART_FLAG_RXNE | UART_FLAG_ORE);
    HAL_UART_AbortReceive(huart_);
    HAL_UART_Receive_DMA(huart_, esc_telem_rx_buf_, ESC_BUFFER_SIZE);
    esc_telem_rd_ptr_ = 0;
    esc_msg.crc_error = 0xFF;  // no valid frame this round
    return;
  }

  /* Consume nothing until the whole frame has landed. The frames carry no
     delimiter, so bytes invented here to fill a half-received frame would push
     the real ones into the next read and shift every frame that follows. */
  if (bytesAvailable() < ESC_TELEM_FRAME_SIZE)
  {
    esc_msg.crc_error = 0xFF;  // no valid frame this round
    return;
  }

  // Byte 0: Temperature
  // Byte 1: Voltage high byte
  // Byte 2: Voltage low byte
  // Byte 3: Current high byte
  // Byte 4: Current low byte
  // Byte 5: Consumption high byte
  // Byte 6: Consumption low byte
  // Byte 7: Rpm high byte
  // Byte 8: Rpm low byte
  // Byte 9: 8-bit CRC

  uint8_t buffer[ESC_TELEM_FRAME_SIZE];  // buffer for KISS esc telemetry data

  for (int i = 0; i < ESC_TELEM_FRAME_SIZE; i++)
  {
    buffer[i] = esc_telem_rx_buf_[esc_telem_rd_ptr_];
    esc_telem_rd_ptr_ = (esc_telem_rd_ptr_ + 1) % ESC_BUFFER_SIZE;
  }

  /* check crc */
  uint8_t crc = get_crc8(buffer, 9);
  esc_msg.crc_error = crc - buffer[9];

  if (crc != buffer[9])
    return;  // corrupt frame: keep the previous values, crc_error flags them as stale

  /* save data in esc_msg_1_ */
  esc_msg.temperature = buffer[0];
  esc_msg.voltage = buffer[1] << 8 | buffer[2];
  esc_msg.current = buffer[3] << 8 | buffer[4];
  esc_msg.consumption = buffer[5] << 8 | buffer[6];
  uint16_t erpm = buffer[7] << 8 | buffer[8];
  int pole_pairs = num_motor_mag_pole_ / 2;  // erpm -> rpm
  if (pole_pairs > 0)
    esc_msg.rpm = erpm * 100 / pole_pairs;
  else
    esc_msg.rpm = 0;
}

/* number of bytes the DMA has landed but we have not consumed yet */
uint32_t ESCReader::bytesAvailable()
{
  uint32_t dma_write_ptr = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (ESC_BUFFER_SIZE);
  return (dma_write_ptr + ESC_BUFFER_SIZE - esc_telem_rd_ptr_) % (ESC_BUFFER_SIZE);
}

/* Re-align the read pointer to the DMA write pointer, discarding whatever was
 * not consumed. Call this at every telemetry slot boundary: the four ESCs share
 * one line and the frames carry no delimiter, so a leftover byte would shift
 * every following frame forever. */
void ESCReader::flush()
{
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
