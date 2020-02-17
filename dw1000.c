/*
 * dw1000.c
 *
 *  Created on: 11 Feb 2020
 *      Author: refa
 */

#include <string.h>
#include "dw1000.h"

void dw1000_SetBit(uint8_t *data, uint8_t bitnum, uint8_t value) {
  if(value) {
    *(data+(bitnum / 8)) |= (1 << bitnum);
  }
  else
  {
    *(data+(bitnum / 8)) &= ~(1 << bitnum);
  }
};

dw1000_dev_id_t dw1000_GetDevID(dw1000_HandleTypeDef *dw1000) {
  dw1000_dev_id_t dev_id;
  uint8_t buffer[DW1000_DEV_ID_LEN];

  dw1000_ReadData(dw1000, DW1000_DEV_ID, buffer, DW1000_DEV_ID_LEN);

  dev_id.rev = buffer[0] & 0x0F;
  dev_id.ver = buffer[0] >> 4;
  dev_id.model = buffer[1];
  dev_id.ridtag = (buffer[3] << 8) | buffer[2];

  return dev_id;
}

dw1000_pan_addr_t dw1000_GetPanAddress(dw1000_HandleTypeDef *dw1000) {
  dw1000_pan_addr_t pan_addr;
  uint8_t buffer[DW1000_PANADR_LEN];

  dw1000_ReadData(dw1000, DW1000_PANADR, buffer, DW1000_PANADR_LEN);

  pan_addr.short_addr = (buffer[1] << 8) | buffer[0];
  pan_addr.pan_id = (buffer[3] << 8) | buffer[2];

  return pan_addr;
}

void dw1000_SetPanAddress(dw1000_HandleTypeDef *dw1000, dw1000_pan_addr_t *pan_addr) {
  uint8_t buffer[] = {
      pan_addr->short_addr & 0x00FF,
      pan_addr->short_addr >> 8,
      pan_addr->pan_id & 0x00FF,
      pan_addr->pan_id >> 8,
  };

  dw1000_WriteData(dw1000, DW1000_PANADR, buffer, DW1000_PANADR_LEN);
}

dw1000_channel_t dw1000_GetChannel(dw1000_HandleTypeDef *dw1000) {
  dw1000_channel_t channel;
  uint8_t buffer[DW1000_CHAN_CTRL_LEN];

  dw1000_ReadData(dw1000, DW1000_CHAN_CTRL, buffer, DW1000_CHAN_CTRL_LEN);

  channel.tx_chan = buffer[0] & 0x0F;
  channel.rx_chan = buffer[0] >> 4;
  channel.prf = (buffer[2] >> 2) & 0x03;
  channel.tx_preamble = ((buffer[3] & 0x07) << 2) | (buffer[2] >> 6);
  channel.rx_preamble = buffer[3] >> 3;

  return channel;
}

void dw1000_SetChannel(dw1000_HandleTypeDef *dw1000, dw1000_channel_t *channel) {
  // Read channel control register first because the data isn't complete
  uint8_t buffer[DW1000_CHAN_CTRL_LEN];
  dw1000_ReadData(dw1000, DW1000_CHAN_CTRL, buffer, DW1000_CHAN_CTRL_LEN);
  
  buffer[0] = (channel->rx_chan << 4) | channel->tx_chan;
  buffer[2] = (buffer[2] & 0xF3) | (channel->prf << 2);
  buffer[2] = (buffer[2] & 0x3F) | ((channel->tx_preamble & 0x03) << 6);
  buffer[3] = (channel->rx_preamble << 3) | (channel->tx_preamble >> 2);

  dw1000_WriteData(dw1000, DW1000_CHAN_CTRL, buffer, DW1000_CHAN_CTRL_LEN);
}

dw1000_timestamp_t dw1000_GetSystemTimeCounter(dw1000_HandleTypeDef *dw1000) {
  uint8_t buffer[DW1000_SYS_TIME_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_TIME, buffer, DW1000_SYS_TIME_LEN);
  return (buffer[4] << 32) | (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
}

uint32_t dw1000_GetReceiverTimeTrackingInterval(dw1000_HandleTypeDef *dw1000) {
  uint8_t buffer[DW1000_RX_TTCKI_LEN];
  dw1000_ReadData(dw1000, DW1000_RX_TTCKI, buffer, DW1000_RX_TTCKI_LEN);
  return (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
}

void dw1000_ClearAllStatus(dw1000_HandleTypeDef *dw1000) {
  uint8_t sys_status[DW1000_SYS_STATUS_LEN];
  memset(sys_status, 0xFF, DW1000_SYS_STATUS_LEN);
  dw1000_WriteData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
}

void dw1000_IdleMode(dw1000_HandleTypeDef *dw1000) {
  uint8_t sys_ctrl[DW1000_SYS_CTRL_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
  dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_TRXOFF, 1);
  dw1000_WriteData(dw1000, DW1000_SYS_CTRL, sys_ctrl, 4);
}

void dw1000_StartTransmit(dw1000_HandleTypeDef *dw1000, uint16_t length, uint8_t use_crc) {
  // Write transmit frame control register
  uint8_t fctrl[DW1000_TX_FCTRL_LEN];
  dw1000_ReadData(dw1000, DW1000_TX_FCTRL, fctrl, DW1000_TX_FCTRL_LEN);
  fctrl[0] = length & 0x00FF;
  fctrl[1] &= ~(0x03);
  fctrl[1] |= ((length >> 8) & 0x03);
  dw1000_WriteData(dw1000, DW1000_TX_FCTRL, fctrl, DW1000_TX_FCTRL_LEN);

  // Start transmit
  uint8_t sys_ctrl[DW1000_SYS_CTRL_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
  dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_SFCST, use_crc);
  dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_TXSTRT, 1);
  dw1000_WriteData(dw1000, DW1000_SYS_CTRL, sys_ctrl, 4);
}

void dw1000_ClearTransmitStatus(dw1000_HandleTypeDef *dw1000) {
  uint8_t sys_status[DW1000_SYS_STATUS_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_TXFRB, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_TXPRS, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_TXPHS, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_TXFRS, 1);
  dw1000_WriteData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
}

void dw1000_SetDataToTransmit(dw1000_HandleTypeDef *dw1000, uint8_t *data, uint16_t length, uint8_t use_crc) {
  if(use_crc) {
    length += 2;
  }
  // TODO: Extended frame length?
  if(length > DW1000_TX_BUFFER_LEN) {
    return;  // Error handling
  }
  dw1000_WriteData(dw1000, DW1000_TX_BUFFER, data, length);
  // txfctrl in arduino-dwm1000?
}

void dw1000_StartReceive(dw1000_HandleTypeDef *dw1000, uint8_t use_crc) {
  uint8_t sys_ctrl[DW1000_SYS_CTRL_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
  dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_SFCST, use_crc);
  dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_RXENAB, 1);
  dw1000_WriteData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
}

void dw1000_ClearReceiveStatus(dw1000_HandleTypeDef *dw1000) {
  uint8_t sys_status[DW1000_SYS_STATUS_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_RXDFR, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_LDEDONE, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_LDEERR, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_RXPHE, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_RXFCE, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_RXFCG, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_RXRFSL, 1);
  dw1000_WriteData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
}
