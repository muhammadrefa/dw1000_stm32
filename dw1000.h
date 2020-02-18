/*
 * dw1000.h
 *
 *  Created on: 11 Feb 2020
 *      Author: refa
 *
 */

#ifndef DW1000_H_
#define DW1000_H_

#include <stdint.h>
#include "main.h"
#include "dw1000_time.h"

// Register map and length
#define DW1000_DEV_ID 0x00
#define DW1000_DEV_ID_LEN 4
#define DW1000_EUI 0x01
#define DW1000_EUI_LEN 8
#define DW1000_PANADR 0x03
#define DW1000_PANADR_LEN 4
#define DW1000_SYS_CFG 0x04
#define DW1000_SYS_CFG_LEN 4
#define DW1000_SYS_TIME 0x06
#define DW1000_SYS_TIME_LEN 5
#define DW1000_TX_FCTRL 0x08
#define DW1000_TX_FCTRL_LEN 5
#define DW1000_TX_BUFFER 0x09
#define DW1000_TX_BUFFER_LEN 1024
#define DW1000_DX_TIME 0x0A
#define DW1000_DX_TIME_LEN 5
#define DW1000_RX_FWTO 0x0C
#define DW1000_RX_FWTO_LEN 2
#define DW1000_SYS_CTRL 0x0D
#define DW1000_SYS_CTRL_LEN 4
#define DW1000_SYS_MASK 0x0E
#define DW1000_SYS_MASK_LEN 4
#define DW1000_SYS_STATUS 0x0F
#define DW1000_SYS_STATUS_LEN 5
#define DW1000_RX_FINFO 0x10
#define DW1000_RX_FINFO_LEN 4
#define DW1000_RX_BUFFER 0x11
#define DW1000_RX_BUFFER_LEN 1024
#define DW1000_RX_FQUAL 0x12
#define DW1000_RX_FQUAL_LEN 8
#define DW1000_RX_TTCKI 0x13
#define DW1000_RX_TTCKI_LEN 4
#define DW1000_RX_TTCKO 0x14
#define DW1000_RX_TTCKO_LEN 5
#define DW1000_RX_TIME 0x15
#define DW1000_RX_TIME_LEN 14
#define DW1000_TX_TIME 0x17
#define DW1000_TX_TIME_LEN 10
#define DW1000_TX_ANTD 0x18
#define DW1000_TX_ANTD_LEN 2
#define DW1000_SYS_STATE 0x19
#define DW1000_SYS_STATE_LEN 5
#define DW1000_ACK_RESP_T 0x1A
#define DW1000_ACK_RESP_T_LEN 4
#define DW1000_RX_SNIFF 0x1D
#define DW1000_RX_SNIFF_LEN 4
#define DW1000_TX_POWER 0x1E
#define DW1000_TX_POWER_LEN 4
#define DW1000_CHAN_CTRL 0x1F
#define DW1000_CHAN_CTRL_LEN 4
#define DW1000_USR_SFD 0x21
#define DW1000_USR_SFD_LEN 41
#define DW1000_AGC_CTRL 0x23
#define DW1000_AGC_CTRL_LEN 33
#define DW1000_EXT_SYNC 0x24
#define DW1000_EXT_SYNC_LEN 12
#define DW1000_ACC_MEM 0x25
#define DW1000_ACC_MEM_LEN 4064
#define DW1000_GPIO_CTRL 0x26
#define DW1000_GPIO_CTRL_LEN 44
#define DW1000_DRX_CONF 0x27
#define DW1000_DRX_CONF_LEN 44
#define DW1000_RF_CONF 0x28
#define DW1000_RF_CONF_LEN 58
#define DW1000_TX_CAL 0x2A
#define DW1000_TX_CAL_LEN 52
#define DW1000_FS_CTRL 0x2B
#define DW1000_FS_CTRL_LEN 21
#define DW1000_AON 0x2C
#define DW1000_AON_LEN 12
#define DW1000_OTP_IF 0x2D
#define DW1000_OTP_IF_LEN 18
#define DW1000_LDE_CTRL 0x2E
#define DW1000_DIG_DIAG 0x2F
#define DW1000_DIG_DIAG_LEN 41
#define DW1000_PSMC 0x36
#define DW1000_PSMC_LEN 48

typedef struct {
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef *ss_port;
  uint16_t ss_pin;
} dw1000_HandleTypeDef;

typedef struct {
  uint8_t rev;
  uint8_t ver;
  uint8_t model;
  uint16_t ridtag;
} dw1000_dev_id_t;

typedef struct {
  uint16_t pan_id;
  uint16_t short_addr;
} dw1000_pan_addr_t;

typedef struct {
  uint8_t tx_chan;
  uint8_t rx_chan;
  dw1000_prf_t prf;
  uint8_t tx_preamble;
  uint8_t rx_preamble
} dw1000_channel_t;

typedef enum {
  DW1000_SYS_CTRL_SFCST = 0,
  DW1000_SYS_CTRL_TXSTRT = 1,
  DW1000_SYS_CTRL_TXDLYS = 2,
  DW1000_SYS_CTRL_CANSFCS = 3,
  DW1000_SYS_CTRL_TRXOFF = 6,
  DW1000_SYS_CTRL_WAIT4RESP = 7,
  DW1000_SYS_CTRL_RXENAB = 8,
  DW1000_SYS_CTRL_RXDLYE = 9,
  DW1000_SYS_CTRL_HRBPT = 24
} dw1000_sys_ctrl_bit_t;

typedef enum {
  DW1000_SYS_STATUS_IRQS = 0,
  DW1000_SYS_STATUS_CPLOCK,
  DW1000_SYS_STATUS_ESYNCR,
  DW1000_SYS_STATUS_AAT,
  DW1000_SYS_STATUS_TXFRB,
  DW1000_SYS_STATUS_TXPRS,
  DW1000_SYS_STATUS_TXPHS,
  DW1000_SYS_STATUS_TXFRS,
  DW1000_SYS_STATUS_RXPRD,	// 8
  DW1000_SYS_STATUS_RXSFDD,
  DW1000_SYS_STATUS_LDEDONE,
  DW1000_SYS_STATUS_RXPHD,
  DW1000_SYS_STATUS_RXPHE,
  DW1000_SYS_STATUS_RXDFR,
  DW1000_SYS_STATUS_RXFCG,
  DW1000_SYS_STATUS_RXFCE,
  DW1000_SYS_STATUS_RXRFSL,	// 16
  DW1000_SYS_STATUS_PXRFTO,
  DW1000_SYS_STATUS_LDEERR,
  DW1000_SYS_STATUS_RXOVRR = 20,
  DW1000_SYS_STATUS_RXPTO,
  DW1000_SYS_STATUS_GPIOIRQ,
  DW1000_SYS_STATUS_SLP2INIT,
  DW1000_SYS_STATUS_RFPLL_LL,	// 24
  DW1000_SYS_STATUS_CLKPLL_LL,
  DW1000_SYS_STATUS_RXSFDTO,
  DW1000_SYS_STATUS_HPDWARN,
  DW1000_SYS_STATUS_TXBERR,
  DW1000_SYS_STATUS_AFFREJ,
  DW1000_SYS_STATUS_HSRBP,
  DW1000_SYS_STATUS_ICRBP,
  DW1000_SYS_STATUS_RXRSCS,	// 32
  DW1000_SYS_STATUS_RXPREJ,
  DW1000_SYS_STATUS_TXPUTE
} dw1000_sys_status_bit_t;

typedef enum {
  DW1000_SYS_CFG_FFEN = 0,
  DW1000_SYS_CFG_FFBC,
  DW1000_SYS_CFG_FFAB,
  DW1000_SYS_CFG_FFAD,
  DW1000_SYS_CFG_FFAA,
  DW1000_SYS_CFG_FFAM,
  DW1000_SYS_CFG_FFAR,
  DW1000_SYS_CFG_FFA4,
  DW1000_SYS_CFG_FFA5,  // 8
  DW1000_SYS_CFG_HIRQ_POL,
  DW1000_SYS_CFG_SPI_EDGE,
  DW1000_SYS_CFG_DIS_FCE,
  DW1000_SYS_CFG_DIS_DRXB,
  DW1000_SYS_CFG_DIS_PHE,
  DW1000_SYS_CFG_DIS_RSDE,
  DW1000_SYS_CFG_INIT2F,
  DW1000_SYS_CFG_PHR_MODE,  // 16
  DW1000_SYS_CFG_DIS_STXP = 18,
  DW1000_SYS_CFG_RXM110K = 22,
  DW1000_SYS_CFG_RXWTOE = 28,
  DW1000_SYS_CFG_RXAUTR,
  DW1000_SYS_CFG_AUTOACK,
  DW1000_SYS_CFG_AACKPEND
} dw1000_sys_cfg_bit_t;

typedef enum {
  DW1000_PRF_16MHZ = 0b01,
  DW1000_PRF_64MHZ = 0b10
} dw1000_prf_t;

void dw1000_SetBit(uint8_t *data, uint8_t bitnum, uint8_t value);

void dw1000_WriteData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint8_t *data, uint8_t len);
void dw1000_ReadData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint8_t *data, uint8_t len);

dw1000_dev_id_t dw1000_GetDevID(dw1000_HandleTypeDef *dw1000);
dw1000_pan_addr_t dw1000_GetPanAddress(dw1000_HandleTypeDef *dw1000);
void dw1000_SetPanAddress(dw1000_HandleTypeDef *dw1000, dw1000_pan_addr_t *pan_addr);
dw1000_channel_t dw1000_GetChannel(dw1000_HandleTypeDef *dw1000);
void dw1000_SetChannel(dw1000_HandleTypeDef *dw1000, dw1000_channel_t *channel);
dw1000_timestamp_t dw1000_GetSystemTimeCounter(dw1000_HandleTypeDef *dw1000);
uint32_t dw1000_GetReceiverTimeTrackingInterval(dw1000_HandleTypeDef *dw1000);
void dw1000_ClearAllStatus(dw1000_HandleTypeDef *dw1000);

void dw1000_IdleMode(dw1000_HandleTypeDef *dw1000);

void dw1000_StartTransmit(dw1000_HandleTypeDef *dw1000, uint16_t length, uint8_t use_crc);
void dw1000_ClearTransmitStatus(dw1000_HandleTypeDef *dw1000);

void dw1000_StartReceive(dw1000_HandleTypeDef *dw1000, uint8_t use_crc);
void dw1000_ClearReceiveStatus(dw1000_HandleTypeDef *dw1000);
void dw1000_ReceiveAutoEnable(dw1000_HandleTypeDef *dw1000, uint8_t val);

void dw1000_SetDataToTransmit(dw1000_HandleTypeDef *dw1000, uint8_t *data, uint16_t length, uint8_t use_crc);
uint16_t dw1000_GetDataReceivedLength(dw1000_HandleTypeDef *dw1000, uint8_t use_crc);
void dw1000_GetDataReceived(dw1000_HandleTypeDef *dw1000, uint8_t* data, uint8_t use_crc);

#endif /* DW1000_H_ */
