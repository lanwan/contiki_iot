/*
 * Copyright (c) 2015, Weptech elektronik GmbH Germany
 * http://www.weptech.de
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

#include "si4460-rf-cfg.h"
#include "si4460-const.h"

/*
 * This is a setup for the following configuration:
 *
 * si4460 at 1.2 kbps, 2-FSK, 12.5 kHz Channel Spacing (868 MHz).
 */

/* Base frequency in kHz */
#define RF_CFG_CHAN_CENTER_F0           867787
/* Channel spacing in Hz */
#define RF_CFG_CHAN_SPACING             12500
/* The minimum channel */
#define RF_CFG_MIN_CHANNEL              0
/* The maximum channel */
#define RF_CFG_MAX_CHANNEL              33
/* The maximum output power in dBm */
#define RF_CFG_MAX_TXPOWER              SI4460_CONST_TX_POWER_MAX
/* The carrier sense level used for CCA in dBm */
#define RF_CFG_CCA_THRESHOLD            (-91)
/* The RSSI offset in dBm */
#define RF_CFG_RSSI_OFFSET              (-99)
/*---------------------------------------------------------------------------*/
static const char rf_cfg_descriptor[] = "868MHz 2-FSK 1.2 kbps";
/*---------------------------------------------------------------------------*/
/* 
 * Register settings exported from SmartRF Studio using the standard template
 * "trxEB RF Settings Performance Line".
 */

// Modulation format = 2-FSK
// Whitening = false
// Symbol rate = 1.2
// Deviation = 3.986359
// Carrier frequency = 867.999878
// Manchester enable = false
// Bit rate = 1.2
// RX filter BW = 10.964912

static const registerSetting_t preferredSettings[]=
{
  {SI4460_IOCFG2,            0x06},
  {SI4460_DEVIATION_M,       0xD1},
  {SI4460_MODCFG_DEV_E,      0x00},
  {SI4460_DCFILT_CFG,        0x5D},
  {SI4460_PREAMBLE_CFG0,     0x8A},
  {SI4460_IQIC,              0xCB},
  {SI4460_CHAN_BW,           0xA6},
  {SI4460_MDMCFG1,           0x40},
  {SI4460_MDMCFG0,           0x05},
  {SI4460_SYMBOL_RATE2,      0x3F},
  {SI4460_SYMBOL_RATE1,      0x75},
  {SI4460_SYMBOL_RATE0,      0x10},
  {SI4460_AGC_REF,           0x20},
  {SI4460_AGC_CS_THR,        0xEC},
  {SI4460_AGC_CFG1,          0x51},
  {SI4460_AGC_CFG0,          0x87},
  {SI4460_FIFO_CFG,          0x00},
  {SI4460_FS_CFG,            0x12},
  {SI4460_PKT_CFG2,          0x00},
  {SI4460_PKT_CFG0,          0x20},
  {SI4460_PKT_LEN,           0xFF},
  {SI4460_IF_MIX_CFG,        0x1C},
  {SI4460_FREQOFF_CFG,       0x22},
  {SI4460_MDMCFG2,           0x0C},
  {SI4460_FREQ2,             0x56},
  {SI4460_FREQ1,             0xCC},
  {SI4460_FREQ0,             0xCC},
  {SI4460_IF_ADC1,           0xEE},
  {SI4460_IF_ADC0,           0x10},
  {SI4460_FS_DIG1,           0x07},
  {SI4460_FS_DIG0,           0xAF},
  {SI4460_FS_CAL1,           0x40},
  {SI4460_FS_CAL0,           0x0E},
  {SI4460_FS_DIVTWO,         0x03},
  {SI4460_FS_DSM0,           0x33},
  {SI4460_FS_DVC0,           0x17},
  {SI4460_FS_PFD,            0x00},
  {SI4460_FS_PRE,            0x6E},
  {SI4460_FS_REG_DIV_CML,    0x1C},
  {SI4460_FS_SPARE,          0xAC},
  {SI4460_FS_VCO0,           0xB5},
  {SI4460_XOSC5,             0x0E},
  {SI4460_XOSC1,             0x03},
};
/*---------------------------------------------------------------------------*/
/* Global linkage: symbol name must be different in each exported file! */
const si4460_rf_cfg_t si4460_868_fsk_1_2kbps = {
  .cfg_descriptor = rf_cfg_descriptor,
  .register_settings = preferredSettings,
  .size_of_register_settings = sizeof(preferredSettings),
  .tx_pkt_lifetime = (2 * RTIMER_SECOND),
  .tx_rx_turnaround = (RTIMER_SECOND / 2),
  .chan_center_freq0 = RF_CFG_CHAN_CENTER_F0,
  .chan_spacing = RF_CFG_CHAN_SPACING,
  .min_channel = RF_CFG_MIN_CHANNEL,
  .max_channel = RF_CFG_MAX_CHANNEL,
  .max_txpower = RF_CFG_MAX_TXPOWER,
  .cca_threshold = RF_CFG_CCA_THRESHOLD,
  .rssi_offset = RF_CFG_RSSI_OFFSET,
};
/*---------------------------------------------------------------------------*/
