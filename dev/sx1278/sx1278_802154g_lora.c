/*
 * Copyright (c) 2017, Lanven Jiaxing China
 * http://www.scadai.com
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

#include "sx1278-rf-cfg.h"
#include "sx1278-const.h"

/*
 * This is a setup for the following configuration:
 *
 * sx1278 at 1.2 kbps, 2-FSK, 12.5 kHz Channel Spacing (868 MHz).
 */

#define RF_CFG_FREQUENCY  870000000         // RFFrequency
#define RF_CFG_POWER      20                // Power
#define RF_CFG_SINGLE_BW  9                 // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
// 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
#define RF_CFG_SPREADING_FACTOR     7        // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
#define RF_CFG_ERROR_CODING         2        // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define RF_CFG_CRC_ON               1        // CrcOn [0: OFF, 1: ON]
#define RF_CFG_IMPLICIT_HEADER_ON   0        // ImplicitHeaderOn [0: OFF, 1: ON]
#define RF_CFG_RX_SINGLE_ON         1        // RxSingleOn [0: Continuous, 1 Single]
#define RF_CFG_FREQ_HOP_ON          0        // FreqHopOn [0: OFF, 1: ON]
#define RF_CFG_HOP_PERIOD           4        // HopPeriod Hops every frequency hopping period symbols
#define RF_CFG_TX_PACKET_TIMEOUT    100      // TxPacketTimeout
#define RF_CFG_RX_PACKET_TIMEOUT    100      // RxPacketTimeout
#define RF_CFG_PAYLOAD_LENGTH       128      // PayloadLength (used for implicit header mode)

/* Base frequency in kHz */
#define RF_CFG_CHAN_CENTER_F0           867787
/* Channel spacing in Hz */
#define RF_CFG_CHAN_SPACING             12500
/* The minimum channel */
#define RF_CFG_MIN_CHANNEL              0
/* The maximum channel */
#define RF_CFG_MAX_CHANNEL              33
/* The maximum output power in dBm */
#define RF_CFG_MAX_TXPOWER              SX1278_CONST_TX_POWER_MAX
/* The carrier sense level used for CCA in dBm */
#define RF_CFG_CCA_THRESHOLD            (-91)
/* The RSSI offset in dBm */
#define RF_CFG_RSSI_OFFSET              (-99)
/*---------------------------------------------------------------------------*/
static const char rf_cfg_descriptor[] = "sx1278 LoRa 868MHz";
/*---------------------------------------------------------------------------*/

/*
 * Defualt Register settings
 */

// Modulation format = 2-FSK
// Whitening = false
// Symbol rate = 1.2
// Deviation = 3.986359
// Carrier frequency = 867.999878
// Manchester enable = false
// Bit rate = 1.2
// RX filter BW = 10.964912

static const registerSetting_t preferredSettings[] =
{

  {SX1278_FREQ0,                0x06},
  {SX1278_FREQ1,                0xD1},
  {SX1278_FREQ2,                0x00},
  {SX1278_BAND_WIDTH,           0x5D},
  {SX1278_SPREADING_FACTOR,     0x8A},
  {SX1278_ERROR_CODING,         0xCB},
  {SX1278_PREAMBLE_LEN,         0xA6},
  {SX1278_LOW_DATARATE_OPTIMIZE, 0x40},
  {SX1278_FIX_LEN,              0x05},
  {SX1278_CRC_ON,               0x3F},
  {SX1278_RX_CONTINUOUS,        0x75},
  {SX1278_TX_TIMEOUT,           0x10},

};
/*---------------------------------------------------------------------------*/
/* Global linkage: symbol name must be different in each exported file! */
const sx1278_rf_cfg_t sx1278_868_fsk_1_2kbps = {
  .cfg_descriptor            = rf_cfg_descriptor,
  .register_settings         = preferredSettings,
  .size_of_register_settings = sizeof(preferredSettings),
  .rf_frequency              =  RF_CFG_FREQUENCY,
  .power                     = RF_CFG_POWER,
  .spreading_factor          = RF_CFG_CHAN_CENTER_F0,
  .signal_bw                 = RF_CFG_SINGLE_BW,
  .error_coding              = RF_CFG_ERROR_CODING,
  .crc_on                    = RF_CFG_CRC_ON,
  .implicit_header_on        = RF_CFG_IMPLICIT_HEADER_ON,
  .rx_single_on              = RF_CFG_RX_SINGLE_ON,
  .freq_hop_on               = RF_CFG_FREQ_HOP_ON,
  .hop_period                = RF_CFG_HOP_PERIOD,
  .tx_packet_timeout         = RF_CFG_TX_PACKET_TIMEOUT,
  .rx_packet_timeout         = RF_CFG_RX_PACKET_TIMEOUT,
  .payload_length            = RF_CFG_PAYLOAD_LENGTH,
};
/*---------------------------------------------------------------------------*/
