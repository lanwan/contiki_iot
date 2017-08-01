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

#ifndef SX1278_H_
#define SX1278_H_

#include "contiki.h"


/*---------------------------------------------------------------------------*/
/*
 * The maximum payload length the driver can handle.
 *
 * - If SX1278_MAX_PAYLOAD_LEN <= 64 we read out the RX FIFO at the end of 
 * the packet. RXOFF_MODE is set to RX in this case.
 *
 * See below for 802.15.4g support.
 */
#ifdef SX1278_CONF_MAX_PAYLOAD_LEN
#define SX1278_MAX_PAYLOAD_LEN           SX1278_CONF_MAX_PAYLOAD_LEN
#else
#define SX1278_MAX_PAYLOAD_LEN           256
#endif
/*---------------------------------------------------------------------------*/
/*
 * Use 802.15.4g frame format? Supports frame lenghts up to 2047 bytes!
 */
#ifdef SX1278_CONF_802154G
#define SX1278_802154G                  SX1278_CONF_802154G
#else
#define SX1278_802154G                  0
#endif
/*---------------------------------------------------------------------------*/
/*
 * Do we use withening in 802.15.4g mode? Set to 1 if enabled, 0 otherwise.
 */
#ifdef SX1278_CONF_802154G_WHITENING
#define SX1278_802154G_WHITENING        SX1278_CONF_802154G_WHITENING
#else
#define SX1278_802154G_WHITENING        0
#endif
/*---------------------------------------------------------------------------*/
/*
 * Do we use CRC16 in 802.15.4g mode? Set to 1 if enabled, 0 otherwise.
 *
 * It set to 0, we use FCS type 0: CRC32.
 */
#ifdef SX1278_CONF_802154G_CRC16
#define SX1278_802154G_CRC16            SX1278_CONF_802154G_CRC16
#else
/* Use FCS type 0: CRC32 */
#define SX1278_802154G_CRC16            0
#endif
/*---------------------------------------------------------------------------*/
/* The RF configuration to be used. */
#ifdef SX1278_CONF_RF_CFG
#define SX1278_RF_CFG                   SX1278_CONF_RF_CFG
#else
#define SX1278_RF_CFG                   sx1278_802154g_lora
#endif
/*---------------------------------------------------------------------------*/
/*
 * The frequency offset
 *
 * Might be hardware dependent (e.g. depending on crystal load capacitances),
 * so we make it a configuration parameter. Written to FREQOFF1 / FREQOFF2.
 * Signed 16 bit number, see cc1200 user's guide.
 *
 * TODO: Make it a parameter for set_value() / get_value()
 */
#ifdef SX1278_CONF_FREQ_OFFSET
#define SX1278_FREQ_OFFSET              SX1278_CONF_FREQ_OFFSET
#else
#define SX1278_FREQ_OFFSET              (0)
#endif
/*---------------------------------------------------------------------------*/
/*
 * The default channel to use. 
 *
 * Permitted values depending on the data rate + band used are defined
 * in the appropriate rf configuration file. Make sure the default value
 * is within these limits!
 * 
 */
#ifdef SX1278_CONF_DEFAULT_CHANNEL
#define SX1278_DEFAULT_CHANNEL          SX1278_CONF_DEFAULT_CHANNEL
#else
/* 880 MHz */
#define SX1278_DEFAULT_CHANNEL          4
#endif
/*---------------------------------------------------------------------------*/
/*
 * Wether to use auto calibration or not.
 *
 * If set to 0, calibration is performed manually when turning the radio
 * on (on()), when transmitting (transmit()) or when changing the channel.
 * Enabling auto calibration will increase turn around times +
 * energy consumption. If enabled, we calibrate every time we go from
 * IDLE to RX or TX.
 * When RDC or channel hopping is used, there is no need to turn calibration
 * on because either on() is called frequently or the channel is updated.
 */
#ifdef SX1278_CONF_AUTOCAL
#define SX1278_AUTOCAL                  SX1278_CONF_AUTOCAL
#else
#define SX1278_AUTOCAL                  0
#endif
/*---------------------------------------------------------------------------*/
/*
 * If SX1278_AUTOCAL is not set, we use this parameter to defer 
 * calibration until a certain amount of time has expired. 
 *
 * This is what happens in detail:
 *
 * - We (manually) calibrate once after initialization
 * - We (manually) calibrate every time we change the channel
 * - We (manually) calibrate when the radio is turned on() only if
 *   the timeout has expired
 * - We (manually) calibrate when transmitting only of the timeout has expired
 * 
 * Set this parameter to 0 when this feature is not used. In this case we 
 * (manually) calibrate in all situations mentioned above.
 */
#ifdef SX1278_CONF_CAL_TIMEOUT_SECONDS
#define SX1278_CAL_TIMEOUT_SECONDS      SX1278_CONF_CAL_TIMEOUT_SECONDS
#else
/* Calibrate at the latest every 15 minutes */
#define SX1278_CAL_TIMEOUT_SECONDS      900
#endif
/*---------------------------------------------------------------------------*/
/*
 * If defined, use these LEDS to indicate TX activity
 *
 * The LEDs are turned on once the radio enters TX mode
 * (together with ENERGEST_ON(ENERGEST_TYPE_TRANSMIT),
 * and turned of as soon as TX has completed.
 */
#ifdef SX1278_CONF_TX_LEDS
#define SX1278_TX_LEDS                  SX1278_CONF_TX_LEDS
#endif
/*---------------------------------------------------------------------------*/
/*
 * If defined, use these LEDS to indicate RX activity
 *
 * The LEDs are turned on as soon as the first byte is read out from
 * the RX FIFO
 */
#ifdef SX1278_CONF_RX_LED
#define SX1278_RX_LEDS                  SX1278_CONF_RX_LEDS
#endif
/*---------------------------------------------------------------------------*/

#endif /* SX1278_H_ */
