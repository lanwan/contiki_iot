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

#ifndef SI4460_CONST_H_
#define SI4460_CONST_H_

/*---------------------------------------------------------------------------*/
/* Register addresses exported from SmartRF Studio */
#define SI4460_IOCFG3                   0x0000 /* GPIO3 IO Pin Configuration */
#define SI4460_IOCFG2                   0x0001 /* GPIO2 IO Pin Configuration */
#define SI4460_IOCFG1                   0x0002 /* GPIO1 IO Pin Configuration */
#define SI4460_IOCFG0                   0x0003 /* GPIO0 IO Pin Configuration */
#define SI4460_SYNC3                    0x0004 /* Sync Word Configuration [31:24] */
#define SI4460_SYNC2                    0x0005 /* Sync Word Configuration [23:16] */
#define SI4460_SYNC1                    0x0006 /* Sync Word Configuration [15:8] */
#define SI4460_SYNC0                    0x0007 /* Sync Word Configuration [7:0] */
#define SI4460_SYNC_CFG1                0x0008 /* Sync Word Detection Configuration Reg. 1 */
#define SI4460_SYNC_CFG0                0x0009 /* Sync Word Detection Configuration Reg. 0 */
#define SI4460_DEVIATION_M              0x000A /* Frequency Deviation Configuration */
#define SI4460_MODCFG_DEV_E             0x000B /* Modulation Format and Frequency Deviation Configur.. */
#define SI4460_DCFILT_CFG               0x000C /* Digital DC Removal Configuration */
#define SI4460_PREAMBLE_CFG1            0x000D /* Preamble Length Configuration Reg. 1 */
#define SI4460_PREAMBLE_CFG0            0x000E /* Preamble Detection Configuration Reg. 0 */
#define SI4460_IQIC                     0x000F /* Digital Image Channel Compensation Configuration */
#define SI4460_CHAN_BW                  0x0010 /* Channel Filter Configuration */
#define SI4460_MDMCFG1                  0x0011 /* General Modem Parameter Configuration Reg. 1 */
#define SI4460_MDMCFG0                  0x0012 /* General Modem Parameter Configuration Reg. 0 */
#define SI4460_SYMBOL_RATE2             0x0013 /* Symbol Rate Configuration Exponent and Mantissa [1.. */
#define SI4460_SYMBOL_RATE1             0x0014 /* Symbol Rate Configuration Mantissa [15:8] */
#define SI4460_SYMBOL_RATE0             0x0015 /* Symbol Rate Configuration Mantissa [7:0] */
#define SI4460_AGC_REF                  0x0016 /* AGC Reference Level Configuration */
#define SI4460_AGC_CS_THR               0x0017 /* Carrier Sense Threshold Configuration */
#define SI4460_AGC_GAIN_ADJUST          0x0018 /* RSSI Offset Configuration */
#define SI4460_AGC_CFG3                 0x0019 /* Automatic Gain Control Configuration Reg. 3 */
#define SI4460_AGC_CFG2                 0x001A /* Automatic Gain Control Configuration Reg. 2 */
#define SI4460_AGC_CFG1                 0x001B /* Automatic Gain Control Configuration Reg. 1 */
#define SI4460_AGC_CFG0                 0x001C /* Automatic Gain Control Configuration Reg. 0 */
#define SI4460_FIFO_CFG                 0x001D /* FIFO Configuration */
#define SI4460_DEV_ADDR                 0x001E /* Device Address Configuration */
#define SI4460_SETTLING_CFG             0x001F /* Frequency Synthesizer Calibration and Settling Con.. */
#define SI4460_FS_CFG                   0x0020 /* Frequency Synthesizer Configuration */
#define SI4460_WOR_CFG1                 0x0021 /* eWOR Configuration Reg. 1 */
#define SI4460_WOR_CFG0                 0x0022 /* eWOR Configuration Reg. 0 */
#define SI4460_WOR_EVENT0_MSB           0x0023 /* Event 0 Configuration MSB */
#define SI4460_WOR_EVENT0_LSB           0x0024 /* Event 0 Configuration LSB */
#define SI4460_RXDCM_TIME               0x0025 /* RX Duty Cycle Mode Configuration */
#define SI4460_PKT_CFG2                 0x0026 /* Packet Configuration Reg. 2 */
#define SI4460_PKT_CFG1                 0x0027 /* Packet Configuration Reg. 1 */
#define SI4460_PKT_CFG0                 0x0028 /* Packet Configuration Reg. 0 */
#define SI4460_RFEND_CFG1               0x0029 /* RFEND Configuration Reg. 1 */
#define SI4460_RFEND_CFG0               0x002A /* RFEND Configuration Reg. 0 */
#define SI4460_PA_CFG1                  0x002B /* Power Amplifier Configuration Reg. 1 */
#define SI4460_PA_CFG0                  0x002C /* Power Amplifier Configuration Reg. 0 */
#define SI4460_ASK_CFG                  0x002D /* ASK Configuration */
#define SI4460_PKT_LEN                  0x002E /* Packet Length Configuration */
#define SI4460_IF_MIX_CFG               0x2F00 /* IF Mix Configuration */
#define SI4460_FREQOFF_CFG              0x2F01 /* Frequency Offset Correction Configuration */
#define SI4460_TOC_CFG                  0x2F02 /* Timing Offset Correction Configuration */
#define SI4460_MARC_SPARE               0x2F03 /* MARC Spare */
#define SI4460_ECG_CFG                  0x2F04 /* External Clock Frequency Configuration */
#define SI4460_MDMCFG2                  0x2F05 /* General Modem Parameter Configuration Reg. 2 */
#define SI4460_EXT_CTRL                 0x2F06 /* External Control Configuration */
#define SI4460_RCCAL_FINE               0x2F07 /* RC Oscillator Calibration Fine */
#define SI4460_RCCAL_COARSE             0x2F08 /* RC Oscillator Calibration Coarse */
#define SI4460_RCCAL_OFFSET             0x2F09 /* RC Oscillator Calibration Clock Offset */
#define SI4460_FREQOFF1                 0x2F0A /* Frequency Offset MSB */
#define SI4460_FREQOFF0                 0x2F0B /* Frequency Offset LSB */
#define SI4460_FREQ2                    0x2F0C /* Frequency Configuration [23:16] */
#define SI4460_FREQ1                    0x2F0D /* Frequency Configuration [15:8] */
#define SI4460_FREQ0                    0x2F0E /* Frequency Configuration [7:0] */
#define SI4460_IF_ADC2                  0x2F0F /* Analog to Digital Converter Configuration Reg. 2 */
#define SI4460_IF_ADC1                  0x2F10 /* Analog to Digital Converter Configuration Reg. 1 */
#define SI4460_IF_ADC0                  0x2F11 /* Analog to Digital Converter Configuration Reg. 0 */
#define SI4460_FS_DIG1                  0x2F12 /* Frequency Synthesizer Digital Reg. 1 */
#define SI4460_FS_DIG0                  0x2F13 /* Frequency Synthesizer Digital Reg. 0 */
#define SI4460_FS_CAL3                  0x2F14 /* Frequency Synthesizer Calibration Reg. 3 */
#define SI4460_FS_CAL2                  0x2F15 /* Frequency Synthesizer Calibration Reg. 2 */
#define SI4460_FS_CAL1                  0x2F16 /* Frequency Synthesizer Calibration Reg. 1 */
#define SI4460_FS_CAL0                  0x2F17 /* Frequency Synthesizer Calibration Reg. 0 */
#define SI4460_FS_CHP                   0x2F18 /* Frequency Synthesizer Charge Pump Configuration */
#define SI4460_FS_DIVTWO                0x2F19 /* Frequency Synthesizer Divide by 2 */
#define SI4460_FS_DSM1                  0x2F1A /* FS Digital Synthesizer Module Configuration Reg. 1 */
#define SI4460_FS_DSM0                  0x2F1B /* FS Digital Synthesizer Module Configuration Reg. 0 */
#define SI4460_FS_DVC1                  0x2F1C /* Frequency Synthesizer Divider Chain Configuration .. */
#define SI4460_FS_DVC0                  0x2F1D /* Frequency Synthesizer Divider Chain Configuration .. */
#define SI4460_FS_LBI                   0x2F1E /* Frequency Synthesizer Local Bias Configuration */
#define SI4460_FS_PFD                   0x2F1F /* Frequency Synthesizer Phase Frequency Detector Con.. */
#define SI4460_FS_PRE                   0x2F20 /* Frequency Synthesizer Prescaler Configuration */
#define SI4460_FS_REG_DIV_CML           0x2F21 /* Frequency Synthesizer Divider Regulator Configurat.. */
#define SI4460_FS_SPARE                 0x2F22 /* Frequency Synthesizer Spare */
#define SI4460_FS_VCO4                  0x2F23 /* FS Voltage Controlled Oscillator Configuration Reg.. */
#define SI4460_FS_VCO3                  0x2F24 /* FS Voltage Controlled Oscillator Configuration Reg.. */
#define SI4460_FS_VCO2                  0x2F25 /* FS Voltage Controlled Oscillator Configuration Reg.. */
#define SI4460_FS_VCO1                  0x2F26 /* FS Voltage Controlled Oscillator Configuration Reg.. */
#define SI4460_FS_VCO0                  0x2F27 /* FS Voltage Controlled Oscillator Configuration Reg.. */
#define SI4460_GBIAS6                   0x2F28 /* Global Bias Configuration Reg. 6 */
#define SI4460_GBIAS5                   0x2F29 /* Global Bias Configuration Reg. 5 */
#define SI4460_GBIAS4                   0x2F2A /* Global Bias Configuration Reg. 4 */
#define SI4460_GBIAS3                   0x2F2B /* Global Bias Configuration Reg. 3 */
#define SI4460_GBIAS2                   0x2F2C /* Global Bias Configuration Reg. 2 */
#define SI4460_GBIAS1                   0x2F2D /* Global Bias Configuration Reg. 1 */
#define SI4460_GBIAS0                   0x2F2E /* Global Bias Configuration Reg. 0 */
#define SI4460_IFAMP                    0x2F2F /* Intermediate Frequency Amplifier Configuration */
#define SI4460_LNA                      0x2F30 /* Low Noise Amplifier Configuration */
#define SI4460_RXMIX                    0x2F31 /* RX Mixer Configuration */
#define SI4460_XOSC5                    0x2F32 /* Crystal Oscillator Configuration Reg. 5 */
#define SI4460_XOSC4                    0x2F33 /* Crystal Oscillator Configuration Reg. 4 */
#define SI4460_XOSC3                    0x2F34 /* Crystal Oscillator Configuration Reg. 3 */
#define SI4460_XOSC2                    0x2F35 /* Crystal Oscillator Configuration Reg. 2 */
#define SI4460_XOSC1                    0x2F36 /* Crystal Oscillator Configuration Reg. 1 */
#define SI4460_XOSC0                    0x2F37 /* Crystal Oscillator Configuration Reg. 0 */
#define SI4460_ANALOG_SPARE             0x2F38 /* Analog Spare */
#define SI4460_PA_CFG3                  0x2F39 /* Power Amplifier Configuration Reg. 3 */
#define SI4460_WOR_TIME1                0x2F64 /* eWOR Timer Counter Value MSB */
#define SI4460_WOR_TIME0                0x2F65 /* eWOR Timer Counter Value LSB */
#define SI4460_WOR_CAPTURE1             0x2F66 /* eWOR Timer Capture Value MSB */
#define SI4460_WOR_CAPTURE0             0x2F67 /* eWOR Timer Capture Value LSB */
#define SI4460_BIST                     0x2F68 /* MARC Built-In Self-Test */
#define SI4460_DCFILTOFFSET_I1          0x2F69 /* DC Filter Offset I MSB */
#define SI4460_DCFILTOFFSET_I0          0x2F6A /* DC Filter Offset I LSB */
#define SI4460_DCFILTOFFSET_Q1          0x2F6B /* DC Filter Offset Q MSB */
#define SI4460_DCFILTOFFSET_Q0          0x2F6C /* DC Filter Offset Q LSB */
#define SI4460_IQIE_I1                  0x2F6D /* IQ Imbalance Value I MSB */
#define SI4460_IQIE_I0                  0x2F6E /* IQ Imbalance Value I LSB */
#define SI4460_IQIE_Q1                  0x2F6F /* IQ Imbalance Value Q MSB */
#define SI4460_IQIE_Q0                  0x2F70 /* IQ Imbalance Value Q LSB */
#define SI4460_RSSI1                    0x2F71 /* Received Signal Strength Indicator Reg. 1 */
#define SI4460_RSSI0                    0x2F72 /* Received Signal Strength Indicator Reg.0 */
#define SI4460_MARCSTATE                0x2F73 /* MARC State */
#define SI4460_LQI_VAL                  0x2F74 /* Link Quality Indicator Value */
#define SI4460_PQT_SYNC_ERR             0x2F75 /* Preamble and Sync Word Error */
#define SI4460_DEM_STATUS               0x2F76 /* Demodulator Status */
#define SI4460_FREQOFF_EST1             0x2F77 /* Frequency Offset Estimate MSB */
#define SI4460_FREQOFF_EST0             0x2F78 /* Frequency Offset Estimate LSB */
#define SI4460_AGC_GAIN3                0x2F79 /* Automatic Gain Control Reg. 3 */
#define SI4460_AGC_GAIN2                0x2F7A /* Automatic Gain Control Reg. 2 */
#define SI4460_AGC_GAIN1                0x2F7B /* Automatic Gain Control Reg. 1 */
#define SI4460_AGC_GAIN0                0x2F7C /* Automatic Gain Control Reg. 0 */
#define SI4460_CFM_RX_DATA_OUT          0x2F7D /* Custom Frequency Modulation RX Data */
#define SI4460_CFM_TX_DATA_IN           0x2F7E /* Custom Frequency Modulation TX Data */
#define SI4460_ASK_SOFT_RX_DATA         0x2F7F /* ASK Soft Decision Output */
#define SI4460_RNDGEN                   0x2F80 /* Random Number Generator Value */
#define SI4460_MAGN2                    0x2F81 /* Signal Magnitude after CORDIC [16] */
#define SI4460_MAGN1                    0x2F82 /* Signal Magnitude after CORDIC [15:8] */
#define SI4460_MAGN0                    0x2F83 /* Signal Magnitude after CORDIC [7:0] */
#define SI4460_ANG1                     0x2F84 /* Signal Angular after CORDIC [9:8] */
#define SI4460_ANG0                     0x2F85 /* Signal Angular after CORDIC [7:0] */
#define SI4460_CHFILT_I2                0x2F86 /* Channel Filter Data Real Part [16] */
#define SI4460_CHFILT_I1                0x2F87 /* Channel Filter Data Real Part [15:8] */
#define SI4460_CHFILT_I0                0x2F88 /* Channel Filter Data Real Part [7:0] */
#define SI4460_CHFILT_Q2                0x2F89 /* Channel Filter Data Imaginary Part [16] */
#define SI4460_CHFILT_Q1                0x2F8A /* Channel Filter Data Imaginary Part [15:8] */
#define SI4460_CHFILT_Q0                0x2F8B /* Channel Filter Data Imaginary Part [7:0] */
#define SI4460_GPIO_STATUS              0x2F8C /* General Purpose Input/Output Status */
#define SI4460_FSCAL_CTRL               0x2F8D /* Frequency Synthesizer Calibration Control */
#define SI4460_PHASE_ADJUST             0x2F8E /* Frequency Synthesizer Phase Adjust */
#define SI4460_PARTNUMBER               0x2F8F /* Part Number */
#define SI4460_PARTVERSION              0x2F90 /* Part Revision */
#define SI4460_SERIAL_STATUS            0x2F91 /* Serial Status */
#define SI4460_MODEM_STATUS1            0x2F92 /* Modem Status Reg. 1 */
#define SI4460_MODEM_STATUS0            0x2F93 /* Modem Status Reg. 0 */
#define SI4460_MARC_STATUS1             0x2F94 /* MARC Status Reg. 1 */
#define SI4460_MARC_STATUS0             0x2F95 /* MARC Status Reg. 0 */
#define SI4460_PA_IFAMP_TEST            0x2F96 /* Power Amplifier Intermediate Frequency Amplifier T.. */
#define SI4460_FSRF_TEST                0x2F97 /* Frequency Synthesizer Test */
#define SI4460_PRE_TEST                 0x2F98 /* Frequency Synthesizer Prescaler Test */
#define SI4460_PRE_OVR                  0x2F99 /* Frequency Synthesizer Prescaler Override */
#define SI4460_ADC_TEST                 0x2F9A /* Analog to Digital Converter Test */
#define SI4460_DVC_TEST                 0x2F9B /* Digital Divider Chain Test */
#define SI4460_ATEST                    0x2F9C /* Analog Test */
#define SI4460_ATEST_LVDS               0x2F9D /* Analog Test LVDS */
#define SI4460_ATEST_MODE               0x2F9E /* Analog Test Mode */
#define SI4460_XOSC_TEST1               0x2F9F /* Crystal Oscillator Test Reg. 1 */
#define SI4460_XOSC_TEST0               0x2FA0 /* Crystal Oscillator Test Reg. 0 */
#define SI4460_AES                      0x2FA1 /* AES */
#define SI4460_MDM_TEST                 0x2FA2 /* MODEM Test */
#define SI4460_RXFIRST                  0x2FD2 /* RX FIFO Pointer First Entry */
#define SI4460_TXFIRST                  0x2FD3 /* TX FIFO Pointer First Entry */
#define SI4460_RXLAST                   0x2FD4 /* RX FIFO Pointer Last Entry */
#define SI4460_TXLAST                   0x2FD5 /* TX FIFO Pointer Last Entry */
#define SI4460_NUM_TXBYTES              0x2FD6 /* TX FIFO Status */
#define SI4460_NUM_RXBYTES              0x2FD7 /* RX FIFO Status */
#define SI4460_FIFO_NUM_TXBYTES         0x2FD8 /* TX FIFO Status */
#define SI4460_FIFO_NUM_RXBYTES         0x2FD9 /* RX FIFO Status */
#define SI4460_RXFIFO_PRE_BUF           0x2FDA /* RX FIFO Status */
#define SI4460_AES_KEY15                0x2FE0 /* Advanced Encryption Standard Key [127:120] */
#define SI4460_AES_KEY14                0x2FE1 /* Advanced Encryption Standard Key [119:112] */
#define SI4460_AES_KEY13                0x2FE2 /* Advanced Encryption Standard Key [111:104] */
#define SI4460_AES_KEY12                0x2FE3 /* Advanced Encryption Standard Key [103:96] */
#define SI4460_AES_KEY11                0x2FE4 /* Advanced Encryption Standard Key [95:88] */
#define SI4460_AES_KEY10                0x2FE5 /* Advanced Encryption Standard Key [87:80] */
#define SI4460_AES_KEY9                 0x2FE6 /* Advanced Encryption Standard Key [79:72] */
#define SI4460_AES_KEY8                 0x2FE7 /* Advanced Encryption Standard Key [71:64] */
#define SI4460_AES_KEY7                 0x2FE8 /* Advanced Encryption Standard Key [63:56] */
#define SI4460_AES_KEY6                 0x2FE9 /* Advanced Encryption Standard Key [55:48] */
#define SI4460_AES_KEY5                 0x2FEA /* Advanced Encryption Standard Key [47:40] */
#define SI4460_AES_KEY4                 0x2FEB /* Advanced Encryption Standard Key [39:32] */
#define SI4460_AES_KEY3                 0x2FEC /* Advanced Encryption Standard Key [31:24] */
#define SI4460_AES_KEY2                 0x2FED /* Advanced Encryption Standard Key [23:16] */
#define SI4460_AES_KEY1                 0x2FEE /* Advanced Encryption Standard Key [15:8] */
#define SI4460_AES_KEY0                 0x2FEF /* Advanced Encryption Standard Key [7:0] */
#define SI4460_AES_BUFFER15             0x2FF0 /* Advanced Encryption Standard Buffer [127:120] */
#define SI4460_AES_BUFFER14             0x2FF1 /* Advanced Encryption Standard Buffer [119:112] */
#define SI4460_AES_BUFFER13             0x2FF2 /* Advanced Encryption Standard Buffer [111:104] */
#define SI4460_AES_BUFFER12             0x2FF3 /* Advanced Encryption Standard Buffer [103:93] */
#define SI4460_AES_BUFFER11             0x2FF4 /* Advanced Encryption Standard Buffer [95:88] */
#define SI4460_AES_BUFFER10             0x2FF5 /* Advanced Encryption Standard Buffer [87:80] */
#define SI4460_AES_BUFFER9              0x2FF6 /* Advanced Encryption Standard Buffer [79:72] */
#define SI4460_AES_BUFFER8              0x2FF7 /* Advanced Encryption Standard Buffer [71:64] */
#define SI4460_AES_BUFFER7              0x2FF8 /* Advanced Encryption Standard Buffer [63:56] */
#define SI4460_AES_BUFFER6              0x2FF9 /* Advanced Encryption Standard Buffer [55:48] */
#define SI4460_AES_BUFFER5              0x2FFA /* Advanced Encryption Standard Buffer [47:40] */
#define SI4460_AES_BUFFER4              0x2FFB /* Advanced Encryption Standard Buffer [39:32] */
#define SI4460_AES_BUFFER3              0x2FFC /* Advanced Encryption Standard Buffer [31:24] */
#define SI4460_AES_BUFFER2              0x2FFD /* Advanced Encryption Standard Buffer [23:16] */
#define SI4460_AES_BUFFER1              0x2FFE /* Advanced Encryption Standard Buffer [15:8] */
#define SI4460_AES_BUFFER0              0x2FFF /* Advanced Encryption Standard Buffer [7:0] */
/*---------------------------------------------------------------------------*/
/* Access to RX / TX FIFO */
#define SI4460_TXFIFO                   0x66  
#define SI4460_RXFIFO                   0x77
/*---------------------------------------------------------------------------*/
/* MARC_STATE */
#define SI4460_MARC_STATE_SLEEP         0x00
#define SI4460_MARC_STATE_IDLE          0x01
#define SI4460_MARC_STATE_RX            0x0D
#define SI4460_MARC_STATE_RX_FIFO_ERR   0x11
#define SI4460_MARC_STATE_TX            0x13
#define SI4460_MARC_STATE_TX_FIFO_ERR   0x16
/*---------------------------------------------------------------------------*/
/* Status byte */
#define SI4460_STATUS_BYTE_IDLE         (0 << 4)
#define SI4460_STATUS_BYTE_RX           (1 << 4)
#define SI4460_STATUS_BYTE_TX           (2 << 4)
#define SI4460_STATUS_BYTE_FSTXON       (3 << 4)
#define SI4460_STATUS_BYTE_CALIBRATE    (4 << 4)
#define SI4460_STATUS_BYTE_SETTLING     (5 << 4)
#define SI4460_STATUS_BYTE_RX_FIFO_ERR  (6 << 4)
#define SI4460_STATUS_BYTE_TX_FIFO_ERR  (7 << 4)
/*---------------------------------------------------------------------------*/
/* GPIO configuration */
#define SI4460_IOCFG_RXFIFO_THR         0
#define SI4460_IOCFG_RXFFIFO_THR_PKT    1
#define SI4460_IOCFG_TXFIFO_THR         2
#define SI4460_IOCFG_PKT_SYNC_RXTX      6
#define SI4460_IOCFG_SERIAL_CLK         8
#define SI4460_IOCFG_SERIAL_RX          9
#define SI4460_IOCFG_CARRIER_SENSE      17
#define SI4460_IOCFG_MARC_2PIN_STATUS_1 37
#define SI4460_IOCFG_MARC_2PIN_STATUS_0 38
#define SI4460_IOCFG_RXFIFO_CHIP_RDY_N  50
/*---------------------------------------------------------------------------*/
/* Command strobes */
#define SI4460_SRES                     0x30
#define SI4460_SFSTXON                  0x31
#define SI4460_SXOFF                    0x32
#define SI4460_SCAL                     0x33
#define SI4460_SRX                      0x34
#define SI4460_STX                      0x35
#define SI4460_SIDLE                    0x36
#define SI4460_SPWD                     0x39
#define SI4460_SFRX                     0x3A
#define SI4460_SFTX                     0x3B
#define SI4460_SNOP                     0x3D
/*---------------------------------------------------------------------------*/
/* SPI access modifier */
#define SI4460_WRITE_BIT                0x00
#define SI4460_READ_BIT                 0x80
#define SI4460_BURST_BIT                0x40
#define SI4460_EXTENDED_WRITE_CMD       (0x2F | SI4460_WRITE_BIT)
#define SI4460_EXTENDED_BURST_WRITE_CMD \
  (0x2F | SI4460_BURST_BIT | SI4460_WRITE_BIT)
#define SI4460_EXTENDED_READ_CMD        (0x2F | SI4460_READ_BIT)
#define SI4460_EXTENDED_BURST_READ_CMD \
  (0x2F | SI4460_BURST_BIT | SI4460_READ_BIT)
/*---------------------------------------------------------------------------*/
#define SI4460_IS_EXTENDED_ADDR(x)      (x & 0x2F00)
/*---------------------------------------------------------------------------*/
/* RSSI0 register */
#define SI4460_CARRIER_SENSE_VALID      (1 << 1)
#define SI4460_CARRIER_SENSE            (1 << 2)
/*---------------------------------------------------------------------------*/
/* MODEM_STATUS1 register */
#define SI4460_SYNC_FOUND               (1 << 7)
#define SI4460_PQT_REACHED              (1 << 1)
/*---------------------------------------------------------------------------*/
#define SI4460_FIFO_SIZE                128
/*---------------------------------------------------------------------------*/
/* Output power in dBm */
/* Up to now we don't handle the special power levels PA_POWER_RAMP < 3, hence
 * the minimum tx power is -16. See update_txpower().
 */
#define SI4460_CONST_TX_POWER_MIN       (-16)
/* 
 * Maximum output power will propably depend on the band we use due to 
 * regulation issues 
 */
#define SI4460_CONST_TX_POWER_MAX       14
/*---------------------------------------------------------------------------*/
/* CCA threshold in dBm */
#define SI4460_CONST_CCA_THRESHOLD_MIN  (-127)
#define SI4460_CONST_CCA_THRESHOLD_MAX  127

#endif /* SI4460_CONST_H_ */
