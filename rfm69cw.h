/*
  Copyright (c) 2014, 2015 Fabian Mink <fabian.mink@gmx.de>
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file rfm69cw.h
 * @author Fabian Mink
 * @date 2015-01-29
 * @brief RFM69CW Driver Header
 * @copyright BSD 2-Clause License
 */

#ifndef __RFM69CW_H
#define __RFM69CW_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "rfm69cw_config.h" //Configuration
#include "rfm69cw_drv.h"    //Hardware driver (Board support package) SPI, IO etc.

#include <stdint.h>

#define FSTEP   (FXOSC/524288)
//#define FSTEP   61.035156  //(@ 32MHz)

#define RFM69CW_Return_ERROR_DRVOTHER  -4  //Hardware (SPI) Driver Other error
#define RFM69CW_Return_ERROR_DRVBUF    -3  //Hardware (SPI) Driver Buffer under/overflow
#define RFM69CW_Return_ERROR_DRVTIME   -2  //Hardware (SPI) Driver Timeout
#define RFM69CW_Return_ERROR_UNSPEC    -1
#define RFM69CW_Return_ERROR_NONE       0
#define RFM69CW_Return_FALSE            0
#define RFM69CW_Return_TRUE             1

#define RFM69CW_TO_RFFREQ(rfFreqHz)             (uint32_t) (((rfFreqHz) / FSTEP)  + 0.5)   //from rffreq/Hz to value
#define RFM69CW_TO_FDEV(fdevHz)                 (uint16_t) (((fdevHz) / FSTEP)  + 0.5)     //from fdev/Hz to value
#define RFM69CW_TO_BITRATE(bitrate)             (uint16_t) ((FXOSC / (bitrate)) + 0.5)     //from bitrate/bps to value
#define RFM69CW_TO_RSSITHRESHOLD(rssiThreshold) (uint8_t)  (((rssiThreshold)*-2.0) + 0.5)  //from rssiThreshold/dBm to value

/** 
  * @brief   RFM69CW Common Configuration Init structure definition
  */ 
typedef struct {
	unsigned int RFM69CW_Sequencer         : 1; /*!< Operating mode is automatically reached
                                                     with the sequencer. */
	unsigned int RFM69CW_DataMode          : 2;
	unsigned int RFM69CW_ModulationType    : 2;
	unsigned int RFM69CW_ModulationShaping : 2;
	uint32_t     RFM69CW_RfFreq;
	uint16_t     RFM69CW_FreqDev;
	uint16_t     RFM69CW_BitRate;
} RFM69CW_CommonInit_t;

#define RFM69CW_Sequencer_On                       0
#define RFM69CW_Sequencer_Off                      1

#define RFM69CW_DataMode_Packet                    0
#define RFM69CW_DataMode_ContWithBitSync           2
#define RFM69CW_DataMode_ContWithoutBitSync        3

#define RFM69CW_ModulationType_FSK                 0
#define RFM69CW_ModulationType_OOK                 1

#define RFM69CW_ModulationShaping_NONE             0
#define RFM69CW_ModulationShaping_FSK_NONE         0
#define RFM69CW_ModulationShaping_FSK_GAUSS_BT1_0  1
#define RFM69CW_ModulationShaping_FSK_GAUSS_BT0_5  2
#define RFM69CW_ModulationShaping_FSK_GAUSS_BT0_3  3
#define RFM69CW_ModulationShaping_OOK_NONE         0
#define RFM69CW_ModulationShaping_OOK_BR1          1
#define RFM69CW_ModulationShaping_OOK_BR2          2


/**
  * @brief   RFM69CW Tx Configuration Init structure definition
  */
typedef struct {
	unsigned int RFM69CW_OutputPower : 5;
	unsigned int RFM69CW_PaRamp      : 4;
	unsigned int RFM69CW_Ocp         : 1;
	unsigned int RFM69CW_OcpCurrent  : 4;
} RFM69CW_TxInit_t;

//L_Pout = (-18 + OutputPower) * dBm
//Power values in mW
#define RFM69CW_OutputPower_0_1mW       8
#define RFM69CW_OutputPower_1mW         18
#define RFM69CW_OutputPower_2mW         21
#define RFM69CW_OutputPower_5mW         25
#define RFM69CW_OutputPower_10mW        28
#define RFM69CW_OutputPower_20mW        31
//same values in dBm
#define RFM69CW_OutputPower_0dBm        18
#define RFM69CW_OutputPower_1dBm        19
#define RFM69CW_OutputPower_2dBm        20
#define RFM69CW_OutputPower_3dBm        21
#define RFM69CW_OutputPower_7dBm        25
#define RFM69CW_OutputPower_10dBm       28
#define RFM69CW_OutputPower_13dBm       31
//min / max power
#define RFM69CW_OutputPower_MIN         0
#define RFM69CW_OutputPower_MAX         31

#define RFM69CW_PaRamp_3_4ms            0x0
#define RFM69CW_PaRamp_2ms              0x1
#define RFM69CW_PaRamp_1ms              0x2
#define RFM69CW_PaRamp_500us            0x3
#define RFM69CW_PaRamp_250us            0x4
#define RFM69CW_PaRamp_125us            0x5
#define RFM69CW_PaRamp_100us            0x6
#define RFM69CW_PaRamp_62us             0x7
#define RFM69CW_PaRamp_50us             0x8
#define RFM69CW_PaRamp_40us             0x9
#define RFM69CW_PaRamp_31us             0xA
#define RFM69CW_PaRamp_25us             0xB
#define RFM69CW_PaRamp_20us             0xC
#define RFM69CW_PaRamp_15us             0xD
#define RFM69CW_PaRamp_12us             0xE
#define RFM69CW_PaRamp_10us             0xF

#define RFM69CW_Ocp_Off                 0
#define RFM69CW_Ocp_On                  1

#define RFM69CW_OcpCurrent_45mA         0x0
#define RFM69CW_OcpCurrent_50mA         0x1
#define RFM69CW_OcpCurrent_55mA         0x2
#define RFM69CW_OcpCurrent_60mA         0x3
#define RFM69CW_OcpCurrent_65mA         0x4
#define RFM69CW_OcpCurrent_70mA         0x5
#define RFM69CW_OcpCurrent_75mA         0x6
#define RFM69CW_OcpCurrent_80mA         0x7
#define RFM69CW_OcpCurrent_85mA         0x8
#define RFM69CW_OcpCurrent_95mA         0xA
#define RFM69CW_OcpCurrent_100mA        0xB
#define RFM69CW_OcpCurrent_105mA        0xC
#define RFM69CW_OcpCurrent_110mA        0xD
#define RFM69CW_OcpCurrent_115mA        0xE
#define RFM69CW_OcpCurrent_120mA        0xF

/**
  * @brief   RFM69CW Rx Configuration Init structure definition
  */
typedef struct {
	unsigned int RFM69CW_LnaZin        :1;  //Input Impedance
	unsigned int RFM69CW_LnaGainSelect :3;  //LNA Gain Setting
	unsigned int RFM69CW_DccFreq       :3;  //DC offset canceler cutoff frequency in % of RxBw
	unsigned int RFM69CW_RxBw          :5;  //Channel filter bandwidth
	unsigned int RFM69CW_DccFreqAfc    :3;  //Same values as above, ...
	unsigned int RFM69CW_RxBwAfc       :5;  //...but used during AFC
	uint8_t      RFM69CW_RssiThresh;        //Rssi threshold
	//uint8_t      TimeoutRxStart;          //Timeout after switching to Rx mode if Rssi interrupt doesn't occur
	//uint8_t      TimeoutRssiThresh;       //Timeout after Rssi interrupt if PayloadReady interrupt doesn’t occur
	//todo: add ook settings, when needed
} RFM69CW_RxInit_t;

#define RFM69CW_LnaZin_50ohms           0
#define RFM69CW_LnaZin_200ohms          1

#define RFM69CW_LnaGainSelect_AGC       0
#define RFM69CW_LnaGainSelect_MAX       1
#define RFM69CW_LnaGainSelect_DAMP6dB   2
#define RFM69CW_LnaGainSelect_DAMP12dB  3
#define RFM69CW_LnaGainSelect_DAMP24dB  4
#define RFM69CW_LnaGainSelect_DAMP36dB  5
#define RFM69CW_LnaGainSelect_DAMP48dB  6

//in percent of RxBw
//if cutoff freq. is increased -> DC components are more suppressed
//but long periods of constant bit values "0" or "1" are no more allowed
#define RFM69CW_DccFreq_16p        0
#define RFM69CW_DccFreq_8p         1
#define RFM69CW_DccFreq_4p         2
#define RFM69CW_DccFreq_2p         3
#define RFM69CW_DccFreq_1p         4
#define RFM69CW_DccFreq_0_5p       5
#define RFM69CW_DccFreq_0_25p      6
#define RFM69CW_DccFreq_0_125p     7

//hi 2bits are mantissa, lo 3bits are exponent
//For OOK, exactly half the bandwidth is resulting for the same value
#define RFM69CW_RxBw_FSK_500kHz    ((0<<3) + 0)
#define RFM69CW_RxBw_FSK_400kHz    ((1<<3) + 0)
#define RFM69CW_RxBw_FSK_333kHz    ((2<<3) + 0)
#define RFM69CW_RxBw_FSK_250kHz    ((0<<3) + 1)
#define RFM69CW_RxBw_FSK_200kHz    ((1<<3) + 1)
#define RFM69CW_RxBw_FSK_167kHz    ((2<<3) + 1)
#define RFM69CW_RxBw_FSK_125kHz    ((0<<3) + 2)
#define RFM69CW_RxBw_FSK_100kHz    ((1<<3) + 2)
#define RFM69CW_RxBw_FSK_83_3kHz   ((2<<3) + 2)
#define RFM69CW_RxBw_FSK_62_5kHz   ((0<<3) + 3)
#define RFM69CW_RxBw_FSK_50_0kHz   ((1<<3) + 3)
#define RFM69CW_RxBw_FSK_41_7kHz   ((2<<3) + 3)
#define RFM69CW_RxBw_FSK_31_3kHz   ((0<<3) + 4)
#define RFM69CW_RxBw_FSK_25_0kHz   ((1<<3) + 4)
#define RFM69CW_RxBw_FSK_20_8kHz   ((2<<3) + 4)
#define RFM69CW_RxBw_FSK_15_6kHz   ((0<<3) + 5)
#define RFM69CW_RxBw_FSK_12_5kHz   ((1<<3) + 5)
#define RFM69CW_RxBw_FSK_10_4kHz   ((2<<3) + 5)
#define RFM69CW_RxBw_FSK_7_8kHz    ((0<<3) + 6)
#define RFM69CW_RxBw_FSK_6_3kHz    ((1<<3) + 6)
#define RFM69CW_RxBw_FSK_5_2kHz    ((2<<3) + 6)
#define RFM69CW_RxBw_FSK_3_9kHz    ((0<<3) + 7)
#define RFM69CW_RxBw_FSK_3_1kHz    ((1<<3) + 7)
#define RFM69CW_RxBw_FSK_2_6kHz    ((2<<3) + 7)

#define RFM69CW_RxBw_OOK_250kHz    ((0<<3) + 0)
#define RFM69CW_RxBw_OOK_200kHz    ((1<<3) + 0)
#define RFM69CW_RxBw_OOK_167kHz    ((2<<3) + 0)
#define RFM69CW_RxBw_OOK_125kHz    ((0<<3) + 1)
#define RFM69CW_RxBw_OOK_100kHz    ((1<<3) + 1)
#define RFM69CW_RxBw_OOK_83_3kHz   ((2<<3) + 1)
#define RFM69CW_RxBw_OOK_62_5kHz   ((0<<3) + 2)
#define RFM69CW_RxBw_OOK_50_0kHz   ((1<<3) + 2)
#define RFM69CW_RxBw_OOK_41_7kHz   ((2<<3) + 2)
#define RFM69CW_RxBw_OOK_31_3kHz   ((0<<3) + 3)
#define RFM69CW_RxBw_OOK_25_0kHz   ((1<<3) + 3)
#define RFM69CW_RxBw_OOK_20_8kHz   ((2<<3) + 3)
#define RFM69CW_RxBw_OOK_15_6kHz   ((0<<3) + 4)
#define RFM69CW_RxBw_OOK_12_5kHz   ((1<<3) + 4)
#define RFM69CW_RxBw_OOK_10_4kHz   ((2<<3) + 4)
#define RFM69CW_RxBw_OOK_7_8kHz    ((0<<3) + 5)
#define RFM69CW_RxBw_OOK_6_3kHz    ((1<<3) + 5)
#define RFM69CW_RxBw_OOK_5_2kHz    ((2<<3) + 5)
#define RFM69CW_RxBw_OOK_3_9kHz    ((0<<3) + 6)
#define RFM69CW_RxBw_OOK_3_1kHz    ((1<<3) + 6)
#define RFM69CW_RxBw_OOK_2_6kHz    ((2<<3) + 6)
#define RFM69CW_RxBw_OOK_2_0kHz    ((0<<3) + 7)
#define RFM69CW_RxBw_OOK_1_6kHz    ((1<<3) + 7)
#define RFM69CW_RxBw_OOK_1_3kHz    ((2<<3) + 7)

/**
  * @brief   RFM69CW Packet Engine Configuration Init structure definition
  */
typedef struct {
	uint16_t     RFM69CW_PreambleSize;
	unsigned int RFM69CW_SyncWordSize       : 4; //1..8 bytes, 0 = no sync word
	unsigned int RFM69CW_SyncTol            : 3;
	uint8_t      RFM69CW_SyncValue[8];
	unsigned int RFM69CW_PacketFormat       : 1;
	unsigned int RFM69CW_DcFree             : 2;
	unsigned int RFM69CW_Crc                : 1;
	unsigned int RFM69CW_CrcAutoClear       : 1;
	unsigned int RFM69CW_AddressFiltering   : 2;
	uint8_t      RFM69CW_PayloadLength;          //For fixed format: 0=unlimited, >0=length. For variable format: Not used for Tx, max length in Rx
	uint8_t      RFM69CW_NodeAddress;
	uint8_t      RFM69CW_BroadcastAddress;
	unsigned int RFM69CW_TxStartCondition   : 1;
	unsigned int RFM69CW_FifoThreshold      : 7;
	unsigned int RFM69CW_InterPacketRxDelay : 4;
	unsigned int RFM69CW_AutoRxRestart      : 1;
	unsigned int RFM69CW_Aes                : 1;
} RFM69CW_PacketEngineInit_t;


#define RFM69CW_PacketFormat_Fixed               0
#define RFM69CW_PacketFormat_Variable            1

#define RFM69CW_DcFree_None                      0
#define RFM69CW_DcFree_Manchester                1
#define RFM69CW_DcFree_Whitening                 2

#define RFM69CW_Crc_Off                          0
#define RFM69CW_Crc_On                           1

#define RFM69CW_CrcAutoClear_On                  0
#define RFM69CW_CrcAutoClear_Off                 1

#define RFM69CW_AddressFiltering_None            0
#define RFM69CW_AddressFiltering_Node            1
#define RFM69CW_AddressFiltering_NodeOrBroadcast 2

#define RFM69CW_TxStartCondition_FifoLevel       0
#define RFM69CW_TxStartCondition_FifoNotEmpty    1

#define RFM69CW_InterPacketRxDelay_1bit          0
#define RFM69CW_InterPacketRxDelay_2bit          1
#define RFM69CW_InterPacketRxDelay_4bit          2
#define RFM69CW_InterPacketRxDelay_8bit          3
#define RFM69CW_InterPacketRxDelay_16bit         4
#define RFM69CW_InterPacketRxDelay_32bit         5
#define RFM69CW_InterPacketRxDelay_64bit         6
#define RFM69CW_InterPacketRxDelay_128bit        7
#define RFM69CW_InterPacketRxDelay_256bit        8
#define RFM69CW_InterPacketRxDelay_512bit        9
#define RFM69CW_InterPacketRxDelay_1024bit       10
#define RFM69CW_InterPacketRxDelay_2048bit       11
#define RFM69CW_InterPacketRxDelay_None          12

#define RFM69CW_AutoRxRestart_Off                0
#define RFM69CW_AutoRxRestart_On                 1

#define RFM69CW_Aes_Off                          0
#define RFM69CW_Aes_On                           1


/**
  * @brief   RFM69CW Handle structure definition
  */
typedef struct  {
	RFM69CW_drvInterfaceHandle_t ifHandle;
} RFM69CW_Handle_t;

extern RFM69CW_Return_t RFM69CW_Init(RFM69CW_Handle_t* handle, RFM69CW_drvInterfaceHandle_t ifHandle);

extern RFM69CW_Return_t RFM69CW_CommonInit(RFM69CW_Handle_t* handle, RFM69CW_CommonInit_t* RFM69CW_CommonInitStruct);
extern RFM69CW_Return_t RFM69CW_TxInit(RFM69CW_Handle_t* handle, RFM69CW_TxInit_t* RFM69CW_TxInitStruct);
extern RFM69CW_Return_t RFM69CW_RxInit(RFM69CW_Handle_t* handle, RFM69CW_RxInit_t* RFM69CW_RxInitStruct);
extern RFM69CW_Return_t RFM69CW_PacketEngineInit(RFM69CW_Handle_t* handle, RFM69CW_PacketEngineInit_t* RFM69CW_PacketEngineInitStruct);

extern void RFM69CW_CommonStructInit(RFM69CW_CommonInit_t* RFM69CW_CommonInitStruct);
extern void RFM69CW_TxStructInit(RFM69CW_TxInit_t* RFM69CW_TxInitStruct);
extern void RFM69CW_RxStructInit(RFM69CW_RxInit_t* RFM69CW_RxInitStruct);
extern void RFM69CW_PacketEngineStructInit(RFM69CW_PacketEngineInit_t* RFM69CW_PacketEngineInitStruct);

extern RFM69CW_Return_t RFM69CW_FifoWrite(RFM69CW_Handle_t* handle, uint8_t data);
extern RFM69CW_Return_t RFM69CW_FifoRead(RFM69CW_Handle_t* handle, uint8_t* data);
#if defined (USE_BURST_OPERATIONS) || defined (EMULATE_BURST_OPERATIONS)
extern RFM69CW_Return_t RFM69CW_FifoBurstWrite(RFM69CW_Handle_t* handle, uint8_t* data, uint8_t cnt);
extern RFM69CW_Return_t RFM69CW_FifoBurstRead(RFM69CW_Handle_t* handle, uint8_t* data, uint8_t cnt);
#endif //USE_BURST_OPERATIONS or EMULATE_BURST_OPERATIONS

extern RFM69CW_Return_t RFM69CW_GoSleep(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_GoStdby(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_GoFs(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_GoTx(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_GoRx(RFM69CW_Handle_t* handle);

extern RFM69CW_Return_t RFM69CW_SetFifoFill(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_ResetFifoFill(RFM69CW_Handle_t* handle);

extern RFM69CW_Return_t RFM69CW_IsSyncAddressMatch(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsAutoMode(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsTimeout(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsRssi(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsPllLock(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsTxReady(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsRxReady(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsModeReady(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsCrcOk(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsPayloadReady(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsPacketSent(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsFifoOverrun(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsFifoLevel(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsFifoNotEmpty(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_IsFifoFull(RFM69CW_Handle_t* handle);

extern RFM69CW_Return_t RFM69CW_IsRssiDone(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_StartRssi(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_GetRssiValue(RFM69CW_Handle_t* handle, uint8_t* rssi);
extern RFM69CW_Return_t RFM69CW_IsFeiDone(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_StartFei(RFM69CW_Handle_t* handle);
extern RFM69CW_Return_t RFM69CW_GetFeiValue(RFM69CW_Handle_t* handle, int16_t* fei);

extern RFM69CW_Return_t RFM69CW_GetLnaCurrentGain(RFM69CW_Handle_t* handle, uint8_t* gain);

extern RFM69CW_Return_t RFM69CW_GetVersion(RFM69CW_Handle_t* handle, uint8_t* versionNo);

extern RFM69CW_Return_t RFM69CW_ForceRestartRx(RFM69CW_Handle_t* handle);

#ifdef INCLUDE_SETRFFREQ_FUNCTION
extern RFM69CW_Return_t RFM69CW_SetRfFreq(RFM69CW_Handle_t* handle, uint32_t rfFreq);
#endif
#ifdef INCLUDE_SETRSSITHRESHOLD_FUNCTION
extern RFM69CW_Return_t RFM69CW_SetRssiThreshold(RFM69CW_Handle_t* handle, uint8_t rssiThresh);
#endif
#ifdef INCLUDE_SETPAYLOADLENGTH_FUNCTION
extern RFM69CW_Return_t RFM69CW_SetPayloadLength(RFM69CW_Handle_t* handle, uint8_t payloadLength);
#endif

#ifdef INCLUDE_SPECIAL_FREQ_CALC_FUNCTIONS
extern RFM69CW_Return_t RFM69CW_SetRfFreqIsm433Channel(RFM69CW_Handle_t* handle, uint8_t chanNo);
extern RFM69CW_Return_t RFM69CW_SetRfFreqPMRChannel(RFM69CW_Handle_t* handle, uint8_t chanNo);
#endif //INCLUDE_SPECIAL_FREQ_CALC_FUNCTIONS

#ifdef INCLUDE_FLOAT_CALC_FUNCTIONS
extern uint32_t RFM69CW_CalcRfFreq(float rfFreqHz);
extern uint16_t RFM69CW_CalcFdev(float FdevHz);
extern uint16_t RFM69CW_CalcBitrate(float bitrate);
extern float RFM69CW_CalcRssi(uint8_t val_rssi);
#endif //INCLUDE_FLOAT_CALC_FUNCTIONS

#ifdef INCLUDE_REGISTER_DUMP_FUNCTION
extern RFM69CW_Return_t RFM69CW_RegisterDump(RFM69CW_Handle_t* handle, uint8_t* memory);
#endif


#ifdef __cplusplus
}
#endif

#endif /* __RFM69CW_H */

