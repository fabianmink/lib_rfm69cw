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
 * @file rfm69cw.c
 * @author Fabian Mink
 * @date 2015-01-29
 * @brief RFM69CW Driver
 * @copyright BSD 2-Clause License
 *
 * <p>Driver for HOPERF RFM69CW TRX Module. Descriptive text is partly taken from rfm69cw datasheet.</p>
 *
 */

#include "rfm69cw.h"
#include "rfm69cw_config.h" //Configuration
#include "rfm69cw_reg.h" //Register definitions


#ifdef USE_SPIDRV_REGISTER_FUNCTIONS
#define READ_REG    rfm69cw_drv_spiReadReg
#define WRITE_REG   rfm69cw_drv_spiWriteReg
extern RFM69CW_Return_t rfm69cw_drv_spiReadReg(uint8_t adr, uint8_t* data, RFM69CW_drvInterfaceHandle_t interface);
extern RFM69CW_Return_t rfm69cw_drv_spiWriteReg(uint8_t adr, uint8_t data, RFM69CW_drvInterfaceHandle_t interface);
#ifdef USE_BURST_OPERATIONS
#define READ_BURST_REG   rfm69cw_drv_spiReadBurstReg
#define WRITE_BURST_REG  rfm69cw_drv_spiWriteBurstReg
extern RFM69CW_Return_t rfm69cw_drv_spiReadBurstReg(uint8_t adr, uint8_t* data, uint8_t cnt, RFM69CW_drvInterfaceHandle_t interface);
extern RFM69CW_Return_t rfm69cw_drv_spiWriteBurstReg(uint8_t adr, uint8_t* data, uint8_t cnt, RFM69CW_drvInterfaceHandle_t interface);
#endif //USE_BURST_OPERATIONS

#else //of USE_SPI_REGISTER_FUNCTIONS
#define READ_REG    readReg
#define WRITE_REG   writeReg
#ifdef USE_BURST_OPERATIONS
#define READ_BURST_REG  readBurstReg
#define WRITE_BURST_REG  writeBurstReg
#endif //USE_BURST_OPERATIONS
//This function must be defined by the driver
//Function must block until all data is sent / answer received
//Incoming data is put back to pdata
//This is possible because the data reception always occurs after data item already
//sent and not needed in buffer any more
//Timeout is possible. Return error then!
extern RFM69CW_Return_t rfm69cw_drv_spiTrx(uint8_t* pdata, uint8_t cnt, RFM69CW_drvInterfaceHandle_t interface);
//This function must be defined by the driver
//Function may block until all data is sent or new function call must wait until
//last operation is complete
//If it does not block, all data from pdata must be copied until function leaves
//Timeout is possible. Return error then!
//Incoming data is discarded
#ifdef USE_TXONLY_FUNCTION
extern RFM69CW_Return_t rfm69cw_drv_spiTx(uint8_t* pdata, uint8_t cnt, RFM69CW_drvInterfaceHandle_t interface);
#endif //USE_TXONLY_FUNCTION
#endif //USE_SPI_REGISTER_FUNCTIONS



//oder so ähnlich Bits 0...5 <-> DIO0...5
extern RFM69CW_Return_t rfm69cw_drv_getDIO(uint8_t* dioData);

//Set callback-function for DIOxx interrupt
//RFM69CW_Return_t rfm69cw_drv_setDIOxxcb(void* cbf);

//Set callback-function for other interrupts...
//RFM69CW_Return_t rfm69cw_drv_setDIO3cb(void* cbf);


#ifdef USE_SPIDRV_REGISTER_FUNCTIONS
//no writeReg / readReg / writeBurstReg / readBurstReg functions necessary -> directly implemented by driver
#else //of USE_SPIDRV_REGISTER_FUNCTIONS

//maybe also add non-blocking (no OS) equivalents for use in real-time code
//These functions must immediately exit with error,
//if the specific amount of data cannot be written
static RFM69CW_Return_t writeReg(uint8_t adr, uint8_t data, RFM69CW_drvInterfaceHandle_t interface){
	uint8_t dataBufOut[2];
	RFM69CW_Return_t ret;

	dataBufOut[0] = 0x80 | adr;  //set "r/w"-bit to one (write)
	dataBufOut[1] = data;
#ifdef USE_TXONLY_FUNCTION
	ret = rfm69cw_drv_spiTx(dataBufOut, 2, interface);
#else
	ret = rfm69cw_drv_spiTrx(dataBufOut, 2, interface);
#endif

	return(ret);
}

static RFM69CW_Return_t readReg(uint8_t adr, uint8_t* data, RFM69CW_drvInterfaceHandle_t interface){
	uint8_t dataBuf[2];
	RFM69CW_Return_t ret;

	dataBuf[0] = 0x7F & adr; //reset "r/w"-bit to zero (read)
	//dataBuf[1] = 0x00;  //dummy -> Filled with data

	ret = rfm69cw_drv_spiTrx(dataBuf, 2, interface);
	if(ret) return(ret);

	*data = dataBuf[1];
	return(RFM69CW_Return_ERROR_NONE);
}

#ifdef USE_BURST_OPERATIONS
//Burst write register
static RFM69CW_Return_t writeBurstReg(uint8_t adr, uint8_t* data, uint8_t cnt, RFM69CW_drvInterfaceHandle_t interface){
	if(cnt > 254) return(RFM69CW_Return_ERROR_UNSPEC);

	RFM69CW_Return_t ret;

	//Waste of stack memory here!! -> Better use USE_SPIDRV_REGISTER_FUNCTIONS
	uint8_t dataBufOut[cnt+1];
	dataBufOut[0] = 0x80 | adr;  //set "r/w"-bit to one (write)
	//Copy data to stack
	uint8_t i;
	for(i=0; i<cnt;i++){
		dataBufOut[i+1] = data[i];
	}

#ifdef USE_TXONLY_FUNCTION
	ret = rfm69cw_drv_spiTx(dataBufOut, cnt+1, interface);
#else
	ret = rfm69cw_drv_spiTrx(dataBufOut, cnt+1, interface);
#endif

	return(ret);
}

//Burst read register
static RFM69CW_Return_t readBurstReg(uint8_t adr, uint8_t* data, uint8_t cnt, RFM69CW_drvInterfaceHandle_t interface){
	if(cnt > 254) return(RFM69CW_Return_ERROR_UNSPEC);

	RFM69CW_Return_t ret;

	//Waste of stack memory here!! -> Better use USE_SPIDRV_REGISTER_FUNCTIONS
	uint8_t dataBufOut[cnt+1];
	dataBufOut[0] = 0x7F & adr;  //reset "r/w"-bit to zero (read)

	ret = rfm69cw_drv_spiTrx(dataBufOut, cnt+1, interface);
	if(ret) return(ret);

	//Copy data from stack
	uint8_t i;
	for(i=0; i<cnt;i++){
		data[i] = dataBufOut[i+1];
	}

	return(RFM69CW_Return_ERROR_NONE);
}
#endif //USE_BURST_OPERATIONS

#endif //else of USE_SPIDRV_REGISTER_FUNCTIONS


/**
 * @brief Initialize RFM69CW handle.
 *
 * Should be called with uninitialized handle for each RFM69CW Module
 * @param handle RFM69CW handle.
 * @param ifHandle User-defined interface handle.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_Init(RFM69CW_Handle_t* handle, RFM69CW_drvInterfaceHandle_t ifHandle){
	handle->ifHandle = ifHandle;
	//Also Databuffers etc. must be initialized here

	return(RFM69CW_Return_ERROR_NONE);
}

/**
 * @brief Initialize common functionality of RFM69CW.
 *
 * @param handle RFM69CW handle.
 * @param RFM69CW_CommonInitStruct Structure with values used for initialization.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_CommonInit(RFM69CW_Handle_t* handle, RFM69CW_CommonInit_t* RFM69CW_CommonInitStruct){
	RFM69CW_Return_t ret;
	uint8_t regValTmp;

	//Init Modulation
	regValTmp = 0x00;
	regValTmp |= (RFM69CW_CommonInitStruct->RFM69CW_DataMode << 5);
	regValTmp |= (RFM69CW_CommonInitStruct->RFM69CW_ModulationType << 3);
	regValTmp |= (RFM69CW_CommonInitStruct->RFM69CW_ModulationShaping << 0);
	ret = WRITE_REG(REG_DataModul, regValTmp, handle->ifHandle);
	if(ret) return ret;

	//Init Bitrate
	ret = WRITE_REG(REG_BitrateMsb, (uint8_t) ((RFM69CW_CommonInitStruct->RFM69CW_BitRate & 0xFF00) >> 0x08), handle->ifHandle);
	if(ret) return ret;
	ret = WRITE_REG(REG_BitrateLsb, (uint8_t) ((RFM69CW_CommonInitStruct->RFM69CW_BitRate & 0x00FF) >> 0x00), handle->ifHandle);
	if(ret) return ret;

	//Init Frequency Deviation
	ret = WRITE_REG(REG_FdevMsb, (uint8_t) ((RFM69CW_CommonInitStruct->RFM69CW_FreqDev & 0xFF00) >> 0x08), handle->ifHandle);
	if(ret) return ret;
	ret = WRITE_REG(REG_FdevLsb, (uint8_t) ((RFM69CW_CommonInitStruct->RFM69CW_FreqDev & 0x00FF) >> 0x00), handle->ifHandle);
	if(ret) return ret;

	//Init RF Frequency
	ret = WRITE_REG(REG_FrfMsb, (uint8_t) ((RFM69CW_CommonInitStruct->RFM69CW_RfFreq & 0x00FF0000) >> 0x10), handle->ifHandle);
	if(ret) return ret;
	ret = WRITE_REG(REG_FrfMid, (uint8_t) ((RFM69CW_CommonInitStruct->RFM69CW_RfFreq & 0x0000FF00) >> 0x08), handle->ifHandle);
	if(ret) return ret;
	ret = WRITE_REG(REG_FrfLsb, (uint8_t) ((RFM69CW_CommonInitStruct->RFM69CW_RfFreq & 0x000000FF) >> 0x00), handle->ifHandle);
	if(ret) return ret;

	return(RFM69CW_Return_ERROR_NONE);
}

/**
 * @brief Initialize transmitter of RFM69CW.
 *
 * @param handle RFM69CW handle.
 * @param RFM69CW_TxInitStruct Structure with values used for initialization.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_TxInit(RFM69CW_Handle_t* handle, RFM69CW_TxInit_t* RFM69CW_TxInitStruct){
	RFM69CW_Return_t ret;
	uint8_t regValTmp;

	//PA0 on, set Output Power
	regValTmp = 0x80 + (RFM69CW_TxInitStruct->RFM69CW_OutputPower & 0x1F);
	ret = WRITE_REG(REG_PaLevel, regValTmp , handle->ifHandle);
	if(ret) return ret;

	//PA Ramping
	regValTmp = RFM69CW_TxInitStruct->RFM69CW_PaRamp & 0x0F;
	ret = WRITE_REG(REG_PaRamp, regValTmp, handle->ifHandle);
	if(ret) return ret;

	//Overload current protection
	regValTmp = 0x00;
	regValTmp |= (RFM69CW_TxInitStruct->RFM69CW_Ocp & 0x01) << 4;
	regValTmp |= (RFM69CW_TxInitStruct->RFM69CW_OcpCurrent & 0x0F) << 0;
	ret = WRITE_REG(REG_Ocp, regValTmp, handle->ifHandle);
	if(ret) return ret;

	return(RFM69CW_Return_ERROR_NONE);
}

/**
 * @brief Initialize receiver of RFM69CW.
 *
 * @param handle RFM69CW handle.
 * @param RFM69CW_RxInitStruct Structure with values used for initialization.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_RxInit(RFM69CW_Handle_t* handle, RFM69CW_RxInit_t* RFM69CW_RxInitStruct){
	RFM69CW_Return_t ret;
	uint8_t regValTmp;

	//LNA
	regValTmp = 0x00;
	regValTmp |= (RFM69CW_RxInitStruct->RFM69CW_LnaZin & 0x01) << 7;
	regValTmp |= (RFM69CW_RxInitStruct->RFM69CW_LnaGainSelect & 0x03) << 0;
	ret = WRITE_REG(REG_Lna, regValTmp, handle->ifHandle);
	if(ret) return ret;

	//DC cutoff and Channel filter bandwidth
	regValTmp = 0x00;
	regValTmp |= (RFM69CW_RxInitStruct->RFM69CW_DccFreq & 0x07) << 5;
	regValTmp |= RFM69CW_RxInitStruct->RFM69CW_RxBw;
	ret = WRITE_REG(REG_RxBw, regValTmp, handle->ifHandle);
	if(ret) return ret;

	//DC cutoff and Channel filter bandwidth during AFC
	regValTmp = 0x00;
	regValTmp |= (RFM69CW_RxInitStruct->RFM69CW_DccFreqAfc & 0x07) << 5;
	regValTmp |= RFM69CW_RxInitStruct->RFM69CW_RxBwAfc;
	ret = WRITE_REG(REG_AfcBw, regValTmp, handle->ifHandle);
	if(ret) return ret;

	//Rssi Threshold
	ret = WRITE_REG(REG_RssiThresh, RFM69CW_RxInitStruct->RFM69CW_RssiThresh, handle->ifHandle);
	if(ret) return ret;

	return(RFM69CW_Return_ERROR_NONE);
}

/**
 * @brief Initialize packet engine of RFM69CW.
 *
 * @param handle RFM69CW handle.
 * @param RFM69CW_PacketEngineInitStruct Structure with values used for initialization.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_PacketEngineInit(RFM69CW_Handle_t* handle, RFM69CW_PacketEngineInit_t* RFM69CW_PacketEngineInitStruct){
	RFM69CW_Return_t ret;
	uint8_t regValTmp;
	uint8_t i;

	// *** Preamble ***
	regValTmp = (uint8_t) ((RFM69CW_PacketEngineInitStruct->RFM69CW_PreambleSize & 0xFF00) >> 0x08);
	ret = WRITE_REG(REG_PreambleMsb, regValTmp, handle->ifHandle);
	if(ret) return ret;
	regValTmp = (uint8_t) ((RFM69CW_PacketEngineInitStruct->RFM69CW_PreambleSize & 0x00FF) >> 0x00);
	ret = WRITE_REG(REG_PreambleLsb, regValTmp, handle->ifHandle);
	if(ret) return ret;

	// *** Sync ***
	regValTmp = 0x00;
	if(RFM69CW_PacketEngineInitStruct->RFM69CW_SyncWordSize > 0){
		regValTmp |= 1<<7; //Enable Sync word generation
		regValTmp |= ((RFM69CW_PacketEngineInitStruct->RFM69CW_SyncWordSize - 1) & 0x07) << 3;
		regValTmp |= RFM69CW_PacketEngineInitStruct->RFM69CW_SyncTol;
	}
	//Remark: FifoFillCondition is needed, when no sync word!
	//From the documentation:
	//On the Rx side the data processing features like
	//Address filtering, Manchester encoding and data whitening are not available if the sync pattern length is set to zero
	//(SyncOn = 0). The filling of the FIFO in this case can be controlled by the bit FifoFillCondition.
	//So: Use functions RFM69CW_SetFifoFill / RFM69CW_ResetFifoFill
	ret = WRITE_REG(REG_SyncConfig, regValTmp, handle->ifHandle);
	if(ret) return ret;

	//Write REG_SyncValue acc. to SyncWordSize
	for(i = 0; i < RFM69CW_PacketEngineInitStruct->RFM69CW_SyncWordSize; i++){
		//(REG_SyncValue1 + i)
		ret = WRITE_REG(REG_SyncValue1 + i, RFM69CW_PacketEngineInitStruct->RFM69CW_SyncValue[i] , handle->ifHandle);
		if(ret) return ret;
	}

	// *** Packet Configuration ***
	regValTmp = 0x00;
	regValTmp |= RFM69CW_PacketEngineInitStruct->RFM69CW_PacketFormat     << 7;
	regValTmp |= RFM69CW_PacketEngineInitStruct->RFM69CW_DcFree           << 5;
	regValTmp |= RFM69CW_PacketEngineInitStruct->RFM69CW_Crc              << 4;
	regValTmp |= RFM69CW_PacketEngineInitStruct->RFM69CW_CrcAutoClear     << 3;
	regValTmp |= RFM69CW_PacketEngineInitStruct->RFM69CW_AddressFiltering << 1;
	ret = WRITE_REG(REG_PacketConfig1, regValTmp, handle->ifHandle);
	if(ret) return ret;
	regValTmp = 0x00;
	regValTmp |= RFM69CW_PacketEngineInitStruct->RFM69CW_InterPacketRxDelay << 4;
	regValTmp |= RFM69CW_PacketEngineInitStruct->RFM69CW_AutoRxRestart      << 1;
	regValTmp |= RFM69CW_PacketEngineInitStruct->RFM69CW_Aes                << 0;
	ret = WRITE_REG(REG_PacketConfig2, regValTmp, handle->ifHandle);
	if(ret) return ret;

	// *** Payload length ***
	ret = WRITE_REG(REG_PayloadLength, RFM69CW_PacketEngineInitStruct->RFM69CW_PayloadLength, handle->ifHandle);
	if(ret) return ret;


	// *** Node and Broadcast address ***
	ret = WRITE_REG(REG_NodeAdrs, RFM69CW_PacketEngineInitStruct->RFM69CW_NodeAddress, handle->ifHandle);
	if(ret) return ret;
	ret = WRITE_REG(REG_BroadcastAdrs, RFM69CW_PacketEngineInitStruct->RFM69CW_BroadcastAddress, handle->ifHandle);
	if(ret) return ret;

	// *** Fifo Threshold and Tx Start ***
	regValTmp = 0x00;
	regValTmp |= RFM69CW_PacketEngineInitStruct->RFM69CW_TxStartCondition << 7;
	regValTmp |= RFM69CW_PacketEngineInitStruct->RFM69CW_FifoThreshold    << 0;
	ret = WRITE_REG(REG_FiFoThresh, regValTmp, handle->ifHandle);
	if(ret) return ret;

	return(RFM69CW_Return_ERROR_NONE);
}

/**
 * @brief Initialize common init structure to default values.
 *
 * @param RFM69CW_CommonInitStruct Structure to initialize.
 */
void RFM69CW_CommonStructInit(RFM69CW_CommonInit_t* RFM69CW_CommonInitStruct){
	RFM69CW_CommonInitStruct->RFM69CW_Sequencer = RFM69CW_Sequencer_On;
	RFM69CW_CommonInitStruct->RFM69CW_DataMode = RFM69CW_DataMode_Packet;
	RFM69CW_CommonInitStruct->RFM69CW_ModulationType = RFM69CW_ModulationType_FSK;
	RFM69CW_CommonInitStruct->RFM69CW_ModulationShaping = RFM69CW_ModulationShaping_NONE;
	RFM69CW_CommonInitStruct->RFM69CW_RfFreq = RFM69CW_TO_RFFREQ(433.3000e6);
	RFM69CW_CommonInitStruct->RFM69CW_FreqDev = RFM69CW_TO_FDEV(2400);
	RFM69CW_CommonInitStruct->RFM69CW_BitRate = RFM69CW_TO_BITRATE(1200);
}

/**
 * @brief Initialize transmitter init structure to default values.
 *
 * @param RFM69CW_TxInitStruct Structure to initialize.
 */
void RFM69CW_TxStructInit(RFM69CW_TxInit_t* RFM69CW_TxInitStruct){
	RFM69CW_TxInitStruct->RFM69CW_OutputPower = RFM69CW_OutputPower_1mW;
	RFM69CW_TxInitStruct->RFM69CW_PaRamp = RFM69CW_PaRamp_40us;
	RFM69CW_TxInitStruct->RFM69CW_Ocp = RFM69CW_Ocp_On;
	RFM69CW_TxInitStruct->RFM69CW_OcpCurrent = RFM69CW_OcpCurrent_95mA;
}

/**
 * @brief Initialize receiver init structure to default values.
 *
 * @param RFM69CW_RxInitStruct Structure to initialize.
 */
void RFM69CW_RxStructInit(RFM69CW_RxInit_t* RFM69CW_RxInitStruct){
	RFM69CW_RxInitStruct->RFM69CW_LnaZin = RFM69CW_LnaZin_200ohms;
	RFM69CW_RxInitStruct->RFM69CW_LnaGainSelect = RFM69CW_LnaGainSelect_AGC;
	RFM69CW_RxInitStruct->RFM69CW_DccFreq = RFM69CW_DccFreq_4p;
	RFM69CW_RxInitStruct->RFM69CW_RxBw = RFM69CW_RxBw_FSK_10_4kHz;
	RFM69CW_RxInitStruct->RFM69CW_DccFreqAfc = RFM69CW_DccFreq_1p;
	RFM69CW_RxInitStruct->RFM69CW_RxBwAfc = RFM69CW_RxBw_FSK_50_0kHz;
	RFM69CW_RxInitStruct->RFM69CW_RssiThresh = 0xE4; //=228 -> -114dBm (=chip default)
}

/**
 * @brief Initialize packet engine init structure to default values.
 *
 * @param RFM69CW_PacketEngineInitStruct Structure to initialize.
 */
void RFM69CW_PacketEngineStructInit(RFM69CW_PacketEngineInit_t* RFM69CW_PacketEngineInitStruct){
	RFM69CW_PacketEngineInitStruct->RFM69CW_PreambleSize = 3;
	RFM69CW_PacketEngineInitStruct->RFM69CW_SyncWordSize = 4;
	RFM69CW_PacketEngineInitStruct->RFM69CW_SyncTol = 0;
	RFM69CW_PacketEngineInitStruct->RFM69CW_SyncValue[0] = 0x01;
	RFM69CW_PacketEngineInitStruct->RFM69CW_SyncValue[1] = 0x01;
	RFM69CW_PacketEngineInitStruct->RFM69CW_SyncValue[2] = 0x01;
	RFM69CW_PacketEngineInitStruct->RFM69CW_SyncValue[3] = 0x01;
	RFM69CW_PacketEngineInitStruct->RFM69CW_SyncValue[4] = 0x01;
	RFM69CW_PacketEngineInitStruct->RFM69CW_SyncValue[5] = 0x01;
	RFM69CW_PacketEngineInitStruct->RFM69CW_SyncValue[6] = 0x01;
	RFM69CW_PacketEngineInitStruct->RFM69CW_SyncValue[7] = 0x01;
	RFM69CW_PacketEngineInitStruct->RFM69CW_PacketFormat = RFM69CW_PacketFormat_Fixed;
	RFM69CW_PacketEngineInitStruct->RFM69CW_DcFree = RFM69CW_DcFree_None;
	RFM69CW_PacketEngineInitStruct->RFM69CW_Crc = RFM69CW_Crc_Off;
	RFM69CW_PacketEngineInitStruct->RFM69CW_CrcAutoClear = RFM69CW_CrcAutoClear_On;
	RFM69CW_PacketEngineInitStruct->RFM69CW_AddressFiltering = 0x00;
	RFM69CW_PacketEngineInitStruct->RFM69CW_PayloadLength = 0x40;
	RFM69CW_PacketEngineInitStruct->RFM69CW_NodeAddress = 0x00;
	RFM69CW_PacketEngineInitStruct->RFM69CW_BroadcastAddress = 0x00;
	RFM69CW_PacketEngineInitStruct->RFM69CW_TxStartCondition = RFM69CW_TxStartCondition_FifoNotEmpty;
	RFM69CW_PacketEngineInitStruct->RFM69CW_FifoThreshold = 15;
	RFM69CW_PacketEngineInitStruct->RFM69CW_InterPacketRxDelay = RFM69CW_InterPacketRxDelay_1bit;
	RFM69CW_PacketEngineInitStruct->RFM69CW_AutoRxRestart = RFM69CW_AutoRxRestart_On;
	RFM69CW_PacketEngineInitStruct->RFM69CW_Aes = RFM69CW_Aes_Off;
}

/**
 * @brief Write single byte to FIFO.
 *
 * @param handle RFM69CW handle.
 * @param data Data byte.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_FifoWrite(RFM69CW_Handle_t* handle, uint8_t data){
	RFM69CW_Return_t ret;
	ret = WRITE_REG(REG_Fifo, data, handle->ifHandle);
	return(ret);
}

/**
 * @brief Read single byte from FIFO.
 *
 * @param handle RFM69CW handle.
 * @param data Pointer to location, where data byte should be written
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_FifoRead(RFM69CW_Handle_t* handle, uint8_t* data){
	RFM69CW_Return_t ret;
	ret = READ_REG(REG_Fifo, data, handle->ifHandle);
	return(ret);
}

#ifdef USE_BURST_OPERATIONS
/**
 * @brief Write multiple bytes to FIFO.
 *
 * @param handle RFM69CW handle.
 * @param data Pointer to location, where data bytes should be read from
 * @cnt Number of bytes to be written.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_FifoBurstWrite(RFM69CW_Handle_t* handle, uint8_t* data, uint8_t cnt){
	RFM69CW_Return_t ret;
	ret = WRITE_BURST_REG(REG_Fifo, data, cnt, handle->ifHandle);
	return(ret);
}

/**
 * @brief Read multiple bytes from FIFO.
 *
 * @param handle RFM69CW handle.
 * @param data Pointer to location, where data bytes should be written
 * @cnt Number of bytes to be read.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_FifoBurstRead(RFM69CW_Handle_t* handle, uint8_t* data, uint8_t cnt){
	RFM69CW_Return_t ret;
	ret = READ_BURST_REG(REG_Fifo, data, cnt, handle->ifHandle);
	return(ret);
}
#else //of USE_BURST_OPERATIONS

//Burst operations are emulated by sequence of "normal" operations
#ifdef EMULATE_BURST_OPERATIONS
/**
 * @brief Write multiple bytes to FIFO.
 *
 * Emulated burst write. Consecutive single FIFO write operations are done.
 * @param handle RFM69CW handle.
 * @param data Pointer to location, where data bytes should be read from
 * @cnt Number of bytes to be written.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_FifoBurstWrite(RFM69CW_Handle_t* handle, uint8_t* data, uint8_t cnt){
	RFM69CW_Return_t ret;
	uint8_t i;

	for(i=0;i<cnt;i++){
		ret = WRITE_REG(REG_Fifo, data[i], handle->ifHandle);
		if(ret != RFM69CW_Return_ERROR_NONE) return(ret);
	}

	return(RFM69CW_Return_ERROR_NONE);
}

/**
 * @brief Read multiple bytes from FIFO.
 *
 * Emulated burst read. Consecutive single FIFO read operations are done.
 * @param handle RFM69CW handle.
 * @param data Pointer to location, where data bytes should be written
 * @cnt Number of bytes to be read.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_FifoBurstRead(RFM69CW_Handle_t* handle, uint8_t* data, uint8_t cnt){
	RFM69CW_Return_t ret;
	uint8_t i;

	for(i=0;i<cnt;i++){
		ret = READ_REG(REG_Fifo, &data[i], handle->ifHandle);
		if(ret != RFM69CW_Return_ERROR_NONE) return(ret);
	}

	return(RFM69CW_Return_ERROR_NONE);
}

#endif //EMULATE_BURST_OPERATIONS
#endif //USE_BURST_OPERATIONS


/**
 * @brief Go to <b>sleep</b> mode
 *
 * Enabled blocks: None
 * @param handle RFM69CW handle.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_GoSleep(RFM69CW_Handle_t* handle){
	//todo: initialize and store bits 7,6 in handle and do not change here
	//Alternative: -Read register. Create define to select (Speed/EEPROM vs. RAM usage)
	//             -bits 7,6, are constant (per define) (fastest, but not configuration possible)
	RFM69CW_Return_t ret;
	ret = WRITE_REG(REG_OpMode, 0x00, handle->ifHandle);
	return(ret);
}

/**
 * @brief Go to <b>standby</b> mode
 *
 * Enabled blocks: Top regulator and crystal oscillator
 * @param handle RFM69CW handle.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_GoStdby(RFM69CW_Handle_t* handle){
	//todo: initialize and store bits 7,6 in handle and do not change here
	RFM69CW_Return_t ret;
	ret = WRITE_REG(REG_OpMode, 0x04, handle->ifHandle);
	return(ret);
}

/**
 * @brief Go to <b>FS</b> mode
 *
 * Enabled blocks: Frequency synthesizer
 * @param handle RFM69CW handle.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_GoFs(RFM69CW_Handle_t* handle){
	//todo: initialize and store bits 7,6 in handle and do not change here
	RFM69CW_Return_t ret;
	ret = WRITE_REG(REG_OpMode, 0x08, handle->ifHandle);
	return(ret);
}

/**
 * @brief Go to <b>Transmit</b> mode
 *
 * Enabled blocks: Frequency synthesizer and transmitter
 * @param handle RFM69CW handle.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_GoTx(RFM69CW_Handle_t* handle){
	//todo: initialize and store bits 7,6 in handle and do not change here
	RFM69CW_Return_t ret;
	ret = WRITE_REG(REG_OpMode, 0x0C, handle->ifHandle);
	return(ret);
}

/**
 * @brief Go to <b>Receive</b> mode
 *
 * Enabled blocks: Frequency synthesizer and receiver
 * @param handle RFM69CW handle.
 * @return RFM69CW_Return_ERROR_NONE if no error occurs.
 */
RFM69CW_Return_t RFM69CW_GoRx(RFM69CW_Handle_t* handle){
	//todo: initialize and store bits 7,6 in handle and do not change here
	RFM69CW_Return_t ret;
	ret = WRITE_REG(REG_OpMode, 0x10, handle->ifHandle);
	return(ret);
}

RFM69CW_Return_t RFM69CW_SetFifoFill(RFM69CW_Handle_t* handle){
	//todo: Can this alternately be done without register read? (see RFM69CW_GoSleep ff.)
	RFM69CW_Return_t ret;
	uint8_t regVal;
	ret = READ_REG(REG_SyncConfig, &regVal, handle->ifHandle);
	regVal |= 1<<6;
	ret = WRITE_REG(REG_SyncConfig, regVal, handle->ifHandle);
	return(ret);
}


RFM69CW_Return_t RFM69CW_ResetFifoFill(RFM69CW_Handle_t* handle){
	//todo: Can this alternately be done without register read? (see RFM69CW_GoSleep ff.)
	RFM69CW_Return_t ret;
	uint8_t regVal;
	ret = READ_REG(REG_SyncConfig, &regVal, handle->ifHandle);
	regVal &= ~(1<<6);
	ret = WRITE_REG(REG_SyncConfig, regVal, handle->ifHandle);
	return(ret);
}


/**
 * @brief Determine, whether <b>SyncAddressMatch</b> flag is set
 *
 * Set when Sync and Address (if enabled) are detected. Cleared when leaving Rx or FIFO is emptied.
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsSyncAddressMatch(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags1, &regData, handle->ifHandle);
	if(ret) return(ret);

	if(regData & (1<<0))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Determine, whether <b>AutoMode</b> flag is set
 *
 * Set when entering Intermediate mode. Cleared when exiting Intermediate mode. Please note that in Sleep mode a small delay can be observed between AutoMode interrupt and the corresponding enter/exit condition
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsAutoMode(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags1, &regData, handle->ifHandle);
	if(ret) return(ret);

	if(regData & (1<<1))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Determine, whether <b>Timeout</b> flag is set.
 *
 * Set, when a timeout occurs (see TimeoutRxStart and TimeoutRssiThresh). Cleared when leaving Rx or FIFO is emptied.
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsTimeout(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags1, &regData, handle->ifHandle);
	if(ret) return(ret);

	if(regData & (1<<2))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}



/**
 * @brief Determine, whether <b>Rssi</b> flag is set.
 *
 * <p>Set in Rx when RssiValue exceeds RssiThreshold. Cleared when leaving Rx.</p>
 * <p>Remark by F.M.: Seems also to be cleared when packet reception is complete and going back to WAIT mode</p>
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsRssi(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags1, &regData, handle->ifHandle);
	if(ret) return(ret);

	if(regData & (1<<3))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}


/**
 * @brief Determine, whether <b>PllLock</b> flag is set.
 *
 * Set (in FS, Rx or Tx) when the PLL is locked. Cleared when it is not.
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsPllLock(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags1, &regData, handle->ifHandle);
	if(ret) return(ret);

	if(regData & (1<<4))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Determine, whether <b>TxReady</b> flag is set.
 *
 * Set in Tx mode, after PA ramp-up. Cleared when leaving Tx.
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsTxReady(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags1, &regData, handle->ifHandle);
	if(ret) return(ret);

	if(regData & (1<<5))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}


/**
 * @brief Determine, whether <b>RxReady</b> flag is set.
 *
 * Set in Rx mode, after RSSI, AGC and AFC. Cleared when leaving Rx.
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsRxReady(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags1, &regData, handle->ifHandle);
	if(ret) return(ret);

	if(regData & (1<<6))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Determine, whether <b>ModeReady</b> flag is set.
 *
 * Set when the operation mode requested by RFM69CW_GoXXX, is ready
 * <ul>
 * <li>Sleep: Entering Sleep mode</li>
 * <li>Standby: XO is running</li>
 * <li>FS: PLL is locked</li>
 * <li>Rx: RSSI sampling starts</li>
 * <li>Tx: PA ramp-up completed</li>
 * <li>Cleared when changing operating mode</li>
 * </ul>
 */
RFM69CW_Return_t RFM69CW_IsModeReady(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags1, &regData, handle->ifHandle);
	if(ret) return(ret);

	if(regData & (1<<7))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Determine, whether <b>CrcOk</b> flag is set.
 *
 * Set in Rx when the CRC of the payload is Ok. Cleared when FIFO is empty.
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsCrcOk(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags2, &regData, handle->ifHandle);
	if(ret) return(ret);

	if(regData & (1<<1))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Determine, whether <b>PayloadReady</b> flag is set.
 *
 * Set in Rx when the payload is ready
 * (i.e. last byte received and CRC, if enabled
 * and CrcAutoClearOff is cleared, is Ok).
 * Cleared when FIFO is empty.
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsPayloadReady(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags2, &regData, handle->ifHandle);
	if(ret) return(-1);

	if(regData & (1<<2))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Determine, whether <b>PacketSent</b> flag is set.
 *
 * Set in Tx when the complete packet has been sent. Cleared when exiting Tx.
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsPacketSent(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags2, &regData, handle->ifHandle);
	if(ret) return(-1);

	if(regData & (1<<3))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Determine, whether <b>FifoOverrun</b> flag is set.
 *
 * Set when FIFO overrun occurs (except in Sleep mode).
 * Flag(s) and FIFO are cleared when this bit is set.
 * The FIFO then becomes immediately available for the next
 * transmission / reception
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsFifoOverrun(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags2, &regData, handle->ifHandle);
	if(ret) return(-1);

	if(regData & (1<<4))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Determine, whether <b>FifoLevel</b> flag is set.
 *
 * Set when the number of bytes in the FIFO strictly exceeds
 * FifoThreshold, else cleared
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsFifoLevel(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags2, &regData, handle->ifHandle);
	if(ret) return(-1);

	if(regData & (1<<5))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Determine, whether <b>FifoNotEmpty</b> flag is set.
 *
 * Set when FIFO contains at least one byte, else cleared
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsFifoNotEmpty(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags2, &regData, handle->ifHandle);
	if(ret) return(-1);

	if(regData & (1<<6))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Determine, whether <b>FifoFull</b> flag is set.
 *
 * Set when FIFO is full (i.e. contains 66 bytes), else cleared
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsFifoFull(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_IrqFlags2, &regData, handle->ifHandle);
	if(ret) return(-1);

	if(regData & (1<<7))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}


/**
 * @brief Determine, whether <b>RssiDone</b> bit is set.
 *
 * <p>0->RSSI is on-going, 1->RSSI sampling is finished, result available</p>
 * <p>Remark by F.M.: This function seems only to be useful after "user triggering" RSSI (by @ref RFM69CW_StartRssi).
 * Otherwise (to check, whether "normal" RSSI at beginning of RX has finished), use @ref RFM69CW_IsRssi </p>
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_TRUE, when set, RFM69CW_Return_FALSE otherwise.
 */
RFM69CW_Return_t RFM69CW_IsRssiDone(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_RssiConfig, &regData, handle->ifHandle);
	if(ret) return(ret);

	if(regData & (1<<1))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

/**
 * @brief Trigger a RSSI measurement
 *
 * @param handle RFM69CW Handle.
 * @return RFM69CW_Return_ERROR_NONE, if no error occurs.
 */
RFM69CW_Return_t RFM69CW_StartRssi(RFM69CW_Handle_t* handle){
	RFM69CW_Return_t ret;
	ret = WRITE_REG(REG_RssiConfig, 0x01, handle->ifHandle);
	return(ret);
}

/**
 * @brief Get RSSI detected value
 *
 * @param handle RFM69CW Handle.
 * @param rssi Pointer to location, where result will be written. It is \f$ l_{\mbox{p}} = \mbox{rssi} \times -0.5 \mbox{dBm}  \f$
 * @return RFM69CW_Return_ERROR_NONE, if no error occurs.
 */
RFM69CW_Return_t RFM69CW_GetRssiValue(RFM69CW_Handle_t* handle, uint8_t* rssi){
	RFM69CW_Return_t ret;
	ret = READ_REG(REG_RssiValue, rssi, handle->ifHandle);
	return(ret);
}

RFM69CW_Return_t RFM69CW_IsFeiDone(RFM69CW_Handle_t* handle){
	uint8_t regData;
	RFM69CW_Return_t ret;

	ret = READ_REG(REG_AfcFei, &regData, handle->ifHandle);
	if(ret) return(ret);

	if(regData & (1<<6))
		return(RFM69CW_Return_TRUE);

	return(RFM69CW_Return_FALSE);
}

RFM69CW_Return_t RFM69CW_StartFei(RFM69CW_Handle_t* handle){
	RFM69CW_Return_t ret;
	ret = WRITE_REG(REG_AfcFei, 0x20, handle->ifHandle);
	return(ret);
}

RFM69CW_Return_t RFM69CW_GetFeiValue(RFM69CW_Handle_t* handle, int16_t* fei){
	RFM69CW_Return_t ret;
	uint8_t regval;

	ret = READ_REG(REG_FeiMsb, &regval, handle->ifHandle);
	if(ret) return ret;
	*fei = regval * 0x0100;

	ret = READ_REG(REG_FeiLsb, &regval, handle->ifHandle);
	if(ret) return ret;
	*fei += regval;

	return(RFM69CW_Return_ERROR_NONE);
}

RFM69CW_Return_t RFM69CW_GetLnaCurrentGain(RFM69CW_Handle_t* handle, uint8_t* gain){
	RFM69CW_Return_t ret;
	ret = READ_REG(REG_Lna, gain, handle->ifHandle);
	*gain = (*gain >> 3) & 0x07;
	return(ret);
}

RFM69CW_Return_t RFM69CW_GetVersion(RFM69CW_Handle_t* handle, uint8_t* versionNo){
	RFM69CW_Return_t ret;
	ret = READ_REG(REG_Version, versionNo, handle->ifHandle);
	return(ret);
}

//todo: Can this alternately be done without register read? (see RFM69CW_GoSleep ff.)
RFM69CW_Return_t RFM69CW_ForceRestartRx(RFM69CW_Handle_t* handle){
	RFM69CW_Return_t ret;
	uint8_t regVal;
	ret = READ_REG(REG_PacketConfig2, &regVal, handle->ifHandle);
	regVal |= 1<<2;
	ret = WRITE_REG(REG_PacketConfig2, regVal, handle->ifHandle);
	return(ret);
}

#ifdef INCLUDE_SETRFFREQ_FUNCTION
//RF frequency setter
//-> also integrated to Common configuration, only needed when freq. needs to be changed "online"
RFM69CW_Return_t RFM69CW_SetRfFreq(RFM69CW_Handle_t* handle, uint32_t rfFreq){
	RFM69CW_Return_t ret=0;

	//todo: Limitation to allowed values! (Module and legal regulations!)
	ret |= WRITE_REG(REG_FrfMsb, (uint8_t) ((rfFreq & 0x00FF0000) >> 0x10), handle->ifHandle);
	ret |= WRITE_REG(REG_FrfMid, (uint8_t) ((rfFreq & 0x0000FF00) >> 0x08), handle->ifHandle);
	ret |= WRITE_REG(REG_FrfLsb, (uint8_t) ((rfFreq & 0x000000FF) >> 0x00), handle->ifHandle);
	return(ret);
}
#endif


//RSSI trigger level for Rssi interrupt: -rssiThresh / 2 *dBm
//-> also integrated to Rx configuration, only needed when threshold needs to be changed "online"
#ifdef INCLUDE_SETRSSITHRESHOLD_FUNCTION
RFM69CW_Return_t RFM69CW_SetRssiThreshold(RFM69CW_Handle_t* handle, uint8_t rssiThresh){
	RFM69CW_Return_t ret;
	ret = WRITE_REG(REG_RssiThresh, rssiThresh, handle->ifHandle);
	return(ret);
}
#endif

#ifdef INCLUDE_SETPAYLOADLENGTH_FUNCTION
//For fixed format: 0=unlimited, >0=length. For variable format: Not used for Tx, max length in Rx
//-> also integrated to PacketEngine configuration, only needed when payload length needs to be changed "online"
RFM69CW_Return_t RFM69CW_SetPayloadLength(RFM69CW_Handle_t* handle, uint8_t payloadLength){
	RFM69CW_Return_t ret;
	ret = WRITE_REG(REG_PayloadLength, payloadLength, handle->ifHandle);
	return(ret);
}
#endif //INCLUDE_SETPAYLOADLENGTH_FUNCTION

#ifdef INCLUDE_SPECIAL_FREQ_CALC_FUNCTIONS
//chanNo must be btw. 1-69
RFM69CW_Return_t RFM69CW_SetRfFreqIsm433Channel(RFM69CW_Handle_t* handle, uint8_t chanNo){
	//todo: PERFORM CALCULATION WITHOUT FLOAT
	RFM69CW_Return_t ret;// = 0;

	float rfFreq = 433.050f + 0.025f*(float)chanNo;
	rfFreq *= 1e6f;
	ret = RFM69CW_SetRfFreq(handle, RFM69CW_CalcRfFreq(rfFreq));
	return(ret);
}

//chanNo must be btw. 1-08
RFM69CW_Return_t RFM69CW_SetRfFreqPMRChannel(RFM69CW_Handle_t* handle, uint8_t chanNo){
	//todo: PERFORM CALCULATION WITHOUT FLOAT
	RFM69CW_Return_t ret;// = 0;

	float rfFreq = 446.00625 + 0.0125f*(float)(chanNo-1);
	rfFreq *= 1e6f;
	ret = RFM69CW_SetRfFreq(handle, RFM69CW_CalcRfFreq(rfFreq));
	return(ret);
}
#endif //INCLUDE_SPECIAL_FREQ_CALC_FUNCTIONS

#ifdef INCLUDE_FLOAT_CALC_FUNCTIONS
//Calculate RF frequency register value out of frequency in Hz
uint32_t RFM69CW_CalcRfFreq(float rfFreqHz){
	uint32_t val_Frf = (rfFreqHz / FSTEP  + 0.5f);  //+0.5f is for correct rounding

	return(val_Frf);
}

//Calculate frequency deviation register value out of frequency in Hz
uint16_t RFM69CW_CalcFdev(float FdevHz){
	uint16_t val_Fdev = (FdevHz / FSTEP  + 0.5f);  //+0.5f is for correct rounding

	return(val_Fdev);
}

uint16_t RFM69CW_CalcBitrate(float bitrate){
	uint16_t val_Bitrate = ((FXOSC / bitrate) + 0.5f);  //+0.5f is for correct rounding

	return(val_Bitrate);
}

//Calculate RSSI in dBm out of register value
float RFM69CW_CalcRssi(uint8_t val_rssi){
	float rssi = val_rssi*-0.5f;

	return(rssi);
}
#endif //INCLUDE_FLOAT_CALC_FUNCTIONS

#ifdef INCLUDE_REGISTER_DUMP_FUNCTION
RFM69CW_Return_t RFM69CW_RegisterDump(RFM69CW_Handle_t* handle, uint8_t* memory){
	//todo: Go through all registers (0x01-0x4F and some others) and output to memory array
	return(RFM69CW_Return_ERROR_NONE);
}
#endif
