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
 * @file rfm69cw_config.h
 * @author Fabian Mink
 * @date 2015-01-29
 * @brief RFM69CW Driver Configuration
 * @copyright BSD 2-Clause License
 *
 * Config File to be changed for user requirements
 *
 */

#ifndef __RFM69CW_CONFIG_H
#define __RFM69CW_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>


#define FXOSC   32e6

//Directly implement functions for RFM69CW register read/write
//Omits one function call and use of stack resources. Use it whenever possible!
//Functions to be implemented:
//extern RFM69CW_Return_t rfm69cw_drv_spiReadReg(uint8_t adr, uint8_t* data, RFM69CW_drvInterfaceHandle_t interface);
//extern RFM69CW_Return_t rfm69cw_drv_spiWriteReg(uint8_t adr, uint8_t data, RFM69CW_drvInterfaceHandle_t interface);
#define USE_SPIDRV_REGISTER_FUNCTIONS

//Use burst read/writes for fifo read/write
//Functions to be implemented:
//extern RFM69CW_Return_t rfm69cw_drv_readBurstReg(uint8_t adr, uint8_t* data, uint8_t cnt, RFM69CW_drvInterfaceHandle_t interface);
//extern RFM69CW_Return_t rfm69cw_drv_readBurstReg(uint8_t adr, uint8_t* data, uint8_t cnt, RFM69CW_drvInterfaceHandle_t interface);
#define USE_BURST_OPERATIONS

//If functions above are not implemented, burst operations can be emulated by subsequent "normal" read/write operations
//#define EMULATE_BURST_OPERATIONS


//Use burst read/writes also for other operations (NOT IMPLEMENTED YET!)
//Functions not necessary any more:
//extern RFM69CW_Return_t rfm69cw_drv_spiReadReg(uint8_t adr, uint8_t* data, RFM69CW_drvInterfaceHandle_t interface);
//extern RFM69CW_Return_t rfm69cw_drv_spiWriteReg(uint8_t adr, uint8_t data, RFM69CW_drvInterfaceHandle_t interface);
//#define USE_ONLY_BURST_OPERATIONS

#define USE_TXONLY_FUNCTION

#define INCLUDE_FLOAT_CALC_FUNCTIONS
#define INCLUDE_SPECIAL_FREQ_CALC_FUNCTIONS
#define INCLUDE_SETRFFREQ_FUNCTION
#define INCLUDE_SETRSSITHRESHOLD_FUNCTION
#define INCLUDE_SETPAYLOADLENGTH_FUNCTION
#define INCLUDE_REGISTER_DUMP_FUNCTION

//User-defined typedef for the handle to the HW-Interface
//This define is used in the rfm69-Code in no way, except that it is stored in the
//RFM69CW Handle structure (RFM69CW_Handle_t) and passed to the HW-Interface driver functions
//when called
typedef int RFM69CW_drvInterfaceHandle_t;

//User-defined typedef for function return type
typedef int RFM69CW_Return_t;

#ifdef __cplusplus
}
#endif

#endif /* __RFM69CW_CONFIG_H */
