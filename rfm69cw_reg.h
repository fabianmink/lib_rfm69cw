/*
  Copyright (c) 2014, Fabian Mink <fabian.mink@gmx.de>
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
 * @file rfm69cw_reg.h
 * @author Fabian Mink
 * @date 2014-12-17
 * @brief RFM69CW Driver Register definition
 * @copyright BSD 2-Clause License
 */

#define REG_Fifo       0x00

#define REG_OpMode     0x01

#define REG_DataModul  0x02

#define REG_BitrateMsb 0x03
#define REG_BitrateLsb 0x04

#define REG_FdevMsb    0x05
#define REG_FdevLsb    0x06

#define REG_FrfMsb     0x07
#define REG_FrfMid     0x08
#define REG_FrfLsb     0x09

#define REG_Osc1       0x0A
#define REG_AfcCtrl    0x0B

#define REG_Listen1    0x0D
#define REG_Listen2    0x0E
#define REG_Listen3    0x0F

#define REG_Version    0x10

#define REG_PaLevel    0x11
#define REG_PaRamp     0x12
#define REG_Ocp        0x13

#define REG_Lna        0x18
#define REG_RxBw       0x19

#define REG_AfcBw      0x1A

#define REG_AfcFei     0x1E
#define REG_AfcMsb     0x1F
#define REG_AfcLsb     0x20
#define REG_FeiMsb     0x21
#define REG_FeiLsb     0x22

#define REG_RssiConfig 0x23
#define REG_RssiValue  0x24

#define REG_DioMapping1 0x25
#define REG_DioMapping2 0x26

#define REG_IrqFlags1  0x27
#define REG_IrqFlags2  0x28

#define REG_RssiThresh  0x29
#define REG_RxTimeout1  0x2A
#define REG_RxTimeout2  0x2B

#define REG_PreambleMsb 0x2C
#define REG_PreambleLsb 0x2D

#define REG_SyncConfig  0x2E
#define REG_SyncValue1  0x2F
#define REG_SyncValue2  0x30
#define REG_SyncValue3  0x31
#define REG_SyncValue4  0x32
#define REG_SyncValue5  0x33
#define REG_SyncValue6  0x34
#define REG_SyncValue7  0x35
#define REG_SyncValue8  0x36

#define REG_PacketConfig1 0x37
#define REG_PayloadLength 0x38
#define REG_NodeAdrs      0x39
#define REG_BroadcastAdrs 0x3A
#define REG_AutoModes     0x3B
#define REG_FiFoThresh    0x3C
#define REG_PacketConfig2 0x3D

#define REG_TestLna  0x58
#define REG_TestPa1  0x5A
#define REG_TestPa2  0x5C

#define REG_TestDagc  0x6F
