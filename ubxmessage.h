/*
 * ubxproto
 * Copyright (c) 2014, Alexey Edelev aka semlanik, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * Additionally to GNU Lesser General Public License you MUST NOT
 * static link this library to your application and MUST add link to author
 * and source of this library in your application.
 *
 * Actual LGPL text https://www.gnu.org/licenses/lgpl.html
 *
 * File: ubxmessage.h
 */
/*! \file */

#ifndef UBXMESSAGE_H
#define UBXMESSAGE_H

#include <stdio.h>

#ifdef __linux__
#include <sys/types.h>
#elif defined(_WIN32)
//Need to add other OS depend headers
#endif

/*
 * Need to clarify how many I/O ports has each version of UBLOX before define UBX_IO_PORTS_NUM macros
 * current proto vesion has hardcoded 6 I/O ports by default. If you have other I/O ports amount
 * please define macro manualy by yourself.
 */
#define UBX_IO_PORTS_NUM 6
//#if!defined UBLOX_VERSION
//#error Before using library define UBLOX_VERSION first
//#else
//#if UBLOX_VERSION == 5 || UBLOX_VERSION == 6
//#define UBX_IO_PORTS_NUM 6
//#else
//#define UBX_IO_PORTS_NUM 6
//#endif
//#endif

#if defined (__linux__)
typedef u_int8_t  UBXU1_t;
typedef int8_t    UBXI1_t;
typedef u_int8_t  UBXX1_t;
typedef u_int16_t UBXU2_t;
typedef int16_t   UBXI2_t;
typedef u_int16_t UBXX2_t;
typedef u_int32_t UBXU4_t;
typedef int32_t   UBXI4_t;
typedef u_int32_t UBXX4_t;
typedef float     UBXR4_t;
typedef double    UBXR8_t;
typedef char      UBXCH_t;
#else
typedef unsigned char  UBXU1_t;
typedef char           UBXI1_t;
typedef unsigned char  UBXX1_t;
typedef unsigned short UBXU2_t;
typedef short          UBXI2_t;
typedef unsigned short UBXX2_t;
typedef unsigned int   UBXU4_t;
typedef int            UBXI4_t;
typedef unsigned int   UBXX4_t;
typedef float          UBXR4_t;
typedef double         UBXR8_t;
typedef char           UBXCH_t;
#endif

static const int UBX_CHECKSUM_SIZE = 2;
static const int UBX_HEADER_SIZE = 6;
static const UBXU2_t UBX_PREAMBLE = 0xB562;

typedef enum
{
    UBXMsgClassNAV = 0x01,
    UBXMsgClassRXM = 0x02,
    UBXMsgClassINF = 0x04,
    UBXMsgClassACK = 0x05,
    UBXMsgClassCFG = 0x06,
    UBXMsgClassMON = 0x0A,
    UBXMsgClassAID = 0x0B,
    UBXMsgClassTIM = 0x0D,
    UBXMsgClassLOG = 0x21,
    UBXMsgClassInvalid = 255
} UBXMessageClass;

typedef enum
{
    UBXMsgIdACK_NACK = 0x00,
    UBXMsgIdACK_ACK = 0x01,

    UBXMsgIdAID_ALM = 0x30,
    UBXMsgIdAID_ALPSRV = 0x32,
    UBXMsgIdAID_ALP = 0x50,
    UBXMsgIdAID_AOP = 0x33,
    UBXMsgIdAID_DATA = 0x10,
    UBXMsgIdAID_EPH = 0x31,
    UBXMsgIdAID_HUI = 0x02,
    UBXMsgIdAID_INI = 0x01,
    UBXMsgIdAID_REQ = 0x00,

    UBXMsgIdCFG_ANT = 0x13,
    UBXMsgIdCFG_CFG = 0x09,
    UBXMsgIdCFG_DAT = 0x06,
    UBXMsgIdCFG_GNSS = 0x3E,
    UBXMsgIdCFG_INF = 0x02,
    UBXMsgIdCFG_ITFM = 0x39,
    UBXMsgIdCFG_LOGFILTER = 0x47,
    UBXMsgIdCFG_MSG = 0x01,
    UBXMsgIdCFG_NAV5 = 0x24,
    UBXMsgIdCFG_NAVX5 = 0x23,
    UBXMsgIdCFG_NMEA = 0x17,
    UBXMsgIdCFG_NVS = 0x22,
    UBXMsgIdCFG_PM2 = 0x3B,
    UBXMsgIdCFG_PRT = 0x00,
    UBXMsgIdCFG_RATE = 0x08,
    UBXMsgIdCFG_RINV = 0x34,
    UBXMsgIdCFG_RST = 0x04,
    UBXMsgIdCFG_RXM = 0x11,
    UBXMsgIdCFG_SBAS = 0x16,
    UBXMsgIdCFG_TP5 = 0x31,
    UBXMsgIdCFG_USB = 0x1B,

    UBXMsgIdINF_DEBUG = 0x04,
    UBXMsgIdINF_ERROR = 0x00,
    UBXMsgIdINF_NOTICE = 0x02,
    UBXMsgIdINF_TEST = 0x03,
    UBXMsgIdINF_WARNING = 0x01,

    UBXMsgIdLOG_CREATE = 0x07,
    UBXMsgIdLOG_ERASE = 0x03,
    UBXMsgIdLOG_FINDTIME = 0x0E,
    UBXMsgIdLOG_INFO = 0x08,
    UBXMsgIdLOG_RETRIEVEPOS = 0x0B,
    UBXMsgIdLOG_RETRIEVESTRING = 0x0D,
    UBXMsgIdLOG_RETRIEVE = 0x09,
    UBXMsgIdLOG_STRING = 0x04,

    UBXMsgIdMON_HW2 = 0x0B,
    UBXMsgIdMON_HW = 0x09,
    UBXMsgIdMON_IO = 0x02,
    UBXMsgIdMON_MSGPP = 0x06,
    UBXMsgIdMON_RXBUF = 0x07,
    UBXMsgIdMON_RXR = 0x21,
    UBXMsgIdMON_TXBUF = 0x08,
    UBXMsgIdMON_VER = 0x04,

    UBXMsgIdNAV_AOPSTATUS = 0x60,
    UBXMsgIdNAV_CLOCK = 0x22,
    UBXMsgIdNAV_DGPS = 0x31,
    UBXMsgIdNAV_DOP = 0x04,
    UBXMsgIdNAV_POSECEF = 0x01,
    UBXMsgIdNAV_POSLLH = 0x02,
    UBXMsgIdNAV_PVT = 0x07,
    UBXMsgIdNAV_SBAS = 0x32,
    UBXMsgIdNAV_SOL = 0x06,
    UBXMsgIdNAV_STATUS = 0x03,
    UBXMsgIdNAV_SVINFO = 0x30,
    UBXMsgIdNAV_TIMEGPS = 0x20,
    UBXMsgIdNAV_TIMEUTC = 0x21,
    UBXMsgIdNAV_VELECEF = 0x11,
    UBXMsgIdNAV_VELNED = 0x12,

    UBXMsgIdRXM_ALM = 0x30,
    UBXMsgIdRXM_EPH = 0x31,
    UBXMsgIdRXM_PMREQ = 0x41,
    UBXMsgIdRXM_RAW = 0x10,
    UBXMsgIdRXM_SFRB = 0x11,
    UBXMsgIdRXM_SVSI = 0x20,

    UBXMsgIdTIM_TM2 = 0x03,
    UBXMsgIdTIM_TP = 0x01,
    UBXMsgIdTIM_VRFY = 0x06,

    UBXMsgIdInvalid = 0xFF
} UBXMessageId;

typedef enum
{
    UBXHardwareReset = 0x00,
    UBXControlledReset = 0x01,
    UBXControlledResetGNSSOnly = 0x02,
    UBXHardwareResetAfterShutdown = 0x04,
    UBXControlledGNSSStop = 0x08,
    UBXControlledGNSSStart = 0x09
} UBXResetMode;

typedef enum
{
    UBXBBRHotstart = 0x0000,
    UBXBBRWarmstart = 0x0001,
    UBXBBRColdstart = 0xFFFF
} UBXBBRSpecialSets;

typedef enum
{
    UBXBBReph = 1,
    UBXBBRalm = 1 << 1,
    UBXBBRhealth = 1 << 2,
    UBXBBRklob = 1 << 3,
    UBXBBRpos = 1 << 4,
    UBXBBRclkd = 1 << 5,
    UBXBBRosc = 1 << 6,
    UBXBBRutc = 1 << 7,
    UBXBBRrtc = 1 << 8,
    UBXBBRsfdr = 1 << 11,
    UBXBBRvmon = 1 << 12,
    UBXBBRtct = 1 << 13,
    UBXBBRaop = 1 << 15
} UBXBBRMask;

typedef enum
{
    UBXHUIHealthValid = 1,
    UBXHUIUTCValid = 1 << 1,
    UBXHUIKlobValid = 1 << 2
} UBXHUIFlags;

typedef enum
{
    UBXINIfEdge = 1 << 1,
    UBXINItm1 = 1 << 4,
    UBXINIf1 = 1 << 6
} UBXINItmCfg;

typedef enum
{
    UBXINIpos = 1,
    UBXINItime = 1 << 1,
    UBXINIclockD = 1 << 2,
    UBXINItp = 1 << 3,
    UBXINIclockF = 1 << 4,
    UBXINIlla = 1 << 5,
    UBXINIaltInv = 1 << 6,
    UBXINIprevTm = 1 << 7,
    UBXINIutc = 1 << 10
} UBXINIFlags;

typedef enum
{
    UBXANTsvcs = 1,
    UBXANTscd = 1 << 1,
    UBXANTocd = 1 << 2,
    UBXANTpdwnOnSCD = 1 << 3,
    UBXANTrecovery = 1 << 4
} UBXANTFlags;

typedef enum
{
    //CFG_CFG message
    UBXCFGioPort = 1,
    UBXCFGmsgConf = 1 << 1,
    UBXCFGinfMsg = 1 << 2,
    UBXCFGnavConf = 1 << 3,
    UBXCFGrxmConf = 1 << 4,
    UBXCFGrinvConf = 1 << 9,
    UBXCFGantConf = 1 << 10,
    //CFG_NVS message
    UBXCFGalm = 1 << 17,
    UBXCFGaopConf = 1 << 29
} UBXCFGMask;

typedef enum
{
    UBXCFGdevBBR = 1,
    UBXCFGdevFlash = 1 << 1,
    UBXCFGdevEEPROM = 1 << 2,
    UBXCFGdevSpiFlash = 1 << 5
} UBXCFGDeviceMask;

typedef enum
{
    UBXCFGTimepulse = 0,
    UBXCFGTimepulse2 = 1
} UBXCFGTimepulses;

typedef enum
{
    UBXCFGTimepulseActive = 1,
    UBXCFGTimepulseLockGpsFreq = 1 << 1,
    UBXCFGTimepulseLockedOtherSet = 1 << 2,
    UBXCFGTimepulseIsFreq = 1 << 3,
    UBXCFGTimepulseIsLenght = 1 << 4,
    UBXCFGTimepulseAlignToTow = 1 << 5,
    UBXCFGTimepulsePolarity = 1 << 6,
    UBXCFGTimepulseGridUTSGPS = 1 << 7
} UBXCFGTimepulseFlags;

typedef enum
{
    UBXProtocol = 0,
    UBXNMEAProtocol
} UBXCFGProtocolIds;

typedef enum
{
    UBXGPS,
    UBXSBAS,
    UBXQZSS = 5,
    UBXGLONASS = 6
} UBXGNSSIds;

typedef enum
{
    UBXInfError = 1,
    UBXInfWarning = 1 << 1,
    UBXInfNotice = 1 << 2,
    UBXInfDebug = 1 << 3,
    UBXInfTest = 1 << 4
} UBXCFGInfMsgMask;

typedef enum
{
    UBXITFMAntUnknown = 0,
    UBXITFMAntPassive = 1,
    UBXITFMAntActive = 2
} UBXITFMAntSetting;

typedef enum
{
    UBXLOGFILTERRecordEnabled = 1,
    UBXLOGFILTERPsmOncePerWakupEnabled = 1 << 1,
    UBXLOGFILTERApplyAllFilterSettings = 1 << 2
} UBXLOGFILTERFlags;


typedef enum
{
    UBXNAV5Dyn = 1,
    UBXNAV5MinEl = 1 << 1,
    UBXNAV5PosFixMode = 1 << 2,
    UBXNAV5DrLim = 1 << 3,
    UBXNAV5PosMask = 1 << 4,
    UBXNAV5TimeMask = 1 << 5,
    UBXNAV5StaticHoldMask = 1 << 6,
    UBXNAV5DgpsMask = 1 << 7,
} UBXNAV5Mask;

typedef enum
{
    UBXNAV5ModelPortable = 0,
    UBXNAV5ModelStationary = 2,
    UBXNAV5ModelPedestrian = 3,
    UBXNAV5ModelAutomotive = 4,
    UBXNAV5ModelSea = 5,
    UBXNAV5ModelAirborne1g = 6,
    UBXNAV5ModelAirborne2g = 7,
    UBXNAV5ModelAirborne4g = 8
} UBXNAV5Model;

typedef enum
{
    UBXNAV5Fix2DOnly = 1,
    UBXNAV5Fix3DOnly = 2,
    UBXNAV5FixAuto2D3D = 3
} UBXNAV5FixMode;

typedef enum
{
    UBXNAVX5AopMinMax = 1 << 2,
    UBXNAVX5AopMinCno = 1 << 3,
    UBXNAVX5AopInitial3dfix = 1 << 6,
    UBXNAVX5AopWknRoll = 1 << 9,
    UBXNAVX5AopPPP = 1 << 13,
    UBXNAVX5Aop = 1 << 14
} UBXNAVX5Mask;

typedef enum
{
    UBXNMEAPosFilter = 1,
    UBXNMEAMskPosFilter = 1 << 1,
    UBXNMEATimeFilter = 1 << 2,
    UBXNMEADateFilter = 1 << 3,
    UBXNMEAGPSOnlyFilter = 1 << 4,
    UBXNMEATrackFilter = 1 << 5,
} UBXNMEAFilter;

typedef enum
{
    UBXNMEAVersion23 = 0x23,
    UBXNMEAVersion21 = 0x21
} UBXNMEAVersion;

typedef enum
{
    UBXNMEACompatFlag = 1,
    UBXNMEAConsiderFlag = 1 << 1
} UBXNMEAFlags;

typedef enum
{
    UBXNMEAGPSFilter = 1,
    UBXNMEASBASFilter = 1 << 1,
    UBXNMEAQZSSFilter = 1 << 4,
    UBXNMEAGLONASSFilter = 1 << 5
} UBXNMEAGNSSToFilter;

typedef enum
{
    UBXNMEASVNumStrict = 0,
    UBXNMEASVNumExtended = 1
} UBXNMEASVNumbering;

typedef enum
{
    UBXNMEATalkerNotOverriden = 0,
    UBXNMEATalkerGP = 1,
    UBXNMEATalkerGL = 2,
    UBXNMEATalkerGN = 3
} UBXNMEATalkerIds;

typedef enum
{
    UBXNMEAGSVTalkerGNSSSpecific = 0,
    UBXNMEAGSVTalkerMain = 1
} UBXNMEAGSVTalkerIds;

typedef enum
{
    UBXPM2LimitCurrentDisabled = 0x00,
    UBXPM2LimitCurrentEnabled = 0x01
} UBXPM2LimitPeakCurrent;

typedef enum
{
    UBXPM2OnOffOperation = 0x00,
    UBXPM2CyclicTrackOperation = 0x01
} UBXPM2Mode;


typedef enum
{
    UBXPRTMode5BitCharLen = 0x00, //Not supported
    UBXPRTMode6BitCharLen = 0x01, //Not supported
    UBXPRTMode7BitCharLen = 0x02, //Supported only with parity
    UBXPRTMode8BitCharLen = 0x03
} UBXPRTModeCharLen;

typedef enum
{
    UBXPRTModeEvenParity = 0,
    UBXPRTModeOddParity = 1,
    UBXPRTModeNoParity = 1 << 3,
    UBXPRTModeReserved = 1 << 2
} UBXPRTModeParity;

typedef enum
{
    UBXPRTMode1StopBit = 0,
    UBXPRTMode1dot5StopBit = 1,
    UBXPRTMode2StopBit = 2,
    UBXPRTMode0dot5StopBit = 3,
} UBXPRTModeStopBits;

typedef enum
{
    UBXPRTInProtoInUBX = 1,
    UBXPRTInProtoInNMEA = 1 << 1,
    UBXPRTInProtoInRTCM = 1 << 2
} UBXPRTInProtoMask;

typedef enum
{
    UBXPRTOutProtoOutUBX = 1,
    UBXPRTOutProtoOutNMEA = 1 << 1
} UBXPRTOutProtoMask;

typedef enum
{
    UBXPRTExtendedTxTimeout = 1 << 1
} UBXPRTFlags;

typedef enum
{
    UBXPRTSPIMode0 = 0, //CPOL = 0, CPHA = 0
    UBXPRTSPIMode1, //CPOL = 0, CPHA = 1
    UBXPRTSPIMode2, //CPOL = 1, CPHA = 0
    UBXPRTSPIMode3, //CPOL = 1, CPHA = 1
} UBXPRTSPIMode;

typedef enum
{
    UBXRINVDump = 1,
    UBXRINVBinary = 1 << 1
} UBXRINVFlags;

typedef enum
{
    UBXRXMContinousMode = 0,
    UBXRXMPowerSaveMode = 1
} UBXRXMLowPowerModes;

typedef enum
{
    UBXSBASModeEnabled = 1,
    UBXSBASModeTest = 1 << 1
} UBXSBASModes;

typedef enum
{
    UBXSBASUsageRange = 1,
    UBXSBASUsageDiffCorr = 1 << 1,
    UBXSBASUsageIntegrity = 1 << 2
} UBXSBASUsage;

typedef enum
{
    UBXSBASScanModePRN152 = 1,
    UBXSBASScanModePRN153 = 1 << 1,
    UBXSBASScanModePRN154 = 1 << 2,
    UBXSBASScanModePRN155 = 1 << 3,
    UBXSBASScanModePRN156 = 1 << 4,
    UBXSBASScanModePRN157 = 1 << 5,
    UBXSBASScanModePRN158 = 1 << 6
} UBXSBASScanModes2;

typedef enum
{
    UBXSBASScanModePRN120 = 1,
    UBXSBASScanModePRN121 = 1 << 1,
    UBXSBASScanModePRN122 = 1 << 2,
    UBXSBASScanModePRN123 = 1 << 3,
    UBXSBASScanModePRN124 = 1 << 4,
    UBXSBASScanModePRN125 = 1 << 5,
    UBXSBASScanModePRN126 = 1 << 6,
    UBXSBASScanModePRN127 = 1 << 7,
    UBXSBASScanModePRN128 = 1 << 8,
    UBXSBASScanModePRN129 = 1 << 9,
    UBXSBASScanModePRN130 = 1 << 10,
    UBXSBASScanModePRN131 = 1 << 11,
    UBXSBASScanModePRN132 = 1 << 12,
    UBXSBASScanModePRN133 = 1 << 13,
    UBXSBASScanModePRN134 = 1 << 14,
    UBXSBASScanModePRN135 = 1 << 15,
    UBXSBASScanModePRN136 = 1 << 16,
    UBXSBASScanModePRN137 = 1 << 17,
    UBXSBASScanModePRN138 = 1 << 18,
    UBXSBASScanModePRN139 = 1 << 19,
    UBXSBASScanModePRN140 = 1 << 20,
    UBXSBASScanModePRN141 = 1 << 21,
    UBXSBASScanModePRN142 = 1 << 22,
    UBXSBASScanModePRN143 = 1 << 23,
    UBXSBASScanModePRN144 = 1 << 24,
    UBXSBASScanModePRN145 = 1 << 25,
    UBXSBASScanModePRN146 = 1 << 26,
    UBXSBASScanModePRN147 = 1 << 27,
    UBXSBASScanModePRN148 = 1 << 28,
    UBXSBASScanModePRN149 = 1 << 29,
    UBXSBASScanModePRN150 = 1 << 30,
    UBXSBASScanModePRN151 = 1 << 31,
} UBXSBASScanModes1;

typedef enum
{
    USBFlagReEnum = 1,
    USBFlagPowerMode = 1 << 1
} UBXUSBFlags;

typedef enum
{
    UBXLOGCfgCircular = 1
} UBXLOGCfg;

typedef enum
{
    UBXLOGMaximumSafeSize = 0,
    UBXLOGMinimunSize = 1,
    UBXLOGUserDefined = 2,
} UBXLOGSize;

typedef enum
{
    UBXLOGStatusRecording = 1 << 3,
    UBXLOGStatusInactive = 1 << 4,
    UBXLOGStatusCircular = 1 << 5
} UBXLOGStatus;

typedef enum
{
    UBXRETRIEVEPOS2DFix = 2,
    UBXRETRIEVEPOS3DFix = 3
} UBXRETRIEVEPOSFixType;

typedef enum
{
    UBXRXRAwake = 1
} UBXRXRFlags;

typedef enum
{
    UBXAOPStatusIdle = 0,
    UBXAOPStatusRunning = 1
} UBXAOPStatus;

typedef enum
{
    UBXAOPCfgUseAOP = 1
} UBXAOPCfg;

typedef enum
{
    UBXGPSNoFix = 0x00,
    UBXGPSDeadReckoning = 0x01,
    UBXGPS2DFix = 0x02,
    UBXGPS3DFix = 0x03,
    UBXGPSGNSSDeadReckoning = 0x04,
    UBXGPSTimeOnlyFix = 0x05
} UBXGPSFix;

typedef enum
{
    UBXPVTValidDate = 1,
    UBXPVTValidTime = 1 << 1,
    UBXPVTFullyResolved = 1 << 2,
} UBXPVTValid;

typedef enum
{
    UBXPVTPSMStateNA = 0,
    UBXPVTPSMStateEnabled = 1,
    UBXPVTPSMStateAcquisition = 2,
    UBXPVTPSMStateTracking = 3,
    UBXPVTPSMStatePowerOptim = 4,
    UBXPVTPSMStateInactive = 5,

} UBXPVTPSMStates;

typedef enum
{
    UBXSBASServiceRanging = 1,
    UBXSBASServiceCorrections = 1 << 1,
    UBXSBASServiceIntegrity = 1 << 2,
    UBXSBASServiceTestmode = 1 << 3
} UBXSBASService;

typedef enum
{
    UBXSBASSOLGPSfixOK = 1,
    UBXSBASSOLDiffSoln = 1 << 1,
    UBXSBASSOLWKNSet = 1 << 2,
    UBXSBASSOLTOWSet = 1 << 3
} UBXSBASSOLFlags;

typedef enum
{
    UBXSVINFOAntarisChip = 0,
    UBXSVINFOUBlox5Chip = 1,
    UBXSVINFOUBlox6Chip = 2
} UBXSVINFOChipGen;

typedef enum
{
    UBXSVINFOFlagsSVUsed = 1,
    UBXSVINFOFlagsDiffCorr = 1 << 1,
    UBXSVINFOFlagsOrbitAvail = 1 << 2,
    UBXSVINFOFlagsOrbitEph = 1 << 3,
    UBXSVINFOFlagsUnhealthy = 1 << 4,
    UBXSVINFOFlagsOrbitAlm = 1 << 5,
    UBXSVINFOFlagsOrbitAop = 1 << 6,
    UBXSVINFOFlagsSmoothed = 1 << 7
} UBXSVINFOFlags;

typedef enum
{
    UBXSVINFOQualityChannelIdle = 0,
    UBXSVINFOQualityChannelSearching = 1,
    UBXSVINFOQualitySignalAquired = 2,
    UBXSVINFOQualitySignalDetected = 3,
    UBXSVINFOQualityCodeLockOnSignal = 4,
    UBXSVINFOQualityCodeCarrierLocked = 5
} UBXSVINFOQualityId;

typedef enum
{
    UBXTIMEGPSTowValid = 1,
    UBXTIMEGPSWeekValid = 1 << 1,
    UBXTIMEGPSLeapSValid = 1 << 2
} UBXTIMEGPSValidityFlags;

typedef enum
{
    UBXTIMEUTCValidTOW = 1,
    UBXTIMEUTCValidWKN = 1 << 1,
    UBXTIMEUTCValidUTC = 1 << 2
} UBXTIMEUTCValidityFlags;

typedef enum
{
    UBXPMREQBackup = 1 << 1
} UBXPMREQFlags;

typedef enum
{
    UBXTM2FlagsModeSingle = 0,
    UBXTM2FlagsModeRunning = 1
} UBXTM2FlagsMode;

typedef enum
{
    UBXTM2FlagsRunArmed = 0,
    UBXTM2FlagsRunStopped = 1
} UBXTM2FlagsRun;

typedef enum
{
    UBXTM2FlagsTimeBaseReceiverTime = 0,
    UBXTM2FlagsTimeBaseGPS = 1,
    UBXTM2FlagsTimeBaseUTC = 2
} UBXTM2FlagsTimeBase;

typedef enum
{
    UBXTM2FlagsUTCNotAvailable = 0,
    UBXTM2FlagsUTCAvailable = 1
} UBXTM2FlagsUTC;

typedef enum
{
    UBXTM2FlagsTimeInvalid = 0,
    UBXTM2FlagsTimeValid = 1
} UBXTM2FlagsTime;

typedef enum
{
    UBXTPTimeBaseUTC = 1,
    UBXTPUTCAvailable = 1 << 1
} UBXTPFlags;

typedef enum
{
    UBXVRFYNoTimeAidingDone = 0,
    UBXVRFYSourceRTC = 2,
    UBXVRFYSourceAID_INI = 3
} UBXVRFYFlagsSource;

#pragma pack(push,1)

typedef struct {
    UBXU1_t msgClass;
    UBXU1_t msgId;
    UBXU2_t length;
} UBXHdr;

typedef struct {
    UBXU1_t idSize;
    UBXU1_t type;
    UBXU2_t offset;
    UBXU2_t size;
    UBXU2_t fileId;
    UBXU2_t dataSize;
    UBXU1_t id1;
    UBXU1_t id2;
    UBXU4_t id3;
} UBXAID_ALPSRV;

typedef struct {
    UBXU1_t msgClass;
    UBXU1_t msgId;
} UBXACK_ACK;

typedef struct {
    UBXU1_t msgClass;
    UBXU1_t msgId;
} UBXACK_NACK;

//typedef struct UBXAID_ALM_POLL {
    //No payload
//};

typedef struct {
    UBXU1_t svid;
} UBXAID_ALM_POLL_OPT;

typedef struct {
    UBXU4_t svid;
    UBXU4_t week;
} UBXAID_ALM;

typedef struct {
    UBXU4_t svid;
    UBXU4_t week;
    UBXU4_t dwrd[8];
} UBXAID_ALM_OPT;

typedef struct {
    //Variable payload
#ifdef __WINDOWS__
    UBXU1_t payload;
#endif
} UBXAID_ALP;

typedef struct {
    UBXU1_t dummy;
} UBXAID_ALP_END;

typedef struct {
    UBXU4_t predTow;
    UBXU4_t predDur;
    UBXI4_t age;
    UBXU2_t predWno;
    UBXU2_t almWno;
    UBXU4_t reserved1;
    UBXU1_t svs;
    UBXU1_t reserved2;
    UBXU2_t reserved3;
} UBXAID_ALP_POLL;

//typedef struct {
    //No payload
//} UBXAID_AOP_POLL;

typedef struct {
    UBXU1_t svid;
} UBXAID_AOP_POLL_OPT;

typedef struct {
    UBXU1_t svid;
    UBXU1_t data[59];
} UBXAID_AOP;

typedef struct {
    UBXU1_t svid;
    UBXU1_t data[59];
    UBXU1_t optional0[48];
    UBXU1_t optional1[48];
    UBXU1_t optional2[48];
} UBXAID_AOP_OPT;

//typedef struct {
    //No payload
//} UBXAID_DATA_POLL;

//typedef struct UBXAID_EPH_POLL {
    //No payload
//};

typedef struct {
    UBXU1_t svid;
} UBXAID_EPH_POLL_OPT;

typedef struct {
    UBXU4_t svid;
    UBXU4_t how;
} UBXAID_EPH;

typedef struct {
    UBXU4_t svid;
    UBXU4_t how;
    UBXU4_t sf1d[8];
    UBXU4_t sf2d[8];
    UBXU4_t sf3d[8];
} UBXAID_EPH_OPT;

//typedef struct {
    //No payload
//} UBXAID_HUI_POLL;

typedef struct {
    UBXI4_t health;
    UBXR4_t utcA0;
    UBXR4_t utcA1;
    UBXI4_t utcTOW;
    UBXI2_t utcWNT;
    UBXI2_t utcLS;
    UBXI2_t utcWNF;
    UBXI2_t utcDN;
    UBXI2_t utcLSF;
    UBXI2_t utcSpare;
    UBXR4_t klobA0;
    UBXR4_t klobA1;
    UBXR4_t klobA2;
    UBXR4_t klobA3;
    UBXR4_t klobB0;
    UBXR4_t klobB1;
    UBXR4_t klobB2;
    UBXR4_t klobB3;
    UBXX2_t flags;
} UBXAID_HUI;

//typedef struct {
    //No payload
//} UBXAID_INI_POLL;

typedef struct {
    UBXI1_t ecefXOrLat;
    UBXI1_t ecefYOrLat;
    UBXI1_t ecefZOrLat;
    UBXU1_t posAcc;
    UBXI1_t tmCfg;
    UBXU2_t wnoOrDate;
    UBXU4_t towOrDate;
    UBXI4_t towNs;
    UBXU4_t tAccMS;
    UBXU4_t tAccNS;
    UBXI4_t clkDOrFreq;
    UBXU4_t clkDAccOrFreqAcc;
    UBXX4_t flags;
} UBXAID_INI;

//typedef struct {
    //No payload
//} UBXAID_REQ;

//typedef struct {
    //No payload
//} UBXCFG_ANT_POLL;

typedef struct {
    UBXX2_t UBXANTpinSwitch:5;
    UBXX2_t UBXANTpinSCD:5;
    UBXX2_t UBXANTpinOCD:5;
    UBXX2_t UBXANTreconfig:1;
} UBXANTPins;

typedef struct {
    UBXX2_t flags; //See UBXANTFlags to fill this field
    UBXANTPins pins;
} UBXCFG_ANT;

typedef struct {
    UBXX4_t clearMask; //See UBXCFGMask to fill this field
    UBXX4_t saveMask; //See UBXCFGMask to fill this field
    UBXX4_t loadMask; //See UBXCFGMask to fill this field
} UBXCFG_CFG;

typedef struct {
    UBXX4_t clearMask; //See UBXCFGMask to fill this field
    UBXX4_t saveMask; //See UBXCFGMask to fill this field
    UBXX4_t loadMask; //See UBXCFGMask to fill this field
    UBXX1_t deviceMask; //See UBXCFGDeviceMask to fill this field
} UBXCFG_CFG_OPT;

//typedef struct {
    //No payload
//} UBXCFG_DAT_POLL;

typedef struct {
    UBXR8_t majA;
    UBXR8_t flat;
    UBXR4_t dX;
    UBXR4_t dY;
    UBXR4_t dZ;
    UBXR4_t rotX;
    UBXR4_t rotY;
    UBXR4_t rotZ;
    UBXR4_t scale;
} UBXCFG_DAT_IN;

typedef struct {
    UBXU2_t datumNum;
    UBXCH_t datumName[6];
    UBXR8_t majA;
    UBXR8_t flat;
    UBXR4_t dX;
    UBXR4_t dY;
    UBXR4_t dZ;
    UBXR4_t rotX;
    UBXR4_t rotY;
    UBXR4_t rotZ;
    UBXR4_t scale;
} UBXCFG_DAT_OUT;

//typedef struct {
    //No payload
//} UBXCFG_GNSS_POLL;

typedef struct {
    UBXU1_t msgVer;
    UBXU1_t numTrkChHw;
    UBXU1_t numTrkChUse;
    UBXU1_t numConfigBlocks;
    //Variable addition here
    //See structure below
} UBXCFG_GNSS;

typedef struct {
    UBXU1_t gnssId; //See UBXGNSSIds to fill this field
    UBXU1_t resTrkCh;
    UBXU1_t maxTrkCh;
    UBXU1_t reserved1;
    UBXX4_t flags; //0 - disabled, 1 - enabled
} UBXCFG_GNSS_PART;

typedef struct {
    UBXU1_t protocolId;
} UBXCFG_INF_POLL;

typedef struct {
    //Variable payload
    //See structure UBXCFG_INF_PART below
#ifdef __WINDOWS__
    UBXU1_t payload;
#endif
} UBXCFG_INF;

typedef struct {
    UBXU1_t protocolId;
    UBXU1_t reserved0;
    UBXU2_t reserved1;
    UBXX1_t infMsgMask[6]; //See UBXCFGInfMsgMask to fill this field
} UBXCFG_INF_PART;

//typedef struct {
    //No payload
//} UBXCFG_ITFM_POLL;

typedef struct
{
    UBXX4_t bbThreshold:4;
    UBXX4_t cwThreshold:5;
    UBXX4_t reserved1:22; //Should be 0x16B156
    UBXX4_t enbled:1;
} UBXITFMConfig;

typedef struct
{
    UBXX4_t reserved2:12; //Should be 0x31E
    UBXX4_t antSetting:2; //See UBXITFMAntSetting to fill this field
    UBXX4_t reserved3:18; //Should be 0x00
} UBXITFMConfig2;

typedef struct {
    UBXITFMConfig config;
    UBXITFMConfig2 config2;
} UBXCFG_ITFM;

//typedef struct {
    //No payload data
//} UBXCFG_LOGFILTER_POLL;

typedef struct {
    UBXU1_t version;
    UBXX1_t flags; //See UBXLOGFILTERFlags to fill this field
    UBXU2_t minIterval;
    UBXU2_t timeThreshold;
    UBXU2_t speedThreshold;
    UBXU4_t positionThreshold;
} UBXCFG_LOGFILTER;

typedef struct {
    UBXU1_t msgClass;
    UBXU1_t msgId;
} UBXCFG_MSG_POLL;

typedef struct {
    UBXU1_t msgClass;
    UBXU1_t msgId;
    UBXU1_t rate[6];
} UBXCFG_MSG_RATES;

typedef struct {
    UBXU1_t msgClass;
    UBXU1_t msgId;
    UBXU1_t rate;
} UBXCFG_MSG_RATE;

//typedef struct {
    //No payload data
//} UBXCFG_NAV5_POLL;

typedef struct {
    UBXX2_t mask; //See UBXNAV5Mask to fill this field
    UBXU1_t dynModel; //See UBXNAV5Model to fill this field
    UBXU1_t fixMode; //See UBXNAV5FixMode to fill this field
    UBXI4_t fixedAlt;
    UBXU4_t fixedAltVar;
    UBXI1_t minElev;
    UBXU1_t drLimit;
    UBXU2_t pDop;
    UBXU2_t tDop;
    UBXU2_t pAcc;
    UBXU2_t tAcc;
    UBXU1_t staticHoldThresh;
    UBXU1_t dgpsTimeOut;
    UBXU1_t cnoThreshNumSVs;
    UBXU1_t cnoThresh;
    UBXU2_t reserved2; //Set to 0
    UBXU4_t reserved3; //Set to 0
    UBXU4_t reserved4; //Set to 0
} UBXCFG_NAV5;

//typedef struct {
    //No payload
//} UBXCFG_NAVX5_POLL;

typedef struct {
    UBXU2_t version;
    UBXX2_t mask1; //See UBXNAVX5Mask to fill this field
    UBXU4_t reserved0; //Set 0
    UBXU1_t reserved1; //Set 0
    UBXU1_t reserved2; //Set 0
    UBXU1_t minSVs;
    UBXU1_t maxSVs;
    UBXU1_t minCNO;
    UBXU1_t reserved5; //Set 0
    UBXU1_t iniFix3D;
    UBXU1_t reserved6; //Set 0
    UBXU1_t reserved7; //Set 0
    UBXU1_t reserved8; //Set 0
    UBXU2_t wknRollover;
    UBXU4_t reserved9; //Set 0
    UBXU1_t reserved10; //Set 0
    UBXU1_t reserved11; //Set 0
    UBXU1_t usePPP;
    UBXU1_t aopCFG; // 0-disabled, 1 - enabled
    UBXU1_t reserved12; //Set 0
    UBXU1_t reserved13; //Set 0
    UBXU1_t aopOrbMaxErr;
    UBXU1_t reserved14; //Set 0
    UBXU1_t reserved15; //Set 0
    UBXU2_t reserved3; //Set 0
    UBXU4_t reserved4; //Set 0
} UBXCFG_NAVX5;

//typedef struct UBXCFG_NMEA_POLL {
    //No payload
//};

typedef struct {
    UBXX1_t filter; //See UBXNMEAFilter to fill this field
    UBXU1_t nmeaVersion;
    UBXU1_t numSV;
    UBXX1_t flags; //See UBXNMEAFlags to fill this field
    UBXX4_t gnssToFilter; //See UBXNMEAGNSSToFilter to fill this field
    UBXU1_t svNumbering;
    UBXU1_t mainTalkerId;
    UBXU1_t gsvTalkerId;
    UBXU1_t reserved;
} UBXCFG_NMEA;

typedef struct {
    UBXX4_t clearMask; //See UBXCFGMask CFG_NVS section to fill this field
    UBXX4_t saveMask; //See UBXCFGMask CFG_NVS section to fill this field
    UBXX4_t loadMask; //See UBXCFGMask CFG_NVS section to fill this field
    UBXX1_t deviceMask; //See UBXCFGDeviceMask to fill this field
} UBXCFG_NVS;

//typedef struct {
    //No payload
//} UBXCFG_PM2_POLL;

typedef struct
{
    UBXX4_t blank1:1;
    UBXX4_t reserved:3;
    UBXX4_t extIntSelect:1;
    UBXX4_t extIntWake:1;
    UBXX4_t extIntBackup:1;
    UBXX4_t blank2:1;
    UBXX4_t limitPeakCurr:2; //See UBXPM2LimitPeakCurrent to fill this field
    UBXX4_t waitTimeFix:1;
    UBXX4_t updateRTC:1;
    UBXX4_t updateEPH:1;
    UBXX4_t blank3:3;
    UBXX4_t doNotEnterOff:1;
    UBXX4_t mode:2; //See UBXPM2Mode to fill this field
} UBXCFG_PM2Flags;

typedef struct {
    UBXU1_t version;
    UBXU1_t reserved1;
    UBXU1_t reserved2;
    UBXU1_t reserved3;
    UBXCFG_PM2Flags flags;
    UBXU4_t updatePeriod;
    UBXU4_t searchPeriod;
    UBXU4_t gridOffset;
    UBXU2_t onTime;
    UBXU2_t minAcqTime;
    UBXU2_t reserved4;
    UBXU2_t reserved5;
    UBXU4_t reserved6;
    UBXU4_t reserved7;
    UBXU1_t reserved8;
    UBXU1_t reserved9;
    UBXU2_t reserved10;
    UBXU4_t reserved11;
} UBXCFG_PM2;

//typedef struct {
    //No payload
//} UBXCFG_PRT_POLL;

typedef struct {
    UBXU1_t portId;
} UBXCFG_PRT_POLL_OPT;

typedef struct
{
    UBXX2_t en:1; //0 - disabled, 1 - enabled
    UBXX2_t pol:1; //0 - High-active, 1 - Low-active
    UBXX2_t pin:5;
    UBXX2_t thres:9; //Given value is multiplied by 8 bytes
} UBXCFG_PRTTxReady;

typedef struct
{
    UBXX4_t blank0:4;
    UBXX4_t reserved1:1;
    UBXX4_t blank1:1;
    UBXX4_t charLen:2; //See UBXPRTModeCharLen to fill this field
    UBXX4_t blank2:1;
    UBXX4_t parity:3; //See UBXPRTModeParity to fill this field
    UBXX4_t nStopBits:2; //See UBXPRTModeStopBits to fill this field
    UBXX4_t blank3:18;
} UBXCFG_PRTUARTMode;

typedef struct
{
    UBXX4_t blank0:1;
    UBXX4_t spiMode:2; //See UBXPRTSPIMode to fill this field
    UBXX4_t blank1:3;
    UBXX4_t flowControl:1; //0 - disabled, 1 - enabled
    UBXX4_t blank2:1;
    UBXX4_t ffCnt:8;
    UBXX4_t blank3:16;
} UBXCFG_PRTSPIMode;

typedef struct
{
    UBXX4_t blank0:1;
    UBXX4_t slaveAddr:7; //Range: 0x07 < slaveAddr < 0x78. Bit 0 shall be 0
    UBXX4_t blank1:24;
} UBXCFG_PRTDDCMode;

typedef union
{
    UBXCFG_PRTUARTMode UART;
    UBXCFG_PRTSPIMode SPI;
    UBXCFG_PRTDDCMode DDC;
    UBXX4_t USB; //reserved
} UBXCFG_PRTMode;

typedef union
{
    UBXU4_t UARTbaudRate;
    UBXU4_t OtherReserved;
} UBXCFG_PRT5Option;

typedef struct
{
    UBXU1_t portID;
    UBXU1_t reserved0;
    UBXCFG_PRTTxReady txReady;
    UBXCFG_PRTMode mode;
    UBXCFG_PRT5Option option;
    UBXX2_t inProtoMask; //See UBXPRTInProtoMask to fill this field
    UBXX2_t outProtoMask; //See UBXPRTOutProtoMask to fill this field
    UBXX2_t flags; //See UBXPRTFlags to fill this field, shall be 0 for USB
    UBXU1_t reserved5;
} UBXCFG_PRT;

//typedef struct {
    //No payload
//} UBXCFG_RATE_POLL;

typedef struct {
    UBXU2_t measRate;
    UBXU2_t navRate;
    UBXU2_t timeRef;
} UBXCFG_RATE;

//typedef struct {
    //No payload
//} UBXCFG_RINV_POLL;

typedef struct {
    UBXX1_t flags; //See UBXRINVFlags to fill this field
    //Variable payload size
} UBXCFG_RINV;

typedef struct {
    UBXX2_t navBBRMask;
    UBXU1_t resetMode;
    UBXU1_t reserved1;
} UBXCFG_RST;

//typedef struct {
    //No payload
//} UBXCFG_RXM_POLL;

typedef struct {
    UBXU1_t reserved1; //Shall be set to 8
    UBXU1_t lpMode; //See UBXRXMLowPowerModes to fill this field
} UBXCFG_RXM;

//typedef struct {
    //No payload
//} UBXCFG_SBAS_POLL;

typedef struct {
    UBXX1_t mode; //See UBXSBASModes to fill this field
    UBXX1_t usage; //See UBXSBASUsage to fill this field
    UBXU1_t maxSBAS;
    UBXX1_t scanmode2; //See UBXSBASScanModes2 to fill this field
    UBXX4_t scanmode1; //See UBXSBASScanModes1 to fill this field
} UBXCFG_SBAS;

//typedef struct {
    //No payload
//} UBXCFG_TP5_POLL;

typedef struct {
    UBXU1_t tpIdx;
} UBXCFG_TP5_POLL_OPT;

typedef struct {
    UBXU1_t tpIdx;
    UBXU1_t reserved0;
    UBXU2_t reserved1;
    UBXI2_t antCableDelay;
    UBXI2_t rfGroupDelay;
    UBXU4_t freqPeriod;
    UBXU4_t freqPeriodLock;
    UBXU4_t pulseLenRatio;
    UBXU4_t pulseLenRatioLock;
    UBXI4_t userConfigDelay;
    UBXX4_t flags; //See UBXCFGTimepulseFlags to fill this field
} UBXCFG_TP5;

//typedef struct {
    //No payload
//} UBXCFG_USB_POLL;

typedef struct {
    UBXU2_t vendorId;
    UBXU2_t productId;
    UBXU2_t reserved1;//Set to 0
    UBXU2_t reserved2;//Set to 1
    UBXU2_t powerConsumption;
    UBXX2_t flags;
    UBXCH_t vendorString[32];
    UBXCH_t productString[32];
    UBXCH_t serialNumber[32];
} UBXCFG_USB;

typedef struct {
#ifdef __WINDOWS__
    UBXCH_t payload;
#endif
    //Variable payload of UBXCH_t
} UBXINF_DEBUG;

typedef struct {
#ifdef __WINDOWS__
    UBXCH_t payload;
#endif
    //Variable payload of UBXCH_t
} UBXINF_ERROR;

typedef struct {
#ifdef __WINDOWS__
    UBXU1_t payload;
#endif
    //Variable payload of UBXCH_t
} UBXINF_NOTICE;

typedef struct {
#ifdef __WINDOWS__
    UBXU1_t payload;
#endif
    //Variable payload of UBXCH_t
} UBXINF_TEST;

typedef struct {
#ifdef __WINDOWS__
    UBXU1_t payload;
#endif
    //Variable payload of UBXCH_t
} UBXINF_WARNING;

typedef struct {
    UBXU1_t version;
    UBXX1_t logCfg; //See UBXLOGCfg
    UBXU1_t reserved; //Set to 0
    UBXU1_t logSize;
    UBXU4_t userDefinedSize;
} UBXLOG_CREATE;

//typedef struct UBXLOG_ERASE {
    //No payload
//};

typedef struct {
    UBXU1_t version; //Shall be 0
    UBXU1_t type; //Shall be 0
    UBXU2_t reserved1;
    UBXU2_t year;
    UBXU1_t month;
    UBXU1_t day;
    UBXU1_t hour;
    UBXU1_t minute;
    UBXU1_t second;
    UBXU1_t reserved2;
} UBXLOG_FINDTIME_IN;

typedef struct {
    UBXU1_t version; //Shall be 1
    UBXU1_t type; //Shall be 1
    UBXU2_t reserved1;
    UBXU4_t entryNumber;
} UBXLOG_FINDTIME_OUT;

//typedef struct UBXLOG_INFO_POLL {
    //No payload
//};

typedef struct
{
    UBXU1_t version;
    UBXU1_t reserved1[3];
    UBXU4_t filestoreCapacity;
    UBXU4_t reserved2;
    UBXU4_t reserved3;
    UBXU4_t currentMaxLogSize;
    UBXU4_t currentLogSize;
    UBXU4_t entryCount;
    UBXU2_t oldestYear;
    UBXU1_t oldestMonth;
    UBXU1_t oldestDay;
    UBXU1_t oldestHour;
    UBXU1_t oldestMinute;
    UBXU1_t oldestSecond;
    UBXU1_t reserved4;
    UBXU2_t newestYear;
    UBXU1_t newestMonth;
    UBXU1_t newestDay;
    UBXU1_t newestHour;
    UBXU1_t newestMinute;
    UBXU1_t newestSecond;
    UBXU1_t reserved5;
    UBXX1_t status; //See UBXLOGStatus to fill this field
    UBXU1_t reserved6[3];
} UBXLOG_INFO;

typedef struct {
    UBXU4_t entryIndex;
    UBXI4_t lon;
    UBXI4_t lat;
    UBXI4_t hMSL;
    UBXU4_t hAcc;
    UBXU4_t gSpeed;
    UBXU4_t heading;
    UBXU1_t version; //Shall be 0
    UBXU1_t fixType; //See UBXRETRIEVEPOSFixType to fill this field
    UBXU2_t year;
    UBXU1_t month;
    UBXU1_t day;
    UBXU1_t hour;
    UBXU1_t minute;
    UBXU1_t second;
    UBXU1_t reserved1;
    UBXU1_t numSV;
    UBXU1_t reserved2;
}  UBXLOG_RETRIEVEPOS;

typedef struct {
    UBXU4_t entryIndex;
    UBXU1_t version; //Shall be 0
    UBXU1_t reserved1;
    UBXU2_t year;
    UBXU1_t month;
    UBXU1_t day;
    UBXU1_t hour;
    UBXU1_t minute;
    UBXU1_t second;
    UBXU1_t reserved2;
    UBXU2_t byteCount;
    //Variable payload according byteCount
} UBXLOG_RETRIEVESTRING;

typedef struct {
    UBXU4_t startNumber;
    UBXU4_t entryCount;
    UBXU1_t version;
    UBXU1_t reserved[3];
} UBXLOG_RETRIEVE;

typedef struct {
#ifdef __WINDOWS__
    UBXU1_t payload;
#endif
    //Variable payload UBXU1_t
} UBXLOG_STRING;

typedef struct {
    UBXI1_t ofsI;
    UBXU1_t magI;
    UBXI1_t ofsQ;
    UBXU1_t magQ;
    UBXU1_t cfgSource;
    UBXU1_t reserved0[3];
    UBXU4_t lowLevCfg;
    UBXU4_t reserved1[2];
    UBXU4_t postStatus;
    UBXU4_t reserved2;
} UBXMON_HW2;

typedef struct
{
    UBXX1_t UBXHWFlagsRTCCalib:1;
    UBXX1_t UBXHWFlagsSafeBoot:1;
    UBXX1_t UBXHWFlagsJammingState:2;
} UBXHWFlags;

typedef struct {
    UBXX4_t pinSel;
    UBXX4_t pinBank;
    UBXX4_t pinDir;
    UBXX4_t pinVal;
    UBXU2_t noisePerMS;
    UBXU2_t agcCnt;
    UBXU1_t aStatus;
    UBXU1_t aPower;
    UBXHWFlags flags;
    UBXU1_t reserved1;
    UBXX4_t usedMask;
    UBXU1_t VP[17];
    UBXU1_t jamInd;
    UBXU2_t reserved3;
    UBXX4_t pinIrq;
    UBXX4_t pullH;
    UBXX4_t pullL;
} UBXMON_HW;

typedef struct
{
    UBXU4_t rxBytes;
    UBXU4_t txBytes;
    UBXU2_t parityErrs;
    UBXU2_t framingErrs;
    UBXU2_t overrunErrs;
    UBXU2_t breakCond;
    UBXU1_t rxBusy;
    UBXU1_t txBusy;
    UBXU2_t reserved1;
} UBXMON_IO_PART;

typedef struct {
    UBXMON_IO_PART ioPortInfo[UBX_IO_PORTS_NUM];
}UBXMON_IO ;

typedef struct {
    UBXU2_t msg1[8];
    UBXU2_t msg2[8];
    UBXU2_t msg3[8];
    UBXU2_t msg4[8];
    UBXU2_t msg5[8];
    UBXU2_t msg6[8];
    UBXU4_t skipped[6];
} UBXMON_MSGPP;

typedef struct {
    UBXU2_t pending[6];
    UBXU1_t usage[6];
    UBXU1_t peakUsage[6];
} UBXMON_RXBUF;

typedef struct {
    UBXX1_t flags; //See UBXRXRFlags to fill this field
} UBXMON_RXR;

typedef struct {
    UBXU2_t pending[6];
    UBXU1_t usage[6];
    UBXU1_t peakUsage[6];
    UBXU1_t tUsage;
    UBXU1_t tPeakusage;
    UBXX1_t errors;
    UBXU1_t reserved1;
} UBXMON_TXBUF;

//typedef struct {
    //No payload
//} UBXMON_VER_POLL;

typedef struct {
    UBXCH_t swVersion[30];
    UBXCH_t hwVersion[10];
    //Variable payload of UBXMON_VER_PART type
} UBXMON_VER;

typedef struct
{
    UBXCH_t extension[30];
} UBXMON_VER_PART;

typedef struct {
    UBXU4_t iTOW;
    UBXU1_t aopCfg; //See UBXAOPCfg to fill this field
    UBXU1_t status; //See UBXAOPStatus to fill this field
    UBXU1_t reserved0;
    UBXU1_t reserved1;
    UBXU4_t availGPS;
    UBXU4_t reserved2;
    UBXU4_t reserved3;
} UBXNAV_AOPSTATUS;

typedef struct {
    UBXU4_t iTOW;
    UBXI4_t clkB;
    UBXI4_t clkD;
    UBXU4_t tAcc;
    UBXU4_t fAcc;
} UBXNAV_CLOCK;

typedef struct {
    UBXU4_t iTOW;
    UBXI4_t age;
    UBXI2_t baseId;
    UBXI2_t baseHealth;
    UBXU1_t numCh;
    UBXU1_t status;
    UBXU2_t reserved1;
} UBXNAV_DGPS;

typedef struct
{
    UBXX1_t channel:4;
    UBXX1_t dgpsUsed:1;
} UBXDGPSFlags;

typedef struct {
    UBXU1_t svid;
    UBXDGPSFlags flags;
    UBXU2_t ageC;
    UBXR4_t prc;
    UBXR4_t prrc;
} UBXNAV_DGPS_PART;

typedef struct {
    UBXU4_t iTOW;
    UBXU2_t gDOP;
    UBXU2_t pDOP;
    UBXU2_t tDOP;
    UBXU2_t vDOP;
    UBXU2_t hDOP;
    UBXU2_t nDOP;
    UBXU2_t eDOP;
} UBXNAV_DOP;

typedef struct {
    UBXU4_t iTOW;
    UBXI4_t ecefX;
    UBXI4_t ecefY;
    UBXI4_t ecefZ;
    UBXU4_t pAcc;
} UBXNAV_POSECEF;

typedef struct {
    UBXU4_t iTOW;
    UBXI4_t lon;
    UBXI4_t lat;
    UBXI4_t height;
    UBXI4_t hMSL;
    UBXU4_t hAcc;
    UBXU4_t vAcc;
} UBXNAV_POSLLH;

typedef struct
{
    UBXX1_t gnssFixOk:1;
    UBXX1_t diffSoln:1;
    UBXX1_t psmState:3; //See UBXPVTPSMStates to fill this field
} UBXPVTFlags;

typedef struct {
    UBXU4_t iTOW;
    UBXU2_t year;
    UBXU1_t month;
    UBXU1_t day;
    UBXU1_t hour;
    UBXU1_t min;
    UBXU1_t sec;
    UBXX1_t valid; //See UBXPVTValid to fill this field
    UBXU4_t tAcc;
    UBXI4_t nano;
    UBXU1_t fixType; //See UBXGPSFix to fill this field
    UBXPVTFlags flags;
    UBXU1_t reserved1;
    UBXU1_t numSV;
    UBXI4_t lon;
    UBXI4_t lat;
    UBXI4_t height;
    UBXI4_t hMSL;
    UBXU4_t hAcc;
    UBXU4_t vAcc;
    UBXI4_t velN;
    UBXI4_t velE;
    UBXI4_t velD;
    UBXI4_t gSpeed;
    UBXI4_t heading;
    UBXU4_t sAcc;
    UBXU4_t headingAcc;
    UBXU2_t pDOP;
    UBXX2_t reserved2;
    UBXU4_t reserved3;
} UBXNAV_PVT;

typedef struct {
    UBXU4_t iTOW;
    UBXU1_t geo;
    UBXU1_t mode;
    UBXI1_t sys;
    UBXX1_t service; //See UBXSBASService to fill this field
    UBXU1_t cnt;
    UBXU1_t reserved0[3];
    //Variable payload of UBXNAV_SBAS_PART type
} UBXNAV_SBAS;

typedef struct {
    UBXU1_t svid;
    UBXU1_t flags;
    UBXU1_t udre;
    UBXU1_t svSys;
    UBXU1_t svService;
    UBXU1_t reserved1;
    UBXI2_t prc;
    UBXU2_t reserved2;
    UBXI2_t ic;
} UBXNAV_SBAS_PART;

typedef struct {
    UBXU4_t iTOW;
    UBXI4_t fTOW;
    UBXI2_t week;
    UBXU1_t gpsFix; //See UBXGPSFix to fill this field
    UBXX1_t flags; //See UBXSBASSOLFlags to fill this field
    UBXI4_t ecefX;
    UBXI4_t ecefY;
    UBXI4_t ecefZ;
    UBXU4_t pAcc;
    UBXI4_t ecefVX;
    UBXI4_t ecefVY;
    UBXI4_t ecefVZ;
    UBXU4_t sAcc;
    UBXU2_t pDOP;
    UBXU1_t reserved1;
    UBXU1_t numSV;
    UBXU4_t reserved2;
} UBXNAV_SOL;

typedef struct {
    UBXU4_t iTOW;
    UBXU1_t gpsFix;
    UBXX1_t flags; //See UBXGPSFix to fill this field
    UBXX1_t fixStat; //See UBXSBASSOLFlags to fill this field
    UBXX1_t flags2;
    UBXU4_t ttff;
    UBXU4_t msss;
} UBXNAV_STATUS;

typedef struct
{
    UBXX1_t chipGen:3; //See UBXSVINFOChipGen to fill this field
} UBXSVINFOGlobalFlags;

typedef struct {
    UBXU4_t iTOW;
    UBXU1_t numCh;
    UBXSVINFOGlobalFlags globalFlags;
    UBXU2_t reserved2;
    //Variable payload of UBXNAV_SVINFO_PART type
} UBXNAV_SVINFO;

typedef struct
{
    UBXX1_t qualityInd:4; //See UBXSVINFOQualityId to fill this field
} UBXSVINFOQuality;

typedef struct {
    UBXU1_t chn;
    UBXU1_t svid;
    UBXX1_t flags; //See UBXSVINFOFlags to fill this field
    UBXX1_t quality;
    UBXU1_t cno;
    UBXI1_t elev;
    UBXI2_t azim;
    UBXI4_t prRes;
} UBXNAV_SVINFO_PART;

typedef struct {
    UBXU4_t iTOW;
    UBXI4_t fTOW;
    UBXI2_t week;
    UBXI1_t leapS;
    UBXX1_t valid; //See UBXTIMEGPSValidityFlags to fill this field
    UBXU4_t tAcc;
} UBXNAV_TIMEGPS;

typedef struct {
    UBXU4_t iTOW;
    UBXU4_t tAcc;
    UBXI4_t nano;
    UBXU2_t year;
    UBXU1_t month;
    UBXU1_t day;
    UBXU1_t hour;
    UBXU1_t min;
    UBXU1_t sec;
    UBXX1_t valid; //See UBXTIMEUTCValidityFlags to fill this field
} UBXNAV_TIMEUTC;

typedef struct {
    UBXU4_t iTOW;
    UBXI4_t ecefVX;
    UBXI4_t ecefVY;
    UBXI4_t ecefVZ;
    UBXU4_t sAcc;
} UBXNAV_VELECEF;

typedef struct {
    UBXU4_t iTOW;
    UBXI4_t velN;
    UBXI4_t velE;
    UBXI4_t velD;
    UBXU4_t speed;
    UBXU4_t gSpeed;
    UBXI4_t heading;
    UBXU4_t sAcc;
    UBXU4_t cAcc;
} UBXNAV_VELNED;

//typedef struct {
    //No payload
//} UBXRXM_ALM_POLL;

typedef struct {
    UBXU1_t svid;
} UBXRXM_ALM_POLL_OPT;

typedef struct {
    /* ###################################################
     * This RMX messages marked as obsolete API use AID instead
     */
    UBXU4_t svid;
    UBXU4_t week;
} UBXRXM_ALM;

typedef struct {
    /* ###################################################
     * This RMX messages marked as obsolete API use AID instead
     */
    UBXU4_t svid;
    UBXU4_t week;
    UBXU4_t dwrd[8];
} UBXRXM_ALM_OPT;

//typedef struct {
    //No payload
//} UBXRXM_EPH_POLL;

typedef struct {
    UBXU1_t svid;
} UBXRXM_EPH_POLL_OPT;

typedef struct {
    /* ###################################################
     * This RMX messages marked as obsolete API use AID instead
     */
    UBXU4_t svid;
    UBXU4_t how;
} UBXRXM_EPH;

typedef struct {
    /* ###################################################
     * This RMX messages marked as obsolete API use AID instead
     */
    UBXU4_t svid;
    UBXU4_t how;
    UBXU4_t sf1d[8];
    UBXU4_t sf2d[8];
    UBXU4_t sf3d[8];
} UBXRXM_EPH_OPT;

typedef struct {
    UBXU4_t duration;
    UBXX4_t flags; //See UBXPMREQFlags to fill this field
} UBXRXM_PMREQ;

typedef struct {
    UBXI4_t rcvTow;
    UBXI2_t week;
    UBXU1_t numSV;
    UBXU1_t reserved1;
    //Variable payload of UBXRXM_RAW_PART type
} UBXRXM_RAW;

typedef struct
{
    UBXR8_t cpMes;
    UBXR8_t prMes;
    UBXR4_t doMes;
    UBXU1_t sv;
    UBXI1_t mesQI;
    UBXI1_t cno;
    UBXU1_t lli;
} UBXRXM_RAW_PART;

typedef struct {
    UBXU1_t chn;
    UBXU1_t svid;
    UBXX4_t dwrd[10];
} UBXRXM_SFRB;

typedef struct {
    UBXU4_t iTOW;
    UBXI2_t week;
    UBXU1_t numVis;
    UBXU1_t numSV;
    //Variable payload of UBXRXM_SVSI_PART type
} UBXRXM_SVSI;

typedef struct
{
    UBXX1_t ura:4;
    UBXX1_t healthy:1;
    UBXX1_t ephVal:1;
    UBXX1_t almVal:1;
    UBXX1_t notAvail:1;
} UBXSVSISVFlags;

typedef struct
{
    UBXX1_t almAge:4;
    UBXX1_t ephAge:4;
} UBXSVSIAge;

typedef struct
{
    UBXU1_t svid;
    UBXSVSISVFlags svFlag;
    UBXI2_t azim;
    UBXI1_t elev;
    UBXSVSIAge age;
} UBXRXM_SVSI_PART;


typedef struct
{
    UBXX1_t mode:1; //See UBXTM2FlagsMode to fill this field
    UBXX1_t run:1; //See UBXTM2FlagsRun to fill this field
    UBXX1_t newFallingEdge:1;
    UBXX1_t timeBase:2; //See UBXTM2FlagsTimeBase to fill this field
    UBXX1_t utc:1; //See UBXTM2FlagsUTC to fill this field
    UBXX1_t time:1; //See UBXTM2FlagsTime to fill this field
    UBXX1_t newRisingEdge:1;
} UBXTM2Flags;

typedef struct {
    UBXU1_t ch;
    UBXTM2Flags flags;
    UBXU2_t count;
    UBXU2_t wnR;
    UBXU2_t wnF;
    UBXU4_t towMsR;
    UBXU4_t towSubMsR;
    UBXU4_t towMsF;
    UBXU4_t towSubMsF;
    UBXU4_t accEst;
} UBXTIM_TM2;

typedef struct {
    UBXU4_t towMS;
    UBXU4_t towSubMS;
    UBXI4_t qErr;
    UBXU2_t week;
    UBXX1_t flags;
    UBXU1_t reserved1;
} UBXTIM_TP;

typedef struct
{
    UBXX1_t src:3; //See UBXVRFYFlagsSource to fill this field
} UBXVRFYFlags;

typedef struct {
    UBXI4_t itow;
    UBXI4_t frac;
    UBXI4_t deltaMs;
    UBXI4_t deltaNs;
    UBXU2_t wno;
    UBXVRFYFlags flags;
    UBXU1_t reserved1;
} UBXTIM_VRFY;

typedef union
{
    UBXACK_NACK ACK_NACK;
    UBXACK_ACK ACK_ACK;
    UBXAID_ALM_POLL_OPT AID_ALM_POLL_OPT;
    UBXAID_ALM AID_ALM;
    UBXAID_ALM_OPT AID_ALM_OPT;
    UBXAID_ALPSRV AID_ALPSRV;
    UBXAID_ALP_POLL AID_ALP_POLL;
    UBXAID_ALP_END AID_ALP_END;
    UBXAID_ALP AID_ALP;
    UBXAID_AOP_POLL_OPT AID_AOP_POLL_OPT;
    UBXAID_AOP AID_AOP;
    UBXAID_AOP_OPT AID_AOP_OPT;
    UBXAID_EPH_POLL_OPT AID_EPH_POLL_OPT;
    UBXAID_EPH AID_EPH;
    UBXAID_EPH_OPT AID_EPH_OPT;
    UBXAID_HUI AID_HUI;
    UBXAID_INI AID_INI;
    UBXCFG_ANT CFG_ANT;
    UBXCFG_CFG CFG_CFG;
    UBXCFG_CFG_OPT CFG_CFG_OPT;
    UBXCFG_DAT_IN CFG_DAT_IN;
    UBXCFG_DAT_OUT CFG_DAT_OUT;
    UBXCFG_GNSS CFG_GNSS;
    UBXCFG_INF_POLL CFG_INF_POLL;
    UBXCFG_INF CFG_INF;
    UBXCFG_ITFM CFG_ITFM;
    UBXCFG_LOGFILTER CFG_LOGFILTER;
    UBXCFG_MSG_POLL CFG_MSG_POLL;
    UBXCFG_MSG_RATE CFG_MSG_RATE;
    UBXCFG_MSG_RATES CFG_MSG_RATES;
    UBXCFG_NAV5 CFG_NAV5;
    UBXCFG_NAVX5 CFG_NAVX5;
    UBXCFG_NMEA CFG_NMEA;
    UBXCFG_NVS CFG_NVS;
    UBXCFG_PM2 CFG_PM2;
    UBXCFG_PRT_POLL_OPT CFG_PRT_POLL_OPT;
    UBXCFG_PRT CFG_PRT;
    UBXCFG_RATE CFG_RATE;
    UBXCFG_RINV CFG_RINV;
    UBXCFG_RST CFG_RST;
    UBXCFG_RXM CFG_RXM;
    UBXCFG_SBAS CFG_SBAS;
    UBXCFG_TP5_POLL_OPT CFG_TP5_POLL_OPT;
    UBXCFG_TP5 CFG_TP5;
    UBXCFG_USB CFG_USB;
    UBXINF_DEBUG INF_DEBUG;
    UBXINF_ERROR INF_ERROR;
    UBXINF_NOTICE INF_NOTICE;
    UBXINF_TEST INF_TEST;
    UBXINF_WARNING INF_WARNING;
    UBXLOG_CREATE LOG_CREATE;
    UBXLOG_FINDTIME_IN LOG_FINDTIME_IN;
    UBXLOG_FINDTIME_OUT LOG_FINDTIME_OUT;
    UBXLOG_INFO LOG_INFO;
    UBXLOG_RETRIEVEPOS LOG_RETRIEVEPOS;
    UBXLOG_RETRIEVESTRING LOG_RETRIEVESTRING;
    UBXLOG_RETRIEVE LOG_RETRIEVE;
    UBXLOG_STRING LOG_STRING;
    UBXMON_HW2 MON_HW2;
    UBXMON_HW MON_HW;
    UBXMON_IO MON_IO;
    UBXMON_MSGPP MON_MSGPP;
    UBXMON_RXBUF MON_RXBUF;
    UBXMON_RXR MON_RXR;
    UBXMON_TXBUF MON_TXBUF;
    UBXMON_VER MON_VER;
    UBXNAV_AOPSTATUS NAV_AOPSTATUS;
    UBXNAV_CLOCK NAV_CLOCK;
    UBXNAV_DGPS NAV_DGPS;
    UBXNAV_DOP NAV_DOP;
    UBXNAV_POSECEF NAV_POSECEF;
    UBXNAV_POSLLH NAV_POSLLH;
    UBXNAV_PVT NAV_PVT;
    UBXNAV_SBAS NAV_SBAS;
    UBXNAV_SOL NAV_SOL;
    UBXNAV_STATUS NAV_STATUS;
    UBXNAV_SVINFO NAV_SVINFO;
    UBXNAV_TIMEGPS NAV_TIMEGPS;
    UBXNAV_TIMEUTC NAV_TIMEUTC;
    UBXNAV_VELECEF NAV_VELECEF;
    UBXNAV_VELNED NAV_VELNED;
    UBXRXM_ALM_POLL_OPT RXM_ALM_POLL_OPT;
    UBXRXM_ALM_OPT RXM_ALM_OPT;
    UBXRXM_EPH_POLL_OPT RXM_EPH_POLL_OPT;
    UBXRXM_EPH RXM_EPH;
    UBXRXM_EPH_OPT RXM_EPH_OPT;
    UBXRXM_PMREQ RXM_PMREQ;
    UBXRXM_RAW RXM_RAW;
    UBXRXM_SFRB RXM_SFRB;
    UBXRXM_SVSI RXM_SVSI;
    UBXTIM_TM2 TIM_TM2;
    UBXTIM_TP TIM_TP;
    UBXTIM_VRFY TIM_VRFY;
} UBXMsgs;

typedef struct
{
    int fileId;
    char* alpData;
    int dataSize;
} UBXAlpFileInfo;

typedef struct
{
    UBXU2_t preamble;
    UBXHdr hdr;
    UBXMsgs payload;
} UBXMsg;

typedef struct
{
    int size;
    char* data;
} UBXMsgBuffer;

#pragma pack(pop)

#endif // UBXMESSAGE_H
