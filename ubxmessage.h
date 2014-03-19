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

#ifndef UBXMESSAGE_H
#define UBXMESSAGE_H

#include <stdio.h>
#include "portable_endian.h"

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

static const int UBX_CHECKSUM_SIZE = 2;
static const int UBX_HEADER_SIZE = 6;
static const u_int16_t UBX_PREAMBLE = 0xB562;

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
#elif defined (_WIN32)
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

enum UBXMessageClass
{
    UBXMsgClassNAV = 0x01, // Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
    UBXMsgClassRXM = 0x02, // Receiver Manager Messages: Satellite Status, RTC Status
    UBXMsgClassINF = 0x04, // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
    UBXMsgClassACK = 0x05, // Ack/Nack Messages: as replies to CFG Input Messages
    UBXMsgClassCFG = 0x06, // Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
    UBXMsgClassMON = 0x0A, // Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
    UBXMsgClassAID = 0x0B, // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
    UBXMsgClassTIM = 0x0D, // Timing Messages: Time Pulse Output, Timemark Results
    UBXMsgClassLOG = 0x21, // Logging Messages: Log creation, deletion, info and retrieval
    UBXMsgClassInvalid = 255
};

enum UBXMessageId
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

    MsgIdInvalid = 0xFF
};

enum UBXResetMode
{
    UBXHardwareReset = 0x00, // Hardware reset (Watchdog) immediately
    UBXControlledReset = 0x01, // Controlled Software reset
    UBXControlledResetGNSSOnly = 0x02, //Controlled Software reset (GNSS only)
    UBXHardwareResetAfterShutdown = 0x04, //Hardware reset (Watchdog) after
    UBXControlledGNSSStop = 0x08, //Controlled GNSS stop
    UBXControlledGNSSStart = 0x09 //Controlled GNSS start
};

enum UBXBBRSpecialSets
{
    UBXBBRHotStart = 0x0000,
    UBXBBRWarmStart = 0x0001,
    UBXBBRColdStart = 0xFFFF
};

enum UBXBBRMask
{
    UBXBBReph = 1, //Ephemeris
    UBXBBRalm = 1 << 1, //Almanac
    UBXBBRhealth = 1 << 2, //Health
    UBXBBRklob = 1 << 3, //Klobuchar parameters
    UBXBBRpos = 1 << 4, //Position
    UBXBBRclkd = 1 << 5, //Clock Drift
    UBXBBRosc = 1 << 6, //Oscillator Parameter
    UBXBBRutc = 1 << 7, //UTC Correction + GPS Leap Seconds Parameters
    UBXBBRrtc = 1 << 8, //RTC
    UBXBBRsfdr = 1 << 11, //SFDR Parameters
    UBXBBRvmon = 1 << 12, //SFDR Vehicle Monitoring Parameters
    UBXBBRtct = 1 << 13, //TCT Parameters
    UBXBBRaop = 1 << 15 //Autonomous Orbit Parameters
};

enum UBXHUIFlags
{
    UBXHUIHealthValid = 1,
    UBXHUIUTCValid = 1 << 1,
    UBXHUIKlobValid = 1 << 2
};

enum UBXINItmCfg
{
    UBXINIfEdge = 1 << 1,
    UBXINItm1 = 1 << 4,
    UBXINIf1 = 1 << 6
};

enum UBXINIFlags
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
};

enum UBXANTFlags
{
    UBXANTsvcs = 1,
    UBXANTscd = 1 << 1,
    UBXANTocd = 1 << 2,
    UBXANTpdwnOnSCD = 1 << 3,
    UBXANTrecovery = 1 << 4
};

enum UBXCFGMask
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
};

enum UBXCFGDeviceMask
{
    UBXCFGdevBBR = 1,
    UBXCFGdevFlash = 1 << 1,
    UBXCFGdevEEPROM = 1 << 2,
    UBXCFGdevSpiFlash = 1 << 5
};

enum UBXCFGTimepulses
{
    UBXCFGTimepulse = 0,
    UBXCFGTimepulse2 = 1
};

enum UBXCFGTimepulseFlags
{
    UBXCFGTimepulseActive = 1,
    UBXCFGTimepulseLockGpsFreq = 1 << 1,
    UBXCFGTimepulseLockedOtherSet = 1 << 2,
    UBXCFGTimepulseIsFreq = 1 << 3,
    UBXCFGTimepulseIsLenght = 1 << 4,
    UBXCFGTimepulseAlignToTow = 1 << 5,
    UBXCFGTimepulsePolarity = 1 << 6,
    UBXCFGTimepulseGridUTSGPS = 1 << 7
};

enum UBXCFGProtocolIds
{
    UBXProtocol = 0,
    UBXNMEAProtocol
};

enum UBXGNSSIds
{
    UBXGPS,
    UBXSBAS,
    UBXQZSS,
    UBXGLONASS
};
enum UBXCFGInfMsgMask
{
    UBXInfError = 1,
    UBXInfWarning = 1 << 1,
    UBXInfNotice = 1 << 2,
    UBXInfDebug = 1 << 3,
    UBXInfTest = 1 << 4
};

enum UBXITFMAntSetting
{
    UBXITFMAntUnknown = 0,
    UBXITFMAntPassive = 1,
    UBXITFMAntActive = 2
};

enum UBXLOGFILTERFlags
{
    UBXLOGFILTERRecordEnabled = 1,
    UBXLOGFILTERPsmOncePerWakupEnabled = 1 << 1,
    UBXLOGFILTERApplyAllFilterSettings = 1 << 2
};


enum UBXNAV5Mask
{
    UBXNAV5Dyn = 1,
    UBXNAV5MinEl = 1 << 1,
    UBXNAV5PosFixMode = 1 << 2,
    UBXNAV5DrLim = 1 << 3,
    UBXNAV5PosMask = 1 << 4,
    UBXNAV5TimeMask = 1 << 5,
    UBXNAV5StaticHoldMask = 1 << 6,
    UBXNAV5DgpsMask = 1 << 7,
};

enum UBXNAVX5Mask
{
    UBXNAVX5AopMinMax = 1 << 2,
    UBXNAVX5AopMinCno = 1 << 3,
    UBXNAVX5AopInitial3dfix = 1 << 6,
    UBXNAVX5AopWknRoll = 1 << 9,
    UBXNAVX5AopPPP = 1 << 13,
    UBXNAVX5Aop = 1 << 14
};


enum UBXNMEAFilter
{
    UBXNMEAPosFilter = 1,
    UBXNMEAMskPosFilter = 1 << 1,
    UBXNMEATimeFilter = 1 << 2,
    UBXNMEADateFilter = 1 << 3,
    UBXNMEAGPSOnlyFilter = 1 << 4,
    UBXNMEATrackFilter = 1 << 5,
};

enum UBXNMEAFlags
{
    UBXNMEACompatFlag = 1,
    UBXNMEAConsiderFlag = 1 << 1
};

enum UBXNMEAGNSSToFilter
{
    UBXNMEAGPSFilter = 1,
    UBXNMEASBASFilter = 1 << 1,
    UBXNMEAQZSSFilter = 1 << 4,
    UBXNMEAGLONASSFilter = 1 << 5
};

enum UBXPM2LimitPeakCurrent
{
    UBXPM2LimitCurrentDisabled = 0x00,
    UBXPM2LimitCurrentEnabled = 0x01
};

enum UBXPM2Mode
{
    UBXPM2OnOffOperation = 0x00,
    UBXPM2CyclicTrackOperation = 0x01
};


enum UBXPRTModeCharLen
{
    UBXPRTMode5BitCharLen = 0x00, //Not supported
    UBXPRTMode6BitCharLen = 0x01, //Not supported
    UBXPRTMode7BitCharLen = 0x02, //Supported only with parity
    UBXPRTMode8BitCharLen = 0x03
};

enum UBXPRTModeParity
{
    UBXPRTModeEvenParity = 0,
    UBXPRTModeOddParity = 1,
    UBXPRTModeNoParity = 1 << 3,
    UBXPRTModeReserved = 1 << 2
};

enum UBXPRTModeStopBits
{
    UBXPRTMode1StopBit = 0,
    UBXPRTMode1dot5StopBit = 1,
    UBXPRTMode2StopBit = 2,
    UBXPRTMode0dot5StopBit = 3,
};

enum UBXPRTInProtoMask
{
    UBXPRTInProtoInUBX = 1,
    UBXPRTInProtoInNMEA = 1 << 1,
    UBXPRTInProtoInRTCM = 1 << 2
};

enum UBXPRTOutProtoMask
{
    UBXPRTOutProtoOutUBX = 1,
    UBXPRTOutProtoOutNMEA = 1 << 1
};

enum UBXPRTFlags
{
    extendedTxTimeout = 1 << 1
};

enum UBXPRTSPIMode {
    UBXPRTSPIMode0 = 0, //CPOL = 0, CPHA = 0
    UBXPRTSPIMode1, //CPOL = 0, CPHA = 1
    UBXPRTSPIMode2, //CPOL = 1, CPHA = 0
    UBXPRTSPIMode3, //CPOL = 1, CPHA = 1
};

enum UBXRINVFlags
{
    UBXRINVBinary = 1,
    UBXRINVDump = 1 << 1
};

enum UBXRXMLowPowerModes
{
    UBXRXMContinousMode = 0,
    UBXRXMPowerSaveMode = 1
};

enum UBXSBASModes
{
    UBXSBASModeEnabled = 1,
    UBXSBASModeTest = 1 << 1
};

enum UBXSBASUsage
{
    UBXSBASUsageRange = 1,
    UBXSBASUsageDiffCorr = 1 << 1,
    UBXSBASUsageIntegrity = 1 << 2
};

enum UBXSBASScanModes2
{
    UBXSBASScanModePRN152 = 1,
    UBXSBASScanModePRN153 = 1 << 1,
    UBXSBASScanModePRN154 = 1 << 2,
    UBXSBASScanModePRN155 = 1 << 3,
    UBXSBASScanModePRN156 = 1 << 4,
    UBXSBASScanModePRN157 = 1 << 5,
    UBXSBASScanModePRN158 = 1 << 6
};

enum UBXSBASScanModes1
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
};

enum UBXUSBFlags
{
    USBFlagReEnum,
    USBFlagPowerMode
};

enum UBXLOGCfg
{
    UBXLOGCfgCircular
};

enum UBXLOGSize
{
    UBXLOGMaximumSafeSize = 0,
    UBXLOGMinimunSize = 1,
    UBXLOGUserDefined = 2,
};

enum UBXLOGStatus
{
    UBXLOGStatusRecording = 1 << 3,
    UBXLOGStatusInactive = 1 << 4,
    UBXLOGStatusCircular = 1 << 5
};

enum UBXRETRIEVEPOSFixType
{
    UBXRETRIEVEPOS2DFix = 2,
    UBXRETRIEVEPOS3DFix = 3
};

enum UBXRXRFlags
{
    UBXRXRAwake = 1
};

enum UBXAOPStatus
{
    UBXAOPStatusIdle = 0,
    UBXAOPStatusRunning = 1
};

enum UBXAOPCfg
{
    UBXAOPCfgUseAOP = 1
};

enum UBXGPSFix
{
    UBXGPSNoFix = 0x00,
    UBXGPSDeadReckoning = 0x01,
    UBXGPS2DFix = 0x02,
    UBXGPS3DFix = 0x03,
    UBXGPSGNSSDeadReckoning = 0x04,
    UBXGPSTimeOnlyFix = 0x05
};

enum UBXPVTValid
{
    UBXPVTValidDate = 1,
    UBXPVTValidTime = 1 << 1,
    UBXPVTFullyResolved = 1 << 2,
};

enum UBXPVTPSMStates
{
    UBXPVTPSMStateNA = 0,
    UBXPVTPSMStateEnabled = 1,
    UBXPVTPSMStateAcquisition = 2,
    UBXPVTPSMStateTracking = 3,
    UBXPVTPSMStatePowerOptim = 4,
    UBXPVTPSMStateInactive = 5,

};

enum UBXSBASService
{
    UBXSBASServiceRanging = 1,
    UBXSBASServiceCorrections = 1 << 1,
    UBXSBASServiceIntegrity = 1 << 2,
    UBXSBASServiceTestmode = 1 << 3
};

enum UBXSBASSOLFlags
{
    UBXSBASSOLGPSfixOK = 1,
    UBXSBASSOLDiffSoln = 1 << 1,
    UBXSBASSOLWKNSet = 1 << 2,
    UBXSBASSOLTOWSet = 1 << 3
};

enum UBXSVINFOChipGen
{
    UBXSVINFOAntarisChip = 0,
    UBXSVINFOUBlox5Chip = 1,
    UBXSVINFOUBlox6Chip = 2
};

enum UBXSVINFOFlags
{
    UBXSVINFOFlagsSVUsed = 1,
    UBXSVINFOFlagsDiffCorr = 1 << 1,
    UBXSVINFOFlagsOrbitAvail = 1 << 2,
    UBXSVINFOFlagsOrbitEph = 1 << 3,
    UBXSVINFOFlagsUnhealthy = 1 << 4,
    UBXSVINFOFlagsOrbitAlm = 1 << 5,
    UBXSVINFOFlagsOrbitAop = 1 << 6,
    UBXSVINFOFlagsSmoothed = 1 << 7
};

enum UBXSVINFOQualityId
{
    UBXSVINFOQualityChannelIdle = 0,
    UBXSVINFOQualityChannelSearching = 1,
    UBXSVINFOQualitySignalAquired = 2,
    UBXSVINFOQualitySignalDetected = 3,
    UBXSVINFOQualityCodeLockOnSignal = 4,
    UBXSVINFOQualityCodeCarrierLocked = 5
};

enum UBXTIMEGPSValidityFlags
{
    UBXTIMEGPSTowValid = 1,
    UBXTIMEGPSWeekValid = 1 << 1,
    UBXTIMEGPSLeapSValid = 1 << 2
};

enum UBXTIMEUTCValidityFlags
{
    UBXTIMEUTCValidTOW = 1,
    UBXTIMEUTCValidWKN = 1 << 1,
    UBXTIMEUTCValidUTC = 1 << 2
};

enum UBXPMREQFlags
{
    UBXPMREQBackup = 1 << 1
};

enum UBXTM2FlagsMode
{
    UBXTM2FlagsModeSingle = 0,
    UBXTM2FlagsModeRunning = 1
};

enum UBXTM2FlagsRun
{
    UBXTM2FlagsRunArmed = 0,
    UBXTM2FlagsRunStopped = 1
};

enum UBXTM2FlagsTimeBase
{
    UBXTM2FlagsTimeBaseReceiverTime = 0,
    UBXTM2FlagsTimeBaseGPS = 1,
    UBXTM2FlagsTimeBaseUTC = 2
};

enum UBXTM2FlagsUTC
{
    UBXTM2FlagsUTCNotAvailable = 0,
    UBXTM2FlagsUTCAvailable = 1
};

enum UBXTM2FlagsTime
{
    UBXTM2FlagsTimeInvalid = 0,
    UBXTM2FlagsTimeValid = 1
};

enum UBXTPFlags
{
    UBXTPTimeBase = 1,
    UBXTPUTC = 1 << 1
};

enum UBXVRFYFlagsSource
{
    UBXVRFYNoTimeAidingSone = 0,
    UBXVRFYSourceRTC = 2,
    UBXVRFYSourceAID_INI = 3
};

#pragma pack(push,1)

struct UBXHdr {
    UBXU1_t msgClass;
    UBXU1_t msgId;
    UBXU2_t length;
};

struct UBXAID_ALPSRV {
    UBXU1_t idSize;
    UBXU1_t type;
    UBXU2_t offset;
    UBXU2_t size;
    UBXU2_t fileId;
    UBXU2_t dataSize;
    UBXU1_t id1;
    UBXU1_t id2;
    UBXU4_t id3;
};

struct UBXACK_ACK {
    UBXU1_t msgClass;
    UBXU1_t msgId;
};

struct UBXACK_NACK {
    UBXU1_t msgClass;
    UBXU1_t msgId;
};

struct UBXAID_ALM_POLL {
    //No payload
};

struct UBXAID_ALM_POLL_OPT {
    UBXU1_t svid;
};

struct UBXAID_ALM {
    UBXU4_t svid;
    UBXU4_t week;
};

struct UBXAID_ALM_OPT {
    UBXU4_t svid;
    UBXU4_t week;
    UBXU4_t dwrd[8];
};

struct UBXAID_ALP {
    //No payload
};

struct UBXAID_ALP_END {
    UBXU1_t dummy;
};

struct UBXAID_ALP_POLL {
    UBXU4_t predTow;
    UBXU4_t predDur;
    UBXI4_t age;
    UBXU2_t predWno;
    UBXU2_t almWno;
    UBXU4_t reserved1;
    UBXU1_t svs;
    UBXU1_t reserved2;
    UBXU2_t reserved3;
};

struct UBXAID_AOP_POLL {
    //No payload
};

struct UBXAID_AOP_POLL_OPT {
    UBXU1_t svid;
};

struct UBXAID_AOP {
    UBXU1_t svid;
    UBXU1_t data[59];
};

struct UBXAID_AOP_OPT {
    UBXU1_t svid;
    UBXU1_t data[59];
    UBXU1_t optional0[48];
    UBXU1_t optional1[48];
    UBXU1_t optional2[48];
};

struct UBXAID_DATA_POLL {
    //No payload
};

struct UBXAID_EPH_POLL {
    //No payload
};

struct UBXAID_EPH_POLL_OPT {
    UBXU1_t svid;
};

struct UBXAID_EPH {
    UBXU4_t svid;
    UBXU4_t how;
};

struct UBXAID_EPH_OPT {
    UBXU4_t svid;
    UBXU4_t how;
    UBXU4_t sf1d[8];
    UBXU4_t sf2d[8];
    UBXU4_t sf3d[8];
};

struct UBXAID_HUI_POLL {
    //No payload
};

struct UBXAID_HUI {
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
};

struct UBXAID_INI_POLL {
    //No payload
};

struct UBXAID_INI {
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
};

struct UBXAID_REQ {
    //No payload
};

struct UBXCFG_ANT_POLL {
    //No payload
};

struct UBXANTPins
{
    UBXX2_t UBXANTpinSwitch:5;
    UBXX2_t UBXANTpinSCD:5;
    UBXX2_t UBXANTpinOCD:5;
    UBXX2_t UBXANTreconfig:1;
};

struct UBXCFG_ANT {
    UBXX2_t flags; //See UBXANTFlags to fill this field
    struct UBXANTPins pins;
};

struct UBXCFG_CFG {
    UBXX4_t clearMask; //See UBXCFGMask to fill this field
    UBXX4_t saveMask; //See UBXCFGMask to fill this field
    UBXX4_t loadMask; //See UBXCFGMask to fill this field
};

struct UBXCFG_CFG_OPT {
    UBXX4_t clearMask; //See UBXCFGMask to fill this field
    UBXX4_t saveMask; //See UBXCFGMask to fill this field
    UBXX4_t loadMask; //See UBXCFGMask to fill this field
    UBXX1_t deviceMask; //See UBXCFGDeviceMask to fill this field
};

struct UBXCFG_DAT_POLL {
    //No payload
};

struct UBXCFG_DAT_IN {
    UBXR8_t majA;
    UBXR8_t flat;
    UBXR4_t dX;
    UBXR4_t dY;
    UBXR4_t dZ;
    UBXR4_t rotX;
    UBXR4_t rotY;
    UBXR4_t rotZ;
    UBXR4_t scale;
};

struct UBXCFG_DAT_OUT {
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
};

struct UBXCFG_GNSS_POLL {
    //No payload
};

struct UBXCFG_GNSS {
    UBXU1_t msgVer;
    UBXU1_t numTrkChHw;
    UBXU1_t numTrkChUse;
    UBXU1_t numConfigBlocks;
    //Variable addition here
    //See structure below
};

struct UBXCFG_GNSS_PART
{
    UBXU1_t gnssId; //See UBXGNSSIds to fill this field
    UBXU1_t resTrkCh;
    UBXU1_t maxTrkCh;
    UBXU1_t reserved1;
    UBXX4_t flags; //0 - disabled, 1 - enabled
};

struct UBXCFG_INF_POLL {
    UBXU1_t protocolId;
};

struct UBXCFG_INF {
    //Variable payload
    //See structure UBXCFG_INF_PART below
};

struct UBXCFG_INF_PART {
    UBXU1_t protocolId;
    UBXU1_t reserved0;
    UBXU2_t reserved1;
    UBXX1_t infMsgMask[6]; //See UBXCFGInfMsgMask to fill this field
};

struct UBXCFG_ITFM_POLL {
    //No payload
};

struct UBXITFMConfig
{
    UBXX4_t bbThreshold:4;
    UBXX4_t cwThreshold:5;
    UBXX4_t reserved1:22; //Should be 0x16B156
    UBXX4_t enbled:1;
};

struct UBXITFMConfig2
{
    UBXX4_t reserved2:12; //Should be 0x31E
    UBXX4_t antSetting:2; //See UBXITFMAntSetting to fill this field
    UBXX4_t reserved3:18; //Should be 0x00
};

struct UBXCFG_ITFM {
    struct UBXITFMConfig config;
    struct UBXITFMConfig2 config2;
};

struct UBXCFG_LOGFILTER_POLL {
    //No payload data
};

struct UBXCFG_LOGFILTER {
    UBXU1_t version;
    UBXX1_t flags; //See UBXLOGFILTERFlags to fill this field
    UBXU2_t minIterval;
    UBXU2_t timeThreshold;
    UBXU2_t speedThreshold;
    UBXU4_t positionThreshold;
};

struct UBXCFG_MSG_POLL {
    UBXU1_t msgClass;
    UBXU1_t msgId;
};

struct UBXCFG_MSG_RATES {
    UBXU1_t msgClass;
    UBXU1_t msgId;
    UBXU1_t rate[6];
};

struct UBXCFG_MSG_RATE {
    UBXU1_t msgClass;
    UBXU1_t msgId;
    UBXU1_t rate;
};

struct UBXCFG_NAV5_POLL {
    //No payload data
};

struct UBXCFG_NAV5 {
    UBXX2_t mask; //See UBXNAV5Mask to fill this field
    UBXU1_t dynModel;
    UBXU1_t fixMode;
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
};

struct UBXCFG_NAVX5_POLL {
    //No payload
};

struct UBXCFG_NAVX5 {
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
};

struct UBXCFG_NMEA_POLL {
    //No payload
};

struct UBXCFG_NMEA {
    UBXX1_t filter; //See UBXNMEAFilter to fill this field
    UBXU1_t nmeaVersion;
    UBXU1_t numSV;
    UBXX1_t flags; //See UBXNMEAFlags to fill this field
    UBXX4_t gnssToFilter; //See UBXNMEAGNSSToFilter to fill this field
    UBXU1_t svNumbering;
    UBXU1_t mainTalkerId;
    UBXU1_t gsvTalkerId;
    UBXU1_t reserved;
};

struct UBXCFG_NVS {
    UBXX4_t clearMask; //See UBXCFGMask CFG_NVS section to fill this field
    UBXX4_t saveMask; //See UBXCFGMask CFG_NVS section to fill this field
    UBXX4_t loadMask; //See UBXCFGMask CFG_NVS section to fill this field
    UBXX1_t deviceMask; //See UBXCFGDeviceMask to fill this field
};

struct UBXCFG_PM2_POLL {
    //No payload
};

struct UBXCFG_PM2Flags
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
};

struct UBXCFG_PM2 {
    UBXU1_t version;
    UBXU1_t reserved1;
    UBXU1_t reserved2;
    UBXU1_t reserved3;
    struct UBXCFG_PM2Flags flags;
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
};

struct UBXCFG_PRT_POLL {
    //No payload
};

struct UBXCFG_PRT_POLL_OPT {
    UBXU1_t portId;
};

struct UBXCFG_PRTTxReady
{
    UBXX2_t en:1; //0 - disabled, 1 - enabled
    UBXX2_t pol:1; //0 - High-active, 1 - Low-active
    UBXX2_t pin:5;
    UBXX2_t thres:9; //Given value is multiplied by 8 bytes
};

struct UBXCFG_PRTUARTMode
{
    UBXX4_t blank0:4;
    UBXX4_t reserved1:1;
    UBXX4_t blank1:1;
    UBXX4_t charLen:2; //See UBXPRTModeCharLen to fill this field
    UBXX4_t blank2:1;
    UBXX4_t parity:3; //See UBXPRTModeParity to fill this field
    UBXX4_t nStopBits:2; //See UBXPRTModeStopBits to fill this field
    UBXX4_t blank3:18;
};

struct UBXCFG_PRTSPIMode
{
    UBXX4_t blank0:1;
    UBXX4_t spiMode:2; //See UBXPRTSPIMode to fill this field
    UBXX4_t blank1:3;
    UBXX4_t flowControl:1; //0 - disabled, 1 - enabled
    UBXX4_t blank2:1;
    UBXX4_t ffCnt:8;
    UBXX4_t blank3:16;
};

struct UBXCFG_PRTDDCMode
{
    UBXX4_t blank0:1;
    UBXX4_t slaveAddr:7; //Range: 0x07 < slaveAddr < 0x78. Bit 0 shall be 0
    UBXX4_t blank1:24;
};

union UBXCFG_PRTMode
{
    struct UBXCFG_PRTUARTMode UART;
    struct UBXCFG_PRTSPIMode SPI;
    struct UBXCFG_PRTDDCMode DDC;
    UBXX4_t USB; //reserved
};

union UBXCFG_PRT5Option
{
    UBXU4_t UARTbaudRate;
    UBXU4_t OtherReserved;
};

struct UBXCFG_PRT
{
    UBXU1_t portID;
    UBXU1_t reserved0;
    struct UBXCFG_PRTTxReady txReady;
    union UBXCFG_PRTMode mode;
    union UBXCFG_PRT5Option option;
    UBXX2_t inProtoMask; //See UBXPRTInProtoMask to fill this field
    UBXX2_t outProtoMask; //See UBXPRTOutProtoMask to fill this field
    UBXX2_t flags; //See UBXPRTFlags to fill this field, shall be 0 for USB
    UBXU1_t reserved5;
};

struct UBXCFG_RATE_POLL {
    //No payload
};

struct UBXCFG_RATE {
    UBXU2_t measRate;
    UBXU2_t navRate;
    UBXU2_t timeRef;
};

struct UBXCFG_RINV_POLL {
    //No payload
};

struct UBXCFG_RINV {
    UBXX1_t flags; //See UBXRINVFlags to fill this field
    //Variable payload size
};

struct UBXCFG_RST {
    UBXX2_t navBBRMask;
    UBXU1_t resetMode;
    UBXU1_t reserved1;
};

struct UBXCFG_RXM_POLL {
    //No payload
};

struct UBXCFG_RXM {
    UBXU1_t reserved1; //Shall be set to 8
    UBXU1_t lpMode; //See UBXRXMLowPowerModes to fill this field
};

struct UBXCFG_SBAS_POLL {
    //No payload
};

struct UBXCFG_SBAS {
    UBXX1_t mode; //See UBXSBASModes to fill this field
    UBXX1_t usage; //See UBXSBASUsage to fill this field
    UBXU1_t maxSBAS;
    UBXX1_t scanmode2; //See UBXSBASScanModes2 to fill this field
    UBXX4_t scanmode1; //See UBXSBASScanModes1 to fill this field
};

struct UBXCFG_TP5_POLL {
    //No payload
};

struct UBXCFG_TP5_POLL_OPT {
    UBXU1_t tpIdx;
};

struct UBXCFG_TP5 {
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
};

struct UBXCFG_USB_POLL {
    //No payload
};

struct UBXCFG_USB {
    UBXU2_t vendorId;
    UBXU2_t productId;
    UBXU2_t reserved1;//Set to 0
    UBXU2_t reserved2;//Set to 1
    UBXU2_t powerConsumption;
    UBXX2_t flags;
    UBXCH_t vendorString[32];
    UBXCH_t productString[32];
    UBXCH_t serialNumber[32];
};

struct UBXINF_DEBUG {
    //Variable payload of UBXCH_t
};

struct UBXINF_ERROR {
    //Variable payload of UBXCH_t
};

struct UBXINF_NOTICE {
    //Variable payload of UBXCH_t
};

struct UBXINF_TEST {
    //Variable payload of UBXCH_t
};

struct UBXINF_WARNING {
    //Variable payload of UBXCH_t
};

struct UBXLOG_CREATE {
    UBXU1_t version;
    UBXX1_t logCfg; //See UBXLOGCfg
    UBXU1_t reserved; //Set to 0
    UBXU1_t logSize;
    UBXU4_t userDefinedSize;
};

struct UBXLOG_ERASE {
    //No payload
};

struct UBXLOG_FINDTIME_IN {
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
};

struct UBXLOG_FINDTIME_OUT {
    UBXU1_t version; //Shall be 1
    UBXU1_t type; //Shall be 1
    UBXU2_t reserved1;
    UBXU4_t entryNumber;
};

struct UBXLOG_INFO_POLL {
    //No payload
};

struct UBXLOG_INFO
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
};

struct UBXLOG_RETRIEVEPOS {
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
};

struct UBXLOG_RETRIEVESTRING {
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
};

struct UBXLOG_RETRIEVE {
    UBXU4_t startNumber;
    UBXU4_t entryCount;
    UBXU1_t version;
    UBXU1_t reserved[3];
};

struct UBXLOG_STRING {
    //Variable payload UBXU1_t
};

struct UBXMON_HW2 {
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
};

struct UBXHWFlags
{
    UBXX1_t UBXHWFlagsRTCCalib:1;
    UBXX1_t UBXHWFlagsSafeBoot:1;
    UBXX1_t UBXHWFlagsJammingState:2;
};

struct UBXMON_HW {
    UBXX4_t pinSel;
    UBXX4_t pinBank;
    UBXX4_t pinDir;
    UBXX4_t pinVal;
    UBXU2_t noisePerMS;
    UBXU2_t agcCnt;
    UBXU1_t aStatus;
    UBXU1_t aPower;
    struct UBXHWFlags flags;
    UBXU1_t reserved1;
    UBXX4_t usedMask;
    UBXU1_t VP[17];
    UBXU1_t jamInd;
    UBXU2_t reserved3;
    UBXX4_t pinIrq;
    UBXX4_t pullH;
    UBXX4_t pullL;
};

struct UBXMON_IO_PART
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
};

struct UBXMON_IO {
    struct UBXMON_IO_PART ioPortInfo[UBX_IO_PORTS_NUM];
};

struct UBXMON_MSGPP {
    UBXU2_t msg1[8];
    UBXU2_t msg2[8];
    UBXU2_t msg3[8];
    UBXU2_t msg4[8];
    UBXU2_t msg5[8];
    UBXU2_t msg6[8];
    UBXU4_t skipped[6];
};

struct UBXMON_RXBUF {
    UBXU2_t pending[6];
    UBXU1_t usage[6];
    UBXU1_t peakUsage[6];
};

struct UBXMON_RXR {
    UBXX1_t flags; //See UBXRXRFlags to fill this field
};

struct UBXMON_TXBUF {
    UBXU2_t pending[6];
    UBXU1_t usage[6];
    UBXU1_t peakUsage[6];
};

struct UBXMON_VER_POLL {
    //No payload
};

struct UBXMON_VER {
    UBXCH_t swVersion[30];
    UBXCH_t hwVersion[10];
    //Variable payload of UBXMON_VER_PART type
};

struct UBXMON_VER_PART
{
    UBXCH_t extension[30];
};

struct UBXNAV_AOPSTATUS {
    UBXU4_t iTOW;
    UBXU1_t aopCfg; //See UBXAOPCfg to fill this field
    UBXU1_t status; //See UBXAOPStatus to fill this field
    UBXU1_t reserved0;
    UBXU1_t reserved1;
    UBXU4_t availGPS;
    UBXU4_t reserved2;
    UBXU4_t reserved3;
};

struct UBXNAV_CLOCK {
    UBXU4_t iTOW;
    UBXI4_t clkB;
    UBXI4_t clkD;
    UBXU4_t tAcc;
    UBXU4_t fAcc;
};

struct UBXNAV_DGPS {
    UBXU4_t iTOW;
    UBXI4_t age;
    UBXI2_t baseId;
    UBXI2_t baseHealth;
    UBXU1_t numCh;
    UBXU1_t status;
    UBXU2_t reserved1;
};

struct UBXDGPSFlags
{
    UBXX1_t channel:4;
    UBXX1_t dgpsUsed:1;
};

struct UBXNAV_DGPS_PART {
    UBXU1_t svid;
    struct UBXDGPSFlags flags;
    UBXU2_t ageC;
    UBXR4_t prc;
    UBXR4_t prrc;
};

struct UBXNAV_DOP {
    UBXU4_t iTOW;
    UBXU2_t gDOP;
    UBXU2_t pDOP;
    UBXU2_t tDOP;
    UBXU2_t vDOP;
    UBXU2_t hDOP;
    UBXU2_t nDOP;
    UBXU2_t eDOP;
};

struct UBXNAV_POSECEF {
    UBXU4_t iTOW;
    UBXI4_t ecefX;
    UBXI4_t ecefY;
    UBXI4_t ecefZ;
    UBXU4_t pAcc;
};

struct UBXNAV_POSLLH {
    UBXU4_t iTOW;
    UBXI4_t lon;
    UBXI4_t lat;
    UBXI4_t height;
    UBXI4_t hMSL;
    UBXU4_t hAcc;
    UBXU4_t vAcc;
};

struct UBXPVTFlags
{
    UBXX1_t gnssFixOk:1;
    UBXX1_t diffSoln:1;
    UBXX1_t psmState:3; //See UBXPVTPSMStates to fill this field
};

struct UBXNAV_PVT {
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
    struct UBXPVTFlags flags;
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
};

struct UBXNAV_SBAS {
    UBXU4_t iTOW;
    UBXU1_t geo;
    UBXU1_t mode;
    UBXI1_t sys;
    UBXX1_t service; //See UBXSBASService to fill this field
    UBXU1_t cnt;
    UBXU1_t reserved0[3];
    //Variable payload of UBXNAV_SBAS_PART type
};

struct UBXNAV_SBAS_PART {
    UBXU1_t svid;
    UBXU1_t flags;
    UBXU1_t udre;
    UBXU1_t svSys;
    UBXU1_t svService;
    UBXU1_t reserved1;
    UBXI2_t prc;
    UBXU2_t reserved2;
    UBXI2_t ic;
};

struct UBXNAV_SOL {
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
};

struct UBXNAV_STATUS {
    UBXU4_t iTOW;
    UBXU1_t gpsFix;
    UBXX1_t flags; //See UBXGPSFix to fill this field
    UBXX1_t fixStat; //See UBXSBASSOLFlags to fill this field
    UBXX1_t flags2;
    UBXU4_t ttff;
    UBXU4_t msss;
};

struct UBXSVINFOGlobalFlags
{
    UBXX1_t chipGen:3; //See UBXSVINFOChipGen to fill this field
};

struct UBXNAV_SVINFO {
    UBXU4_t iTOW;
    UBXU1_t numCh;
    struct UBXSVINFOGlobalFlags globalFlags;
    UBXU2_t reserved2;
    //Variable payload of UBXNAV_SVINFO_PART type
};

struct UBXSVINFOQuality
{
    UBXX1_t qualityInd:4; //See UBXSVINFOQualityId to fill this field
};

struct UBXNAV_SVINFO_PART {
    UBXU1_t chn;
    UBXU1_t svid;
    UBXX1_t flags; //See UBXSVINFOFlags to fill this field
    UBXX1_t quality;
    UBXU1_t cno;
    UBXI1_t elev;
    UBXI2_t azim;
    UBXI4_t prRes;
};

struct UBXNAV_TIMEGPS {
    UBXU4_t iTOW;
    UBXI4_t fTOW;
    UBXI2_t week;
    UBXI1_t leapS;
    UBXX1_t valid; //See UBXTIMEGPSValidityFlags to fill this field
    UBXU4_t tAcc;
};

struct UBXNAV_TIMEUTC {
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
};

struct UBXNAV_VELECEF {
    UBXU4_t iTOW;
    UBXI4_t ecefVX;
    UBXI4_t ecefVY;
    UBXI4_t ecefVZ;
    UBXU4_t sAcc;
};

struct UBXNAV_VELNED {
    UBXU4_t iTOW;
    UBXI4_t velN;
    UBXI4_t velE;
    UBXI4_t velD;
    UBXU4_t speed;
    UBXU4_t gSpeed;
    UBXI4_t heading;
    UBXU4_t sAcc;
    UBXU4_t cAcc;
};

struct UBXRXM_ALM_POLL {
    //No payload
};

struct UBXRXM_ALM_POLL_OPT {
    UBXU1_t svid;
};

struct UBXRXM_ALM {
    /* ###################################################
     * This RMX messages marked as obsolete API use AID instead
     */
    UBXU4_t svid;
    UBXU4_t week;
};

struct UBXRXM_ALM_OPT {
    /* ###################################################
     * This RMX messages marked as obsolete API use AID instead
     */
    UBXU4_t svid;
    UBXU4_t week;
    UBXU4_t dwrd[8];
};

struct UBXRXM_EPH_POLL {
    //No payload
};

struct UBXRXM_EPH_POLL_OPT {
    UBXU1_t svid;
};

struct UBXRXM_EPH {
    /* ###################################################
     * This RMX messages marked as obsolete API use AID instead
     */
    UBXU4_t svid;
    UBXU4_t how;
};

struct UBXRXM_EPH_OPT {
    /* ###################################################
     * This RMX messages marked as obsolete API use AID instead
     */
    UBXU4_t svid;
    UBXU4_t how;
    UBXU4_t sf1d[8];
    UBXU4_t sf2d[8];
    UBXU4_t sf3d[8];
};

struct UBXRXM_PMREQ {
    UBXU4_t duration;
    UBXX4_t flags; //See UBXPMREQFlags to fill this field
};

struct UBXRXM_RAW {
    UBXI4_t rcvTow;
    UBXI2_t week;
    UBXU1_t numSV;
    UBXU1_t reserved1;
    //Variable payload of UBXRXM_RAW_PART type
};

struct UBXRXM_RAW_PART
{
    UBXR8_t cpMes;
    UBXR8_t prMes;
    UBXR4_t doMes;
    UBXU1_t sv;
    UBXI1_t mesQI;
    UBXI1_t cno;
    UBXU1_t lli;
};

struct UBXRXM_SFRB {
    UBXU1_t chn;
    UBXU1_t svid;
    UBXX4_t dwrd[10];
};

struct UBXRXM_SVSI {
    UBXU4_t iTOW;
    UBXI2_t week;
    UBXU1_t numVis;
    UBXU1_t numSV;

};

struct UBXSVSISVFlags
{
    UBXX1_t ura:4;
    UBXX1_t healthy:1;
    UBXX1_t ephVal:1;
    UBXX1_t almVal:1;
    UBXX1_t notAvail:1;
};

struct UBXSVSIAge
{
    UBXX1_t almAge:4;
    UBXX1_t ephAge:4;
};

struct UBXRXM_SVSI_PART
{
    UBXU1_t svid;
    struct UBXSVSISVFlags svFlag;
    UBXI2_t azim;
    UBXI1_t elev;
    struct UBXSVSIAge age;
};


struct UBXTM2Flags
{
    UBXX1_t mode:1; //See UBXTM2FlagsMode to fill this field
    UBXX1_t run:1; //See UBXTM2FlagsRun to fill this field
    UBXX1_t newFallingEdge:1;
    UBXX1_t timeBase:2; //See UBXTM2FlagsTimeBase to fill this field
    UBXX1_t utc:1; //See UBXTM2FlagsUTC to fill this field
    UBXX1_t time:1; //See UBXTM2FlagsTime to fill this field
    UBXX1_t newRisingEdge:1;
};

struct UBXTIM_TM2 {
    UBXU1_t ch;
    struct UBXTM2Flags flags;
    UBXU2_t count;
    UBXU2_t wnR;
    UBXU2_t wnF;
    UBXU4_t towMsR;
    UBXU4_t towSubMsR;
    UBXU4_t towMsF;
    UBXU4_t towSubMsF;
    UBXU4_t accEst;
};

struct UBXTIM_TP {
    UBXU4_t towMS;
    UBXU4_t towSubMS;
    UBXI4_t qErr;
    UBXU2_t week;
    UBXX1_t flags;
    UBXU1_t reserved1;
};

struct UBXVRFYFlags
{
    UBXX1_t src:3; //See UBXVRFYFlagsSource to fill this field
};

struct UBXTIM_VRFY {
    UBXI4_t itow;
    UBXI4_t frac;
    UBXI4_t deltaMs;
    UBXI4_t deltaNs;
    UBXU2_t wno;
    struct UBXVRFYFlags flags;
    UBXU1_t reserved1;
};

union UBXMsgs
{
    struct UBXACK_NACK ACK_NACK;
    struct UBXACK_ACK ACK_ACK;
    struct UBXAID_ALM_POLL AID_ALM_POLL;
    struct UBXAID_ALM_POLL_OPT AID_ALM_POLL_OPT;
    struct UBXAID_ALM AID_ALM;
    struct UBXAID_ALM_OPT AID_ALM_OPT;
    struct UBXAID_ALPSRV AID_ALPSRV;
    struct UBXAID_ALP_POLL AID_ALP_POLL;
    struct UBXAID_ALP_END AID_ALP_END;
    struct UBXAID_ALP AID_ALP;
    struct UBXAID_AOP_POLL AID_AOP_POLL;
    struct UBXAID_AOP_POLL_OPT AID_AOP_POLL_OPT;
    struct UBXAID_AOP AID_AOP;
    struct UBXAID_AOP_OPT AID_AOP_OPT;
    struct UBXAID_DATA_POLL AID_DATA_POLL;
    struct UBXAID_EPH_POLL AID_EPH_POLL;
    struct UBXAID_EPH_POLL_OPT AID_EPH_POLL_OPT;
    struct UBXAID_EPH AID_EPH;
    struct UBXAID_EPH_OPT AID_EPH_OPT;
    struct UBXAID_HUI_POLL AID_HUI_POLL;
    struct UBXAID_HUI AID_HUI;
    struct UBXAID_INI_POLL AID_INI_POLL;
    struct UBXAID_INI AID_INI;
    struct UBXAID_REQ AID_REQ;
    struct UBXCFG_ANT CFG_ANT;
    struct UBXCFG_ANT_POLL CFG_ANT_POLL;
    struct UBXCFG_CFG CFG_CFG;
    struct UBXCFG_CFG_OPT CFG_CFG_OPT;
    struct UBXCFG_DAT_IN CFG_DAT_IN;
    struct UBXCFG_DAT_OUT CFG_DAT_OUT;
    struct UBXCFG_DAT_POLL CFG_DAT_POLL;
    struct UBXCFG_GNSS_POLL CFG_GNSS_POLL;
    struct UBXCFG_GNSS CFG_GNSS;
    struct UBXCFG_INF_POLL CFG_INF_POLL;
    struct UBXCFG_INF CFG_INF;
    struct UBXCFG_ITFM_POLL CFG_ITFM_POLL;
    struct UBXCFG_ITFM CFG_ITFM;
    struct UBXCFG_LOGFILTER_POLL CFG_LOGFILTER_POLL;
    struct UBXCFG_LOGFILTER CFG_LOGFILTER;
    struct UBXCFG_MSG_POLL CFG_MSG_POLL;
    struct UBXCFG_MSG_RATE CFG_MSG_RATE;
    struct UBXCFG_MSG_RATES CFG_MSG_RATES;
    struct UBXCFG_NAV5_POLL CFG_NAV5_POLL;
    struct UBXCFG_NAV5 CFG_NAV5;
    struct UBXCFG_NAVX5_POLL CFG_NAVX5_POLL;
    struct UBXCFG_NAVX5 CFG_NAVX5;
    struct UBXCFG_NMEA_POLL CFG_NMEA_POLL;
    struct UBXCFG_NMEA CFG_NMEA;
    struct UBXCFG_NVS CFG_NVS;
    struct UBXCFG_PM2_POLL CFG_PM2_POLL;
    struct UBXCFG_PM2 CFG_PM2;
    struct UBXCFG_PRT_POLL CFG_PRT_POLL;
    struct UBXCFG_PRT_POLL_OPT CFG_PRT_POLL_OPT;
    struct UBXCFG_PRT CFG_PRT;
    struct UBXCFG_RATE_POLL CFG_RATE_POLL;
    struct UBXCFG_RATE CFG_RATE;
    struct UBXCFG_RINV CFG_RINV;
    struct UBXCFG_RINV_POLL CFG_RINV_POLL;
    struct UBXCFG_RST CFG_RST;
    struct UBXCFG_RXM CFG_RXM;
    struct UBXCFG_RXM_POLL CFG_RXM_POLL;
    struct UBXCFG_SBAS_POLL CFG_SBAS_POLL;
    struct UBXCFG_SBAS CFG_SBAS;
    struct UBXCFG_TP5_POLL CFG_TP5_POLL;
    struct UBXCFG_TP5_POLL_OPT CFG_TP5_POLL_OPT;
    struct UBXCFG_TP5 CFG_TP5;
    struct UBXCFG_USB_POLL CFG_USB_POLL;
    struct UBXCFG_USB CFG_USB;
    struct UBXINF_DEBUG INF_DEBUG;
    struct UBXINF_ERROR INF_ERROR;
    struct UBXINF_NOTICE INF_NOTICE;
    struct UBXINF_TEST INF_TEST;
    struct UBXINF_WARNING INF_WARNING;
    struct UBXLOG_CREATE LOG_CREATE;
    struct UBXLOG_ERASE LOG_ERASE;
    struct UBXLOG_FINDTIME_IN LOG_FINDTIME_IN;
    struct UBXLOG_FINDTIME_OUT LOG_FINDTIME_OUT;
    struct UBXLOG_INFO_POLL LOG_INFO_POLL;
    struct UBXLOG_INFO LOG_INFO;
    struct UBXLOG_RETRIEVEPOS LOG_RETRIEVEPOS;
    struct UBXLOG_RETRIEVESTRING LOG_RETRIEVESTRING;
    struct UBXLOG_RETRIEVE LOG_RETRIEVE;
    struct UBXLOG_STRING LOG_STRING;
    struct UBXMON_HW2 MON_HW2;
    struct UBXMON_HW MON_HW;
    struct UBXMON_IO MON_IO;
    struct UBXMON_MSGPP MON_MSGPP;
    struct UBXMON_RXBUF MON_RXBUF;
    struct UBXMON_RXR MON_RXR;
    struct UBXMON_TXBUF MON_TXBUF;
    struct UBXMON_VER_POLL MON_VER_POLL;
    struct UBXMON_VER MON_VER;
    struct UBXNAV_AOPSTATUS NAV_AOPSTATUS;
    struct UBXNAV_CLOCK NAV_CLOCK;
    struct UBXNAV_DGPS NAV_DGPS;
    struct UBXNAV_DOP NAV_DOP;
    struct UBXNAV_POSECEF NAV_POSECEF;
    struct UBXNAV_POSLLH NAV_POSLLH;
    struct UBXNAV_PVT NAV_PVT;
    struct UBXNAV_SBAS NAV_SBAS;
    struct UBXNAV_SOL NAV_SOL;
    struct UBXNAV_STATUS NAV_STATUS;
    struct UBXNAV_SVINFO NAV_SVINFO;
    struct UBXNAV_TIMEGPS NAV_TIMEGPS;
    struct UBXNAV_TIMEUTC NAV_TIMEUTC;
    struct UBXNAV_VELECEF NAV_VELECEF;
    struct UBXNAV_VELNED NAV_VELNED;
    struct UBXRXM_ALM_POLL RXM_ALM_POLL;
    struct UBXRXM_ALM_POLL_OPT RXM_ALM_POLL_OPT;
    struct UBXRXM_ALM_OPT RXM_ALM_OPT;
    struct UBXRXM_EPH_POLL RXM_EPH_POLL;
    struct UBXRXM_EPH_POLL_OPT RXM_EPH_POLL_OPT;
    struct UBXRXM_EPH RXM_EPH;
    struct UBXRXM_EPH_OPT RXM_EPH_OPT;
    struct UBXRXM_PMREQ RXM_PMREQ;
    struct UBXRXM_RAW RXM_RAW;
    struct UBXRXM_SFRB RXM_SFRB;
    struct UBXRXM_SVSI RXM_SVSI;
    struct UBXTIM_TM2 TIM_TM2;
    struct UBXTIM_TP TIM_TP;
    struct UBXTIM_VRFY TIM_VRFY;
};

struct UBXAlpFileInfo
{
    int fileId;
    char* alpData;
    int dataSize;
};

struct UBXMsg
{
    u_int16_t preamble;
    struct UBXHdr hdr;
    union UBXMsgs payload;
};

struct UBXMsgBuffer
{
    int size;
    char* data;
};

#pragma pack(pop)

#endif // UBXMESSAGE_H
