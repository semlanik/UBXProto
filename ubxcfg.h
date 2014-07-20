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
 * static link this library and MUST add link to author
 * and source of this library in your application.
 *
 * Actual LGPL text https://www.gnu.org/licenses/lgpl.html
 *
 * File: ubxcfg.h
 */
/*! \file */
#ifndef UBLOXCFG_H
#define UBLOXCFG_H

#include "ubxmessage.h"
#ifdef __cplusplus
extern "C"
{
#endif

extern UBXMsgBuffer getCFG_ANT(UBXX2_t flags, //See UBXANTFlags to fill this field
                                      UBXANTPins pins);
extern UBXMsgBuffer getCFG_ANT_POLL();
extern UBXMsgBuffer getCFG_CFG(UBXX4_t clearMask, //See UBXCFGMask to fill this field
                                      UBXX4_t saveMask, //See UBXCFGMask to fill this field
                                      UBXX4_t loadMask //See UBXCFGMask to fill this field
                                      );
extern UBXMsgBuffer getCFG_CFG_OPT(UBXX4_t clearMask, //See UBXCFGMask to fill this field
                                          UBXX4_t saveMask, //See UBXCFGMask to fill this field
                                          UBXX4_t loadMask, //See UBXCFGMask to fill this field
                                          UBXX1_t deviceMask //See UBXCFGDeviceMask to fill this field
                                          );
extern UBXMsgBuffer getCFG_DAT_IN(UBXR8_t majA,
                                         UBXR8_t flat,
                                         UBXR4_t dX,
                                         UBXR4_t dY,
                                         UBXR4_t dZ,
                                         UBXR4_t rotX,
                                         UBXR4_t rotY,
                                         UBXR4_t rotZ,
                                         UBXR4_t scale);
extern UBXMsgBuffer getCFG_DAT_POLL();
extern UBXMsgBuffer getCFG_GNSS_POLL();
extern UBXMsgBuffer getCFG_GNSS(UBXU1_t msgVer,
                                       UBXU1_t numTrkChHw,
                                       UBXU1_t numTrkChUse,
                                       UBXU1_t numConfigBlocks,
                                       UBXCFG_GNSS_PART* gnssPart);
extern UBXMsgBuffer getCFG_INF_POLL(UBXU1_t protocolId);
extern UBXMsgBuffer getCFG_INF(UBXCFG_INF_PART *infPart, int infPartCount);
extern UBXMsgBuffer getCFG_ITFM_POLL();
extern UBXMsgBuffer getCFG_ITFM(UBXITFMConfig config, UBXITFMConfig2 config2);
extern UBXMsgBuffer getCFG_LOGFILTER_POLL();
extern UBXMsgBuffer getCFG_LOGFILTER(UBXU1_t version,
                                            UBXX1_t flags, //See UBXLOGFILTERFlags to fill this field
                                            UBXU2_t minIterval,
                                            UBXU2_t timeThreshold,
                                            UBXU2_t speedThreshold,
                                            UBXU4_t positionThreshold);
extern UBXMsgBuffer getCFG_MSG_POLL(UBXMessageClass msgClass, UBXMessageId msgId);
extern UBXMsgBuffer getCFG_MSG_RATE(UBXMessageClass msgClass, UBXMessageId msgId, UBXU1_t rate);
extern UBXMsgBuffer getCFG_MSG_RATES(UBXMessageClass msgClass, UBXMessageId msgId, UBXU1_t rate[6]);
extern UBXMsgBuffer getCFG_NAV5_POLL();
extern UBXMsgBuffer getCFG_NAV5(UBXX2_t mask, //See UBXNAV5Mask to fill this field
                                       UBXNAV5Model dynModel,
                                       UBXNAV5FixMode fixMode,
                                       UBXI4_t fixedAlt,
                                       UBXU4_t fixedAltVar,
                                       UBXI1_t minElev,
                                       UBXU2_t pDop,
                                       UBXU2_t tDop,
                                       UBXU2_t pAcc,
                                       UBXU2_t tAcc,
                                       UBXU1_t staticHoldThresh,
                                       UBXU1_t dgpsTimeOut,
                                       UBXU1_t cnoThreshNumSVs,
                                       UBXU1_t cnoThresh);
extern UBXMsgBuffer getCFG_NAVX5_POLL();
extern UBXMsgBuffer getCFG_NAVX5(UBXU2_t version,
                                        UBXX2_t mask1,//See UBXNAVX5Mask to fill this field
                                        UBXU1_t minSVs,
                                        UBXU1_t maxSVs,
                                        UBXU1_t minCNO,
                                        UBXU1_t iniFix3D,
                                        UBXU2_t wknRollover,
                                        UBXU1_t usePPP,
                                        UBXU1_t aopCFG,// 0-disabled, 1 - enabled
                                        UBXU1_t aopOrbMaxErr);
extern UBXMsgBuffer getCFG_NMEA_POLL();
extern UBXMsgBuffer getCFG_NMEA(UBXX1_t filter, //See UBXNMEAFilter to fill this field
                                       UBXU1_t nmeaVersion,
                                       UBXU1_t numSV,
                                       UBXX1_t flags, //See UBXNMEAFlags to fill this field
                                       UBXX4_t gnssToFilter, //See UBXNMEAGNSSToFilter to fill this field
                                       UBXNMEASVNumbering svNumbering,
                                       UBXNMEATalkerIds mainTalkerId,
                                       UBXNMEAGSVTalkerIds gsvTalkerId);
extern UBXMsgBuffer getCFG_NVS(UBXX4_t clearMask, //See UBXCFGMask CFG_NVS section to fill this field
                                      UBXX4_t saveMask, //See UBXCFGMask CFG_NVS section to fill this field
                                      UBXX4_t loadMask, //See UBXCFGMask CFG_NVS section to fill this field
                                      UBXX1_t deviceMask //See UBXCFGDeviceMask to fill this field
                                      );
extern UBXMsgBuffer getCFG_PM2_POLL();
extern UBXMsgBuffer getCFG_PM2(UBXCFG_PM2Flags flags,
                                      UBXU4_t updatePeriod,
                                      UBXU4_t searchPeriod,
                                      UBXU4_t gridOffset,
                                      UBXU2_t onTime,
                                      UBXU2_t minAcqTime);
extern UBXMsgBuffer getCFG_PRT_POLL();
extern UBXMsgBuffer getCFG_PRT_POLL_OPT(UBXU1_t portId);
extern UBXMsgBuffer getCFG_PRT_UART();
extern UBXMsgBuffer getCFG_PRT_USB();
extern UBXMsgBuffer getCFG_PRT_SPI();
extern UBXMsgBuffer getCFG_PRT_DDC();
extern UBXMsgBuffer getCFG_RATE_POLL();
extern UBXMsgBuffer getCFG_RATE(UBXU2_t measRate, UBXU2_t navRate, UBXU2_t timeRef);
extern UBXMsgBuffer getCFG_RINV(UBXX1_t flags, UBXU1_t *data, int dataSize);
extern UBXMsgBuffer getCFG_RINV_POLL();
extern UBXMsgBuffer getCFG_RST(int mode, UBXU2_t mask);
extern UBXMsgBuffer getCFG_RST_OPT(int mode, UBXBBRSpecialSets special);
extern UBXMsgBuffer getCFG_RXM(UBXU1_t lpMode //See UBXRXMLowPowerModes to fill this field
                                      );
extern UBXMsgBuffer getCFG_RXM_POLL();
extern UBXMsgBuffer getCFG_SBAS_POLL();
extern UBXMsgBuffer getCFG_SBAS(UBXX1_t mode, //See UBXSBASModes to fill this field
                                       UBXX1_t usage, //See UBXSBASUsage to fill this field
                                       UBXU1_t maxSBAS,
                                       UBXX1_t scanmode2, //See UBXSBASScanModes2 to fill this field
                                       UBXX4_t scanmode1 //See UBXSBASScanModes1 to fill this fields
                                       );
extern UBXMsgBuffer getCFG_TP5_POLL();
extern UBXMsgBuffer getCFG_TP5_POLL_OPT(UBXCFGTimepulses tpIdx);
extern UBXMsgBuffer getCFG_TP5(UBXCFGTimepulses tpIdx, UBXI2_t antCableDelay,  UBXI2_t rfGroupDelay,
                                      UBXU4_t freqPeriod, UBXU4_t freqPeriodLock, UBXU4_t pulseLenRatio,
                                      UBXU4_t pulseLenRatioLock, UBXU4_t userConfigDelay, UBXU4_t flags);
extern UBXMsgBuffer getCFG_USB_POLL();
extern UBXMsgBuffer getCFG_USB(UBXU2_t vendorId,
                                      UBXU2_t productId,
                                      UBXU2_t powerConsumption,
                                      UBXX2_t flags, //See UBXUSBFlags to fill this field
                                      UBXCH_t* vendorString,
                                      UBXCH_t* productString,
                                      UBXCH_t* serialNumber);

#ifdef __cplusplus
}
#endif

#endif // UBLOXCFG_H
