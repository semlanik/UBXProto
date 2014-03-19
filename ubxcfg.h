/*
 * ubloxproto
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
 * File: ubloxcfg.h
 */

#ifndef UBLOXCFG_H
#define UBLOXCFG_H

#include "ubxmessage.h"
#ifdef __cplusplus
extern "C"
{
#endif

extern struct UBXMsgBuffer getCFG_ANT();
extern struct UBXMsgBuffer getCFG_ANT_POLL();
extern struct UBXMsgBuffer getCFG_CFG();
extern struct UBXMsgBuffer getCFG_CFG_OPT();
extern struct UBXMsgBuffer getCFG_DAT_IN();
extern struct UBXMsgBuffer getCFG_DAT_POLL();
extern struct UBXMsgBuffer getCFG_GNSS_POLL();
extern struct UBXMsgBuffer getCFG_GNSS();
extern struct UBXMsgBuffer getCFG_INF_POLL(UBXU1_t protocolId);
extern struct UBXMsgBuffer getCFG_INF();
extern struct UBXMsgBuffer getCFG_ITFM_POLL();
extern struct UBXMsgBuffer getCFG_ITFM();
extern struct UBXMsgBuffer getCFG_LOGFILTER_POLL();
extern struct UBXMsgBuffer getCFG_LOGFILTER();
extern struct UBXMsgBuffer getCFG_MSG_POLL(enum UBXMessageClass msgClass, enum UBXMessageId msgId);
extern struct UBXMsgBuffer getCFG_MSG_RATE(enum UBXMessageClass msgClass, enum UBXMessageId msgId, u_int8_t rate);
extern struct UBXMsgBuffer getCFG_MSG_RATES(enum UBXMessageClass msgClass, enum UBXMessageId msgId, u_int8_t rate[6]);
extern struct UBXMsgBuffer getCFG_NAV5_POLL();
extern struct UBXMsgBuffer getCFG_NAV5();
extern struct UBXMsgBuffer getCFG_NAVX5_POLL();
extern struct UBXMsgBuffer getCFG_NAVX5();
extern struct UBXMsgBuffer getCFG_NMEA_POLL();
extern struct UBXMsgBuffer getCFG_NMEA();
extern struct UBXMsgBuffer getCFG_NVS();
extern struct UBXMsgBuffer getCFG_PM2_POLL();
extern struct UBXMsgBuffer getCFG_PM2();
extern struct UBXMsgBuffer getCFG_PRT_POLL();
extern struct UBXMsgBuffer getCFG_PRT_POLL_OPT(UBXU1_t portId);
extern struct UBXMsgBuffer getCFG_PRT();
extern struct UBXMsgBuffer getCFG_RATE_POLL();
extern struct UBXMsgBuffer getCFG_RATE();
extern struct UBXMsgBuffer getCFG_RINV();
extern struct UBXMsgBuffer getCFG_RINV_POLL();
extern struct UBXMsgBuffer getCFG_RST(enum UBXResetMode mode, u_int16_t mask);
extern struct UBXMsgBuffer getCFG_RXM();
extern struct UBXMsgBuffer getCFG_RXM_POLL();
extern struct UBXMsgBuffer getCFG_SBAS_POLL();
extern struct UBXMsgBuffer getCFG_SBAS();
extern struct UBXMsgBuffer getCFG_TP5_POLL();
extern struct UBXMsgBuffer getCFG_TP5_POLL_OPT(enum UBXCFGTimepulses tpIdx);
extern struct UBXMsgBuffer getCFG_TP5(enum UBXCFGTimepulses tpIdx,
                        int16_t antCableDelay,
                        int16_t rfGroupDelay,
                        u_int32_t freqPeriod,
                        u_int32_t freqPeriodLock,
                        u_int32_t pulseLenRatio,
                        u_int32_t pulseLenRatioLock,
                        int32_t userConfigDelay,
                        int32_t flags);
extern struct UBXMsgBuffer getCFG_USB_POLL();
extern struct UBXMsgBuffer getCFG_USB();

#ifdef __cplusplus
}
#endif

#endif // UBLOXCFG_H
