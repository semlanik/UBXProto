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
 * File: ubloxaid.h
 */

#ifndef UBLOXAID_H
#define UBLOXAID_H

#include "ubxmessage.h"
#ifdef __cplusplus
extern "C"
{
#endif

extern struct UBXMsgBuffer getAID_ALM_POLL();
extern struct UBXMsgBuffer getAID_ALM_POLL_OPT(UBXU1_t svid);
extern struct UBXMsgBuffer getAID_ALM(UBXU4_t svid, UBXU4_t week);
extern struct UBXMsgBuffer getAID_ALM_OPT(UBXU4_t svid, UBXU4_t week, UBXU4_t dwrd[8]);
extern struct UBXMsgBuffer getAID_ALPSRV(struct UBXMsg* clientMgs, const struct UBXAlpFileInfo *fileInfo);
extern struct UBXMsgBuffer getAID_ALP_POLL(UBXU4_t predTow, UBXU4_t predDur, UBXI4_t age, UBXU2_t predWno, UBXU2_t almWno, UBXU1_t svs);
extern struct UBXMsgBuffer getAID_ALP_END();
extern struct UBXMsgBuffer getAID_ALP();
extern struct UBXMsgBuffer getAID_AOP_POLL();
extern struct UBXMsgBuffer getAID_AOP_POLL_OPT(UBXU1_t svid);
extern struct UBXMsgBuffer getAID_AOP();
extern struct UBXMsgBuffer getAID_AOP_OPT();
extern struct UBXMsgBuffer getAID_DATA_POLL();
extern struct UBXMsgBuffer getAID_EPH_POLL();
extern struct UBXMsgBuffer getAID_EPH_POLL_OPT(UBXU1_t svid);
extern struct UBXMsgBuffer getAID_EPH();
extern struct UBXMsgBuffer getAID_EPH_OPT();
extern struct UBXMsgBuffer getAID_HUI_POLL();
extern struct UBXMsgBuffer getAID_HUI();
extern struct UBXMsgBuffer getAID_INI_POLL();
extern struct UBXMsgBuffer getAID_INI();
extern struct UBXMsgBuffer getAID_REQ();

#ifdef __cplusplus
}
#endif

#endif // UBLOXAID_H
