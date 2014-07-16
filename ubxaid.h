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
 * File: ubxaid.h
 */
/*! \file */
#ifndef UBLOXAID_H
#define UBLOXAID_H

#include "ubxmessage.h"
#ifdef __cplusplus
extern "C"
{
#endif

extern UBXMsgBuffer getAID_ALM_POLL();
extern UBXMsgBuffer getAID_ALM_POLL_OPT(UBXU1_t svid);
extern UBXMsgBuffer getAID_ALM(UBXU4_t svid, UBXU4_t week);
extern UBXMsgBuffer getAID_ALM_OPT(UBXU4_t svid, UBXU4_t week, UBXU4_t dwrd[8]);
extern UBXMsgBuffer getAID_ALPSRV(UBXMsg* clientMgs, const UBXAlpFileInfo *fileInfo);
extern UBXMsgBuffer getAID_ALP_POLL(UBXU4_t predTow, UBXU4_t predDur, UBXI4_t age, UBXU2_t predWno, UBXU2_t almWno, UBXU1_t svs);
extern UBXMsgBuffer getAID_ALP_END();
extern UBXMsgBuffer getAID_ALP(UBXU2_t *chunk, int chunkSize);
extern UBXMsgBuffer getAID_AOP_POLL();
extern UBXMsgBuffer getAID_AOP_POLL_OPT(UBXU1_t svid);
extern UBXMsgBuffer getAID_AOP(UBXU1_t svid, UBXU1_t data[59]);
extern UBXMsgBuffer getAID_AOP_OPT(UBXU1_t svid, UBXU1_t data[59], UBXU1_t optional0[48], UBXU1_t optional1[48], UBXU1_t optional2[48]);
extern UBXMsgBuffer getAID_DATA_POLL();
extern UBXMsgBuffer getAID_EPH_POLL();
extern UBXMsgBuffer getAID_EPH_POLL_OPT(UBXU1_t svid);
extern UBXMsgBuffer getAID_EPH(UBXU4_t svid, UBXU4_t how);
extern UBXMsgBuffer getAID_EPH_OPT(UBXU4_t svid, UBXU4_t how, UBXU4_t sf1d[8], UBXU4_t sf2d[8], UBXU4_t sf3d[8]);
extern UBXMsgBuffer getAID_HUI_POLL();
extern UBXMsgBuffer getAID_HUI(UBXI4_t health,
                                      UBXR4_t utcA0,
                                      UBXR4_t utcA1,
                                      UBXI4_t utcTOW,
                                      UBXI2_t utcWNT,
                                      UBXI2_t utcLS,
                                      UBXI2_t utcWNF,
                                      UBXI2_t utcDN,
                                      UBXI2_t utcLSF,
                                      UBXI2_t utcSpare,
                                      UBXR4_t klobA0,
                                      UBXR4_t klobA1,
                                      UBXR4_t klobA2,
                                      UBXR4_t klobA3,
                                      UBXR4_t klobB0,
                                      UBXR4_t klobB1,
                                      UBXR4_t klobB2,
                                      UBXR4_t klobB3,
                                      UBXX2_t flags //See UBXHUIFlags to fill this field
                                      );
extern UBXMsgBuffer getAID_INI_POLL();
extern UBXMsgBuffer getAID_INI(UBXI1_t ecefXOrLat,
                                      UBXI1_t ecefYOrLat,
                                      UBXI1_t ecefZOrLat,
                                      UBXU1_t posAcc,
                                      UBXI1_t tmCfg, //See UBXINItmCfg to fill this field
                                      UBXU2_t wnoOrDate,
                                      UBXU4_t towOrDate,
                                      UBXI4_t towNs,
                                      UBXU4_t tAccMS,
                                      UBXU4_t tAccNS,
                                      UBXI4_t clkDOrFreq,
                                      UBXU4_t clkDAccOrFreqAcc,
                                      UBXX4_t flags //See UBXINIFlags to fill this field
                                      );

#ifdef __cplusplus
}
#endif

#endif // UBLOXAID_H
