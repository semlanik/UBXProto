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
 * File: ubxrxm.h
 */
/*! \file */

#ifndef UBLOXRXM_H
#define UBLOXRXM_H

#include "ubxmessage.h"
#ifdef __cplusplus
extern "C"
{
#endif

extern UBXMsgBuffer getRXM_ALM_POLL();
extern UBXMsgBuffer getRXM_ALM_POLL_OPT(UBXU1_t svid);
extern UBXMsgBuffer getRXM_EPH_POLL();
extern UBXMsgBuffer getRXM_EPH_POLL_OPT(UBXU1_t svid);
extern UBXMsgBuffer getRXM_PMREQ(UBXU4_t duration,
                                        UBXX4_t flags //See UBXPMREQFlags to fill this field
                                        );
extern UBXMsgBuffer getRXM_SVSI(UBXU4_t iTOW,
                                       UBXI2_t week,
                                       UBXU1_t numVis,
                                       UBXU1_t numSV,
                                       UBXRXM_SVSI_PART* svsiPart,
                                       int svsiPartCount);

#ifdef __cplusplus
}
#endif

#endif // UBLOXRXM_H
