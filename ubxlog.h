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
 * File: ubxlog.h
 */
/*! \file */

#ifndef UBLOXLOG_H
#define UBLOXLOG_H

#include "ubxmessage.h"
#ifdef __cplusplus
extern "C"
{
#endif


extern UBXMsgBuffer getLOG_CREATE(UBXX1_t logCfg, //See UBXLOGCfg
                                         UBXU1_t logSize,
                                         UBXU4_t userDefinedSize);
extern UBXMsgBuffer getLOG_ERASE();
extern UBXMsgBuffer getLOG_FINDTIME_IN(UBXU2_t year,
                                              UBXU1_t month,
                                              UBXU1_t day,
                                              UBXU1_t hour,
                                              UBXU1_t minute,
                                              UBXU1_t second);
extern UBXMsgBuffer getLOG_INFO_POLL();
extern UBXMsgBuffer getLOG_RETRIEVE(UBXU4_t startNumber,
                                           UBXU4_t entryCount,
                                           UBXU1_t version);
extern UBXMsgBuffer getLOG_STRING();

#ifdef __cplusplus
}
#endif

#endif // UBLOXLOG_H
