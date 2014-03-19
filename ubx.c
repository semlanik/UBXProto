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
 * File: ubx.c
 */

#include "ubxmessage.h"
#include "ubx.h"
#include "malloc.h"
#include "memory.h"

void fletcherChecksum(unsigned char* buffer, int size, unsigned char* checkSumA, unsigned char* checkSumB)
{
    int i = 0;
    *checkSumA = 0;
    *checkSumB = 0;
    for(; i < size; i++)
    {
        *checkSumA += buffer[i];
        *checkSumB += *checkSumA;
    }
}

void completeMsg(struct UBXMsgBuffer* buffer, int payloadSize)
{
    unsigned char* checkSumA = (unsigned char*)(buffer->data + UBX_HEADER_SIZE  + payloadSize);
    unsigned char* checkSumB = (unsigned char*)(buffer->data + UBX_HEADER_SIZE  + payloadSize + 1);
    fletcherChecksum((unsigned char*)(buffer->data + sizeof(UBX_PREAMBLE)), payloadSize + 4, checkSumA, checkSumB);
}

void initMsg(struct UBXMsg* msg, int payloadSize, enum UBXMessageClass msgClass, enum UBXMessageId msgId)
{
    msg->preamble = htobe16(UBX_PREAMBLE);
    msg->hdr.msgClass = msgClass;
    msg->hdr.msgId = msgId;
    msg->hdr.length = payloadSize;
}

struct UBXMsgBuffer createBuffer(int payloadSize)
{
    struct UBXMsgBuffer buffer = {0, 0};
    buffer.size = UBX_HEADER_SIZE + payloadSize + UBX_CHECKSUM_SIZE;
    buffer.data = malloc(buffer.size);
    memset(buffer.data, 0, buffer.size);
    return buffer;
}

struct UBXMsgBuffer getAID_ALPSRV(struct UBXMsg* clientMgs, const struct UBXAlpFileInfo *fileInfo)
{
    int requestedAlpSize = (clientMgs->payload.AID_ALPSRV.size << 1);
    if(fileInfo->dataSize < (clientMgs->payload.AID_ALPSRV.offset + requestedAlpSize))
    {
        requestedAlpSize = fileInfo->dataSize - clientMgs->payload.AID_ALPSRV.offset - 1;
    }
    int alpMsgSize = sizeof(struct UBXAID_ALPSRV);
    int payloadSize = alpMsgSize + requestedAlpSize;
    struct UBXMsgBuffer buffer  = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*) buffer.data;

    if(requestedAlpSize < 0)
    {
        return buffer;
    }

    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_ALPSRV);
    msg->payload.AID_ALPSRV.idSize = clientMgs->payload.AID_ALPSRV.idSize;
    msg->payload.AID_ALPSRV.type = clientMgs->payload.AID_ALPSRV.type;
    msg->payload.AID_ALPSRV.offset = clientMgs->payload.AID_ALPSRV.offset;
    msg->payload.AID_ALPSRV.size = clientMgs->payload.AID_ALPSRV.size;
    msg->payload.AID_ALPSRV.fileId = fileInfo->fileId;
    msg->payload.AID_ALPSRV.dataSize = requestedAlpSize;
    msg->payload.AID_ALPSRV.id1 = clientMgs->payload.AID_ALPSRV.id1;
    msg->payload.AID_ALPSRV.id2 = clientMgs->payload.AID_ALPSRV.id2;
    msg->payload.AID_ALPSRV.id3 = clientMgs->payload.AID_ALPSRV.id3;
    memcpy(buffer.data + UBX_HEADER_SIZE + alpMsgSize, fileInfo->alpData + msg->payload.AID_ALPSRV.offset, requestedAlpSize);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_MSG_POLL(enum UBXMessageClass msgClass, enum UBXMessageId msgId)
{
    int payloadSize = sizeof(struct UBXCFG_MSG_POLL);
    struct UBXMsgBuffer buffer  = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_MSG);
    msg->payload.CFG_MSG_POLL.msgClass = msgClass;
    msg->payload.CFG_MSG_POLL.msgId = msgId;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_MSG_RATE(enum UBXMessageClass msgClass, enum UBXMessageId msgId, UBXU1_t rate)
{
    int payloadSize = sizeof(struct UBXCFG_MSG_RATE);
    struct UBXMsgBuffer buffer  = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_MSG);
    msg->payload.CFG_MSG_RATE.msgClass = msgClass;
    msg->payload.CFG_MSG_RATE.msgId = msgId;
    msg->payload.CFG_MSG_RATE.rate = rate;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_MSG_RATES(enum UBXMessageClass msgClass, enum UBXMessageId msgId, UBXU1_t rate[])
{
    int payloadSize = sizeof(struct UBXCFG_MSG_RATES);
    struct UBXMsgBuffer buffer  = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_MSG);
    msg->payload.CFG_MSG_RATES.msgClass = msgClass;
    msg->payload.CFG_MSG_RATES.msgId = msgId;
    memcpy(msg->payload.CFG_MSG_RATES.rate, rate, 6*sizeof(u_int8_t));
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_RST(enum UBXResetMode mode, u_int16_t mask)
{
    int payloadSize = sizeof(struct UBXCFG_RST);
    struct UBXMsgBuffer buffer  = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_RST);
    msg->payload.CFG_RST.resetMode = mode;
    msg->payload.CFG_RST.navBBRMask = mask;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_TP5_POLL_OPT(enum UBXCFGTimepulses tpIdx)
{
    int payloadSize = sizeof(struct UBXCFG_TP5_POLL_OPT);
    struct UBXMsgBuffer buffer  = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_TP5);
    msg->payload.CFG_TP5_POLL_OPT.tpIdx = tpIdx;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_TP5(enum UBXCFGTimepulses tpIdx,
                               int16_t antCableDelay,
                               int16_t rfGroupDelay,
                               u_int32_t freqPeriod,
                               u_int32_t freqPeriodLock,
                               u_int32_t pulseLenRatio,
                               u_int32_t pulseLenRatioLock,
                               int32_t userConfigDelay,
                               int32_t flags)
{
    int payloadSize = sizeof(struct UBXCFG_TP5);
    struct UBXMsgBuffer buffer  = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_TP5);
    msg->payload.CFG_TP5.tpIdx = tpIdx;
    msg->payload.CFG_TP5.antCableDelay = antCableDelay;
    msg->payload.CFG_TP5.rfGroupDelay = rfGroupDelay;
    msg->payload.CFG_TP5.freqPeriod = freqPeriod;
    msg->payload.CFG_TP5.freqPeriodLock = freqPeriodLock;
    msg->payload.CFG_TP5.pulseLenRatio = pulseLenRatio;
    msg->payload.CFG_TP5.pulseLenRatioLock = pulseLenRatioLock;
    msg->payload.CFG_TP5.userConfigDelay = userConfigDelay;
    msg->payload.CFG_TP5.flags = flags;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_ALM_POLL()
{
    int payloadSize = sizeof(struct UBXAID_ALM_POLL);
    struct UBXMsgBuffer buffer  = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_ALP);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_ALM_POLL_OPT(UBXU1_t svid)
{
    int payloadSize = sizeof(struct UBXAID_ALM_POLL_OPT);
    struct UBXMsgBuffer buffer  = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_ALP);
    msg->payload.AID_ALM_POLL_OPT.svid = svid;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_ALM(UBXU4_t svid, UBXU4_t week)
{
    int payloadSize = sizeof(struct UBXAID_ALM);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_ALM);
    msg->payload.AID_ALM.svid = svid;
    msg->payload.AID_ALM.week = week;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_ALM_OPT(UBXU4_t svid, UBXU4_t week, UBXU4_t dwrd[8])
{
    int payloadSize = sizeof(struct UBXAID_ALM_OPT);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_ALM);
    msg->payload.AID_ALM_OPT.svid = svid;
    msg->payload.AID_ALM_OPT.week = week;
    memcpy(msg->payload.AID_ALM_OPT.dwrd, dwrd, 8*sizeof(UBXU4_t));
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_ALP_POLL(UBXU4_t predTow,
                                    UBXU4_t predDur,
                                    UBXI4_t age,
                                    UBXU2_t predWno,
                                    UBXU2_t almWno,
                                    UBXU1_t svs)
{
    int payloadSize = sizeof(struct UBXAID_ALP_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_ALP);
    msg->payload.AID_ALP_POLL.predTow = predTow;
    msg->payload.AID_ALP_POLL.predDur = predDur;
    msg->payload.AID_ALP_POLL.age = age;
    msg->payload.AID_ALP_POLL.predWno = predWno;
    msg->payload.AID_ALP_POLL.almWno = almWno;
    msg->payload.AID_ALP_POLL.svs = svs;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_ALP_END()
{
    int payloadSize = sizeof(struct UBXAID_ALP_END);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_ALP);
    msg->payload.AID_ALP_END.dummy = 0xAA;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_ALP(UBXU2_t* chunk, int chunkSize)
{
    int payloadSize = sizeof(struct UBXAID_ALP) + chunkSize;
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_ALP);
    memcpy(&(msg->payload) + sizeof(struct UBXAID_ALP), chunk, chunkSize);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_AOP_POLL()
{
    int payloadSize = sizeof(struct UBXAID_AOP_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_AOP);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_AOP_POLL_OPT(UBXU1_t svid)
{
    int payloadSize = sizeof(struct UBXAID_AOP_POLL_OPT);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_AOP);
    msg->payload.AID_AOP_POLL_OPT.svid = svid;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_AOP(UBXU1_t svid, UBXU1_t data[59])
{
    int payloadSize = sizeof(struct UBXAID_AOP);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_AOP);
    msg->payload.AID_AOP.svid = svid;
    memcpy(msg->payload.AID_AOP.data, data, 59*sizeof(UBXU1_t));
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_AOP_OPT(UBXU1_t svid, UBXU1_t data[59], UBXU1_t optional0[48], UBXU1_t optional1[48], UBXU1_t optional2[48])
{
    int payloadSize = sizeof(struct UBXAID_AOP_OPT);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_AOP);
    msg->payload.AID_AOP_OPT.svid = svid;
    memcpy(msg->payload.AID_AOP_OPT.data, data, 59*sizeof(UBXU1_t));
    memcpy(msg->payload.AID_AOP_OPT.optional0, optional0, 48*sizeof(UBXU1_t));
    memcpy(msg->payload.AID_AOP_OPT.optional1, optional1, 48*sizeof(UBXU1_t));
    memcpy(msg->payload.AID_AOP_OPT.optional2, optional2, 48*sizeof(UBXU1_t));
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_DATA_POLL()
{
    int payloadSize = sizeof(struct UBXAID_DATA_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_DATA);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_EPH_POLL()
{
    int payloadSize = sizeof(struct UBXAID_EPH_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_EPH);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_EPH_POLL_OPT(UBXU1_t svid)
{
    int payloadSize = sizeof(struct UBXAID_EPH_POLL_OPT);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_EPH);
    msg->payload.AID_EPH_POLL_OPT.svid = svid;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_EPH(UBXU4_t svid, UBXU4_t how)
{
    int payloadSize = sizeof(struct UBXAID_EPH);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_EPH);
    msg->payload.AID_EPH.svid = svid;
    msg->payload.AID_EPH.how = how;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_EPH_OPT(UBXU4_t svid, UBXU4_t how, UBXU4_t sf1d[8], UBXU4_t sf2d[8], UBXU4_t sf3d[8])
{
    int payloadSize = sizeof(struct UBXAID_EPH_OPT);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_EPH);
    msg->payload.AID_EPH_OPT.svid = svid;
    msg->payload.AID_EPH_OPT.how = how;
    memcpy(msg->payload.AID_EPH_OPT.sf1d, sf1d, 8*sizeof(UBXU4_t));
    memcpy(msg->payload.AID_EPH_OPT.sf2d, sf2d, 8*sizeof(UBXU4_t));
    memcpy(msg->payload.AID_EPH_OPT.sf3d, sf3d, 8*sizeof(UBXU4_t));
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_HUI_POLL()
{
    int payloadSize = sizeof(struct UBXAID_HUI_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_HUI);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_HUI(UBXI4_t health,
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
                               UBXX2_t flags)
{
    int payloadSize = sizeof(struct UBXAID_HUI);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_HUI);
    msg->payload.AID_HUI.health = health;
    msg->payload.AID_HUI.utcA0 = utcA0;
    msg->payload.AID_HUI.utcA1 = utcA1;
    msg->payload.AID_HUI.utcTOW = utcTOW;
    msg->payload.AID_HUI.utcWNT = utcWNT;
    msg->payload.AID_HUI.utcLS = utcLS;
    msg->payload.AID_HUI.utcWNF = utcWNF;
    msg->payload.AID_HUI.utcDN = utcDN;
    msg->payload.AID_HUI.utcLSF = utcLSF;
    msg->payload.AID_HUI.utcSpare = utcSpare;
    msg->payload.AID_HUI.klobA0 = klobA0;
    msg->payload.AID_HUI.klobA1 = klobA1;
    msg->payload.AID_HUI.klobA2 = klobA2;
    msg->payload.AID_HUI.klobA3 = klobA3;
    msg->payload.AID_HUI.klobA0 = klobB0;
    msg->payload.AID_HUI.klobA1 = klobB1;
    msg->payload.AID_HUI.klobA2 = klobB2;
    msg->payload.AID_HUI.klobA3 = klobB3;
    msg->payload.AID_HUI.flags = flags;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_INI_POLL()
{
    int payloadSize = sizeof(struct UBXAID_INI_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_INI);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getAID_INI(UBXI1_t ecefXOrLat,
                               UBXI1_t ecefYOrLat,
                               UBXI1_t ecefZOrLat,
                               UBXU1_t posAcc,
                               UBXI1_t tmCfg,
                               UBXU2_t wnoOrDate,
                               UBXU4_t towOrDate,
                               UBXI4_t towNs,
                               UBXU4_t tAccMS,
                               UBXU4_t tAccNS,
                               UBXI4_t clkDOrFreq,
                               UBXU4_t clkDAccOrFreqAcc,
                               UBXX4_t flags)
{
    int payloadSize = sizeof(struct UBXAID_INI);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassAID, UBXMsgIdAID_INI);
    msg->payload.AID_INI.ecefXOrLat = ecefXOrLat;
    msg->payload.AID_INI.ecefYOrLat = ecefYOrLat;
    msg->payload.AID_INI.ecefZOrLat = ecefZOrLat;
    msg->payload.AID_INI.posAcc = posAcc;
    msg->payload.AID_INI.tmCfg = tmCfg;
    msg->payload.AID_INI.wnoOrDate = wnoOrDate;
    msg->payload.AID_INI.towOrDate = towOrDate;
    msg->payload.AID_INI.towNs = towNs;
    msg->payload.AID_INI.tAccMS = tAccMS;
    msg->payload.AID_INI.tAccNS = tAccNS;
    msg->payload.AID_INI.clkDOrFreq = clkDOrFreq;
    msg->payload.AID_INI.clkDAccOrFreqAcc = clkDAccOrFreqAcc;
    msg->payload.AID_INI.flags = flags;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_ANT(UBXX2_t flags, struct UBXANTPins pins)
{
    int payloadSize = sizeof(struct UBXCFG_ANT);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_ANT);
    msg->payload.CFG_ANT.flags = flags;
    msg->payload.CFG_ANT.pins = pins;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_ANT_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_ANT_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_ANT);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_CFG(UBXX4_t clearMask, UBXX4_t saveMask, UBXX4_t loadMask)
{
    int payloadSize = sizeof(struct UBXCFG_CFG);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_CFG);
    msg->payload.CFG_CFG.clearMask = clearMask;
    msg->payload.CFG_CFG.saveMask = saveMask;
    msg->payload.CFG_CFG.loadMask = loadMask;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_CFG_OPT(UBXX4_t clearMask, UBXX4_t saveMask, UBXX4_t loadMask, UBXX1_t deviceMask)
{
    int payloadSize = sizeof(struct UBXCFG_CFG_OPT);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_CFG);
    msg->payload.CFG_CFG_OPT.clearMask = clearMask;
    msg->payload.CFG_CFG_OPT.saveMask = saveMask;
    msg->payload.CFG_CFG_OPT.loadMask = loadMask;
    msg->payload.CFG_CFG_OPT.deviceMask = deviceMask;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_DAT_IN(UBXR8_t majA, UBXR8_t flat, UBXR4_t dX, UBXR4_t dY, UBXR4_t dZ, UBXR4_t rotX, UBXR4_t rotY, UBXR4_t rotZ, UBXR4_t scale)
{
    int payloadSize = sizeof(struct UBXCFG_DAT_IN);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_CFG);
    msg->payload.CFG_DAT_IN.majA = majA;
    msg->payload.CFG_DAT_IN.flat = flat;
    msg->payload.CFG_DAT_IN.dX = dX;
    msg->payload.CFG_DAT_IN.dY = dY;
    msg->payload.CFG_DAT_IN.dZ = dZ;
    msg->payload.CFG_DAT_IN.rotX = rotX;
    msg->payload.CFG_DAT_IN.rotY = rotY;
    msg->payload.CFG_DAT_IN.rotZ = rotZ;
    msg->payload.CFG_DAT_IN.scale = scale;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_DAT_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_DAT_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_DAT);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_GNSS_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_GNSS_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_GNSS);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_GNSS(UBXU1_t msgVer,
                                UBXU1_t numTrkChHw,
                                UBXU1_t numTrkChUse,
                                UBXU1_t numConfigBlocks,
                                struct UBXCFG_GNSS_PART* gnssPart,
                                int gnssPartCount)
{
    int payloadSize = sizeof(struct UBXCFG_GNSS) + gnssPartCount*sizeof(struct UBXCFG_GNSS_PART);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_GNSS);
    msg->payload.CFG_GNSS.msgVer = msgVer;
    msg->payload.CFG_GNSS.numTrkChHw = numTrkChHw;
    msg->payload.CFG_GNSS.numTrkChUse = numTrkChUse;
    msg->payload.CFG_GNSS.numConfigBlocks = numConfigBlocks;
    memcpy(&(msg->payload.CFG_GNSS) + sizeof(struct UBXCFG_GNSS), gnssPart, gnssPartCount*sizeof(struct UBXCFG_GNSS_PART));
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_INF_POLL(UBXU1_t protocolId)
{
    int payloadSize = sizeof(struct UBXCFG_INF_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_INF);
    msg->payload.CFG_INF_POLL.protocolId = protocolId;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_INF(struct UBXCFG_INF_PART* infPart, int infPartCount)
{
    int payloadSize = sizeof(struct UBXCFG_INF) + sizeof(struct UBXCFG_INF_PART)*infPartCount;
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_INF);
    memcpy(&(msg->payload.CFG_INF) + sizeof(struct UBXCFG_INF), infPart, infPartCount*sizeof(struct UBXCFG_INF_PART));
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_ITFM_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_ITFM_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_ITFM);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_ITFM(struct UBXITFMConfig config,
                                struct UBXITFMConfig2 config2)
{
    int payloadSize = sizeof(struct UBXCFG_ITFM);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_ITFM);
    msg->payload.CFG_ITFM.config = config;
    msg->payload.CFG_ITFM.config2 = config2;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_LOGFILTER_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_LOGFILTER_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_LOGFILTER);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_LOGFILTER(UBXU1_t version,
                                     UBXX1_t flags,
                                     UBXU2_t minIterval,
                                     UBXU2_t timeThreshold,
                                     UBXU2_t speedThreshold,
                                     UBXU4_t positionThreshold)
{
    int payloadSize = sizeof(struct UBXCFG_LOGFILTER);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_LOGFILTER);
    msg->payload.CFG_LOGFILTER.version = version;
    msg->payload.CFG_LOGFILTER.flags = flags;
    msg->payload.CFG_LOGFILTER.minIterval = minIterval;
    msg->payload.CFG_LOGFILTER.timeThreshold = timeThreshold;
    msg->payload.CFG_LOGFILTER.speedThreshold = speedThreshold;
    msg->payload.CFG_LOGFILTER.positionThreshold = positionThreshold;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_NAV5_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_NAV5_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_NAV5);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_NAV5(UBXX2_t mask,
                                enum UBXNAV5Model dynModel,
                                enum UBXNAV5FixMode fixMode,
                                UBXI4_t fixedAlt,
                                UBXU4_t fixedAltVar,
                                UBXI1_t minElev,
                                UBXU1_t drLimit,
                                UBXU2_t pDop,
                                UBXU2_t tDop,
                                UBXU2_t pAcc,
                                UBXU2_t tAcc,
                                UBXU1_t staticHoldThresh,
                                UBXU1_t dgpsTimeOut,
                                UBXU1_t cnoThreshNumSVs,
                                UBXU1_t cnoThresh)
{
    int payloadSize = sizeof(struct UBXCFG_NAV5);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_NAV5);
    msg->payload.CFG_NAV5.mask = mask;
    msg->payload.CFG_NAV5.dynModel = dynModel;
    msg->payload.CFG_NAV5.fixMode = fixMode;
    msg->payload.CFG_NAV5.fixedAlt = fixedAlt;
    msg->payload.CFG_NAV5.fixedAltVar = fixedAltVar;
    msg->payload.CFG_NAV5.minElev = minElev;
    msg->payload.CFG_NAV5.drLimit = drLimit;
    msg->payload.CFG_NAV5.pDop = pDop;
    msg->payload.CFG_NAV5.tDop = tDop;
    msg->payload.CFG_NAV5.pAcc = pAcc;
    msg->payload.CFG_NAV5.tAcc = tAcc;
    msg->payload.CFG_NAV5.staticHoldThresh = staticHoldThresh;
    msg->payload.CFG_NAV5.dgpsTimeOut = dgpsTimeOut;
    msg->payload.CFG_NAV5.cnoThreshNumSVs = cnoThreshNumSVs;
    msg->payload.CFG_NAV5.cnoThresh = cnoThresh;
    msg->payload.CFG_NAV5.reserved2 = 0;
    msg->payload.CFG_NAV5.reserved3 = 0;
    msg->payload.CFG_NAV5.reserved4 = 0;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_NAVX5_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_NAVX5_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_NAVX5);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_NAVX5(UBXU2_t version,
                                 UBXX2_t mask1,
                                 UBXU1_t minSVs,
                                 UBXU1_t maxSVs,
                                 UBXU1_t minCNO,
                                 UBXU1_t iniFix3D,
                                 UBXU2_t wknRollover,
                                 UBXU1_t usePPP,
                                 UBXU1_t aopCFG,
                                 UBXU1_t aopOrbMaxErr)
{
    int payloadSize = sizeof(struct UBXCFG_NAVX5);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_NAVX5);
    msg->payload.CFG_NAVX5.version = version;
    msg->payload.CFG_NAVX5.mask1 = mask1;
    msg->payload.CFG_NAVX5.reserved0 = 0;
    msg->payload.CFG_NAVX5.reserved1 = 0;
    msg->payload.CFG_NAVX5.reserved2 = 0;
    msg->payload.CFG_NAVX5.minSVs = minSVs;
    msg->payload.CFG_NAVX5.maxSVs = maxSVs;
    msg->payload.CFG_NAVX5.minCNO = minCNO;
    msg->payload.CFG_NAVX5.reserved5 = 0;
    msg->payload.CFG_NAVX5.iniFix3D = iniFix3D;
    msg->payload.CFG_NAVX5.reserved6 = 0;
    msg->payload.CFG_NAVX5.reserved7 = 0;
    msg->payload.CFG_NAVX5.reserved8 = 0;
    msg->payload.CFG_NAVX5.wknRollover = wknRollover;
    msg->payload.CFG_NAVX5.reserved9 = 0;
    msg->payload.CFG_NAVX5.reserved10 = 0;
    msg->payload.CFG_NAVX5.reserved11 = 0;
    msg->payload.CFG_NAVX5.usePPP = usePPP;
    msg->payload.CFG_NAVX5.aopCFG = aopCFG;
    msg->payload.CFG_NAVX5.reserved12 = 0;
    msg->payload.CFG_NAVX5.reserved13 = 0;
    msg->payload.CFG_NAVX5.aopOrbMaxErr = aopOrbMaxErr;
    msg->payload.CFG_NAVX5.reserved14 = 0;
    msg->payload.CFG_NAVX5.reserved15 = 0;
    msg->payload.CFG_NAVX5.reserved3 = 0;
    msg->payload.CFG_NAVX5.reserved4 = 0;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_NMEA_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_NMEA_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_NMEA);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_NMEA(UBXX1_t filter,
                                UBXU1_t nmeaVersion,
                                UBXU1_t numSV,
                                UBXX1_t flags,
                                UBXX4_t gnssToFilter,
                                enum UBXNMEASVNumbering svNumbering,
                                enum UBXNMEATalkerIds mainTalkerId,
                                enum UBXNMEAGSVTalkerIds gsvTalkerId)
{
    int payloadSize = sizeof(struct UBXCFG_NMEA);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_NMEA);
    msg->payload.CFG_NMEA.filter = filter;
    msg->payload.CFG_NMEA.nmeaVersion = nmeaVersion;
    msg->payload.CFG_NMEA.numSV = numSV;
    msg->payload.CFG_NMEA.flags = flags;
    msg->payload.CFG_NMEA.gnssToFilter = gnssToFilter;
    msg->payload.CFG_NMEA.svNumbering = svNumbering;
    msg->payload.CFG_NMEA.mainTalkerId = mainTalkerId;
    msg->payload.CFG_NMEA.gsvTalkerId = gsvTalkerId;
    msg->payload.CFG_NMEA.reserved = 0;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_NVS(UBXX4_t clearMask,
                               UBXX4_t saveMask,
                               UBXX4_t loadMask,
                               UBXX1_t deviceMask)
{
    int payloadSize = sizeof(struct UBXCFG_NVS);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_NVS);
    msg->payload.CFG_NVS.clearMask = clearMask;
    msg->payload.CFG_NVS.saveMask = saveMask;
    msg->payload.CFG_NVS.loadMask = loadMask;
    msg->payload.CFG_NVS.deviceMask = deviceMask;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_PM2_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_PM2_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_PM2);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_PM2(struct UBXCFG_PM2Flags flags, UBXU4_t updatePeriod, UBXU4_t searchPeriod, UBXU4_t gridOffset, UBXU2_t onTime, UBXU2_t minAcqTime)
{
    int payloadSize = sizeof(struct UBXCFG_PM2);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_PM2);
    msg->payload.CFG_PM2.flags = flags;
    msg->payload.CFG_PM2.updatePeriod = updatePeriod;
    msg->payload.CFG_PM2.searchPeriod = searchPeriod;
    msg->payload.CFG_PM2.gridOffset = gridOffset;
    msg->payload.CFG_PM2.onTime = onTime;
    msg->payload.CFG_PM2.minAcqTime = minAcqTime;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_PRT_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_PRT_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_PRT);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_PRT_POLL_OPT(UBXU1_t portId)
{
    int payloadSize = sizeof(struct UBXCFG_PRT_POLL_OPT);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_PRT);
    msg->payload.CFG_PRT_POLL_OPT.portId = portId;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_PRT_UART()
{

}

struct UBXMsgBuffer getCFG_PRT_USB()
{

}

struct UBXMsgBuffer getCFG_PRT_SPI()
{

}

struct UBXMsgBuffer getCFG_PRT_DDC()
{

}

struct UBXMsgBuffer getCFG_RATE_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_RATE_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_RATE);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_RATE(){}
struct UBXMsgBuffer getCFG_RINV(){}
struct UBXMsgBuffer getCFG_RINV_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_RINV_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_RINV);
    completeMsg(&buffer, payloadSize);
    return buffer;
}
struct UBXMsgBuffer getCFG_RXM(){}
struct UBXMsgBuffer getCFG_RXM_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_RXM_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_RXM);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_SBAS(){}
struct UBXMsgBuffer getCFG_SBAS_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_SBAS_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_SBAS);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_TP5_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_TP5_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_TP5);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_USB_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_USB_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_USB);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_USB(){}
struct UBXMsgBuffer getLOG_CREATE(){}
struct UBXMsgBuffer getLOG_ERASE(){}
struct UBXMsgBuffer getLOG_FINDTIME_IN(){}

struct UBXMsgBuffer getLOG_INFO_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_INF_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_INF);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getLOG_RETRIEVE(){}
struct UBXMsgBuffer getLOG_STRING(){}

struct UBXMsgBuffer getMON_VER_POLL()
{
    int payloadSize = sizeof(struct UBXMON_VER_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassMON, UBXMsgIdMON_VER);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getRXM_ALM_POLL()
{
    int payloadSize = sizeof(struct UBXRXM_ALM_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassRXM, UBXMsgIdRXM_ALM);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getRXM_ALM_POLL_OPT(UBXU1_t svid)
{
    int payloadSize = sizeof(struct UBXRXM_ALM_POLL_OPT);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassRXM, UBXMsgIdRXM_ALM);
    msg->payload.RXM_ALM_POLL_OPT.svid = svid;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getRXM_EPH_POLL()
{
    int payloadSize = sizeof(struct UBXRXM_EPH_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassRXM, UBXMsgIdRXM_EPH);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getRXM_EPH_POLL_OPT(UBXU1_t svid)
{
    int payloadSize = sizeof(struct UBXRXM_EPH_POLL_OPT);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassRXM, UBXMsgIdRXM_ALM);
    msg->payload.RXM_ALM_POLL_OPT.svid = svid;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getRXM_PMREQ(){}
struct UBXMsgBuffer getRXM_SVSI(){}
