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

struct UBXMsgBuffer getCFG_RST(int mode, UBXU2_t mask)
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

struct UBXMsgBuffer getCFG_RST_OPT(enum UBXBBRSpecialSets special, UBXU2_t mask)
{
    int payloadSize = sizeof(struct UBXCFG_RST);
    struct UBXMsgBuffer buffer  = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_RST);
    msg->payload.CFG_RST.resetMode = special;
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
    //TODO
    struct UBXMsgBuffer buffer = createBuffer(0);
    return buffer;
}

struct UBXMsgBuffer getCFG_PRT_USB()
{
    //TODO
    struct UBXMsgBuffer buffer = createBuffer(0);
    return buffer;
}

struct UBXMsgBuffer getCFG_PRT_SPI()
{
    //TODO
    struct UBXMsgBuffer buffer = createBuffer(0);
    return buffer;
}

struct UBXMsgBuffer getCFG_PRT_DDC()
{
    //TODO
    struct UBXMsgBuffer buffer = createBuffer(0);
    return buffer;
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

struct UBXMsgBuffer getCFG_RATE(UBXU2_t measRate, UBXU2_t navRate, UBXU2_t timeRef)
{
    int payloadSize = sizeof(struct UBXCFG_RATE);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_RATE);
    msg->payload.CFG_RATE.measRate = measRate;
    msg->payload.CFG_RATE.navRate = navRate;
    msg->payload.CFG_RATE.timeRef = timeRef;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_RINV(UBXX1_t flags, UBXU1_t* data, int dataSize)
{
    int payloadSize = sizeof(struct UBXCFG_RINV) + dataSize*sizeof(UBXU1_t);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_RINV);
    msg->payload.CFG_RINV.flags = flags;
    memcpy(&(msg->payload.CFG_RINV) + sizeof(struct UBXCFG_RINV), data, dataSize*sizeof(UBXU1_t));
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_RINV_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_RINV_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_RINV);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_RXM(UBXU1_t lpMode)
{
    int payloadSize = sizeof(struct UBXCFG_RXM);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_RXM);
    msg->payload.CFG_RXM.reserved1 = 8;
    msg->payload.CFG_RXM.lpMode = lpMode;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_RXM_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_RXM_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_RXM);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_SBAS(UBXX1_t mode, UBXX1_t usage, UBXU1_t maxSBAS, UBXX1_t scanmode2, UBXX4_t scanmode1)
{
    int payloadSize = sizeof(struct UBXCFG_SBAS);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_SBAS);
    msg->payload.CFG_SBAS.mode = mode;
    msg->payload.CFG_SBAS.usage = usage;
    msg->payload.CFG_SBAS.maxSBAS = maxSBAS;
    msg->payload.CFG_SBAS.scanmode2 = scanmode2;
    msg->payload.CFG_SBAS.scanmode1 = scanmode1;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

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

struct UBXMsgBuffer getCFG_USB(UBXU2_t vendorId,
                               UBXU2_t productId,
                               UBXU2_t powerConsumption,
                               UBXX2_t flags,
                               UBXCH_t* vendorString,
                               UBXCH_t* productString,
                               UBXCH_t* serialNumber)
{
    int payloadSize = sizeof(struct UBXCFG_USB);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_USB);
    msg->payload.CFG_USB.vendorId = vendorId;
    msg->payload.CFG_USB.productId = productId;
    msg->payload.CFG_USB.reserved1 = 0;
    msg->payload.CFG_USB.reserved2 = 1;
    msg->payload.CFG_USB.powerConsumption = powerConsumption;
    msg->payload.CFG_USB.flags = flags;
    int vendorStringSize = strlen(vendorString)>32?32:strlen(vendorString);
    memcpy(msg->payload.CFG_USB.vendorString, vendorString, vendorStringSize);

    int productStringSize = strlen(productString)>32?32:strlen(productString);
    memcpy(msg->payload.CFG_USB.productString, productString, productStringSize);

    int serialNumberSize = strlen(serialNumber)>32?32:strlen(serialNumber);
    memcpy(msg->payload.CFG_USB.serialNumber, serialNumber, serialNumberSize);

    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getLOG_CREATE(UBXU1_t version, UBXX1_t logCfg, UBXU1_t logSize, UBXU4_t userDefinedSize)
{
    int payloadSize = sizeof(struct UBXLOG_CREATE);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassLOG, UBXMsgIdLOG_CREATE);
    msg->payload.LOG_CREATE.version = version;
    msg->payload.LOG_CREATE.logCfg = logCfg;
    msg->payload.LOG_CREATE.reserved = 0;
    msg->payload.LOG_CREATE.logSize = logSize;
    msg->payload.LOG_CREATE.userDefinedSize = userDefinedSize;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getLOG_ERASE()
{
    int payloadSize = sizeof(struct UBXLOG_ERASE);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassLOG, UBXMsgIdLOG_ERASE);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getLOG_FINDTIME_IN(UBXU2_t year, UBXU1_t month, UBXU1_t day, UBXU1_t hour, UBXU1_t minute, UBXU1_t second)
{
    int payloadSize = sizeof(struct UBXLOG_FINDTIME_IN);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassLOG, UBXMsgIdLOG_FINDTIME);
    msg->payload.LOG_FINDTIME_IN.version = 0;
    msg->payload.LOG_FINDTIME_IN.type = 0;
    msg->payload.LOG_FINDTIME_IN.year = year;
    msg->payload.LOG_FINDTIME_IN.month = month;
    msg->payload.LOG_FINDTIME_IN.day = day;
    msg->payload.LOG_FINDTIME_IN.hour = hour;
    msg->payload.LOG_FINDTIME_IN.minute = minute;
    msg->payload.LOG_FINDTIME_IN.second = second;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getLOG_INFO_POLL()
{
    int payloadSize = sizeof(struct UBXLOG_INFO_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassLOG, UBXMsgIdLOG_INFO);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getLOG_RETRIEVE(UBXU4_t startNumber,
                                    UBXU4_t entryCount,
                                    UBXU1_t version)
{
    int payloadSize = sizeof(struct UBXLOG_RETRIEVE);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassLOG, UBXMsgIdLOG_RETRIEVE);
    msg->payload.LOG_RETRIEVE.startNumber = startNumber;
    msg->payload.LOG_RETRIEVE.entryCount = entryCount;
    msg->payload.LOG_RETRIEVE.version = version;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getLOG_STRING()
{
    int payloadSize = sizeof(struct UBXLOG_STRING);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassLOG, UBXMsgIdLOG_STRING);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

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

struct UBXMsgBuffer getRXM_PMREQ(UBXU4_t duration, UBXX4_t flags)
{
    int payloadSize = sizeof(struct UBXRXM_PMREQ);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassRXM, UBXMsgIdRXM_PMREQ);
    msg->payload.RXM_PMREQ.duration = duration;
    msg->payload.RXM_PMREQ.flags = flags;
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getRXM_SVSI(UBXU4_t iTOW,
                                UBXI2_t week,
                                UBXU1_t numVis,
                                UBXU1_t numSV,
                                struct UBXRXM_SVSI_PART* svsiPart,
                                int svsiPartCount)
{
    int payloadSize = sizeof(struct UBXRXM_SVSI) + svsiPartCount*sizeof(struct UBXRXM_SVSI_PART);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassRXM, UBXMsgIdRXM_SVSI);
    msg->payload.RXM_SVSI.iTOW = iTOW;
    msg->payload.RXM_SVSI.week = week;
    msg->payload.RXM_SVSI.numVis = numVis;
    msg->payload.RXM_SVSI.numSV = numSV;
    memcpy(&(msg->payload.RXM_SVSI) + sizeof(struct UBXRXM_SVSI), svsiPart, svsiPartCount*sizeof(struct UBXRXM_SVSI_PART));
    completeMsg(&buffer, payloadSize);
    return buffer;
}

/*!
 * \enum UBXMessageClass
 * \brief UBXMessageClass is a grouping of messages which are related to each other.
 * The following table lists all the current message classes.
 * \var UBXMsgClassNAV
 * Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
 * \var UBXMsgClassRXM
 * Receiver Manager Messages: Satellite Status, RTC Status
 * \var UBXMsgClassINF
 * Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
 * \var UBXMsgClassACK
 * Ack/Nack Messages: as replies to CFG Input Messages
 * \var UBXMsgClassCFG
 * Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
 * \var UBXMsgClassMON
 * Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
 * \var UBXMsgClassAID
 * AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
 * \var UBXMsgClassTIM
 * Timing Messages: Time Pulse Output, Timemark Results
 * \var UBXMsgClassLOG
 * Logging Messages: Log creation, deletion, info and retrieval
 * \var UBXMsgClassInvalid
 * Default invalid message class
 */

/*!
 * \enum UBXMessageId
 * \brief UBXMessageId is a type of messages
 * \var UBXMsgIdACK_NACK
 * Message Acknowledged
 * \var UBXMsgIdACK_ACK
 * Message Not-Acknowledged
 *
 * \var UBXMsgIdAID_ALM
 * GPS Aiding Almanac Data
 * \var UBXMsgIdAID_ALPSRV
 * AlmanacPlus data
 * \var UBXMsgIdAID_ALP
 * ALP file data transfer
 * \var UBXMsgIdAID_AOP
 * AssistNow Autonomous data
 * \var UBXMsgIdAID_DATA
 * GPS Initial Aiding Data
 * \var UBXMsgIdAID_EPH
 * GPS Aiding Ephemeris Data
 * \var UBXMsgIdAID_HUI
 * GPS Health, UTC and ionosphere parameters
 * \var UBXMsgIdAID_INI
 * Aiding position, time, frequency, clock drift
 * \var UBXMsgIdAID_REQ
 * Sends a poll (AID-DATA) for all GPS Aiding Data
 *
 * \var UBXMsgIdCFG_ANT
 * Antenna Control Settings
 * \var UBXMsgIdCFG_CFG
 * Clear, Save and Load configurations
 * \var UBXMsgIdCFG_DAT
 * Datum Setting
 * \var UBXMsgIdCFG_GNSS
 * GNSS system configuration
 * \var UBXMsgIdCFG_INF
 * Information message configuration
 * \var UBXMsgIdCFG_ITFM
 * Jamming/Interference Monitor configuration.
 * \var UBXMsgIdCFG_LOGFILTER
 * Data Logger Configuration
 * \var UBXMsgIdCFG_MSG
 * Message Configuration
 * \var UBXMsgIdCFG_NAV5
 * Navigation Engine Settings
 * \var UBXMsgIdCFG_NAVX5
 * Navigation Engine Expert Settings
 * \var UBXMsgIdCFG_NMEA
 * NMEA protocol configuration
 * \var UBXMsgIdCFG_NVS
 * Clear, Save and Load non-volatile storage data
 * \var UBXMsgIdCFG_PM2
 * Extended Power Management configuration
 * \var UBXMsgIdCFG_PRT
 * Ports Configuration
 * \var UBXMsgIdCFG_RATE
 * Navigation/Measurement Rate Settings
 * \var UBXMsgIdCFG_RINV
 * Contents of Remote Inventory
 * \var UBXMsgIdCFG_RST
 * Reset Receiver / Clear Backup Data Structures
 * \var UBXMsgIdCFG_RXM
 * RXM configuration
 * \var UBXMsgIdCFG_SBAS
 * SBAS Configuration
 * \var UBXMsgIdCFG_TP5
 * Time Pulse Parameters
 * \var UBXMsgIdCFG_USB
 * USB Configuration
 *
 * \var UBXMsgIdINF_DEBUG
 * ASCII String output, indicating debug output
 * \var UBXMsgIdINF_ERROR
 * ASCII String output, indicating an error
 * \var UBXMsgIdINF_NOTICE
 * ASCII String output, with informational contents
 * \var UBXMsgIdINF_TEST
 * ASCII String output, indicating test output
 * \var UBXMsgIdINF_WARNING
 * ASCII String output, indicating a warning
 *
 * \var UBXMsgIdLOG_CREATE
 * Create Log File
 * \var UBXMsgIdLOG_ERASE
 * Erase Logged Data
 * \var UBXMsgIdLOG_FINDTIME
 * Find the index of the first log entry <= given time
 * \var UBXMsgIdLOG_INFO
 * Log information
 * \var UBXMsgIdLOG_RETRIEVEPOS
 * Position fix log entry
 * \var UBXMsgIdLOG_RETRIEVESTRING
 * Byte string log entry
 * \var UBXMsgIdLOG_RETRIEVE
 * Request log data
 * \var UBXMsgIdLOG_STRING
 * Store arbitrary string in on-board Flash memory
 *
 * \var UBXMsgIdMON_HW2
 * Extended Hardware Status
 * \var UBXMsgIdMON_HW
 * Hardware Status
 * \var UBXMsgIdMON_IO
 * I/O Subsystem Status
 * \var UBXMsgIdMON_MSGPP
 * Message Parse and Process Status
 * \var UBXMsgIdMON_RXBUF
 * Receiver Buffer Status
 * \var UBXMsgIdMON_RXR
 * Receiver Status Information
 * \var UBXMsgIdMON_TXBUF
 * Transmitter Buffer Status
 * \var UBXMsgIdMON_VER
 * Receiver/Software Version
 *
 * \var UBXMsgIdNAV_AOPSTATUS
 * AssistNow Autonomous Status
 * \var UBXMsgIdNAV_CLOCK
 * Clock Solution
 * \var UBXMsgIdNAV_DGPS
 * DGPS Data Used for NAV
 * \var UBXMsgIdNAV_DOP
 * Dilution of precision
 * \var UBXMsgIdNAV_POSECEF
 * Position Solution in ECEF
 * \var UBXMsgIdNAV_POSLLH
 * Geodetic Position Solution
 * \var UBXMsgIdNAV_PVT
 * Navigation Position Velocity Time Solution
 * \var UBXMsgIdNAV_SBAS
 * SBAS Status Data
 * \var UBXMsgIdNAV_SOL
 * Navigation Solution Information
 * \var UBXMsgIdNAV_STATUS
 * Receiver Navigation Status
 * \var UBXMsgIdNAV_SVINFO
 * Space Vehicle Information
 * \var UBXMsgIdNAV_TIMEGPS
 * GPS Time Solution
 * \var UBXMsgIdNAV_TIMEUTC
 * UTC Time Solution
 * \var UBXMsgIdNAV_VELECEF
 * Velocity Solution in ECEF
 * \var UBXMsgIdNAV_VELNED
 * Velocity Solution in NED
 *
 * \var UBXMsgIdRXM_ALM
 * GPS Constellation Almanac Data
 * \var UBXMsgIdRXM_EPH
 * GPS Constellation Ephemeris Data
 * \var UBXMsgIdRXM_PMREQ
 * Requests a Power Management task
 * \var UBXMsgIdRXM_RAW
 * Raw Measurement Data
 * \var UBXMsgIdRXM_SFRB
 * Subframe Buffer
 * \var UBXMsgIdRXM_SVSI
 * SV Status Info
 *
 * \var UBXMsgIdTIM_TM2
 * Time mark data
 * \var UBXMsgIdTIM_TP
 * Time Pulse Timedata
 * \var UBXMsgIdTIM_VRFY
 * Sourced Time Verification
 *
 * \var MsgIdInvalid
 * Default invalid message identificator
 */

/*!
 * \enum UBXResetMode
 * \brief UBXResetMode describes possible reset modes
 * for #UBXCFG_RST message.
 * \var UBXHardwareReset
 * Hardware reset (Watchdog) immediately
 * \var UBXControlledReset
 * Controlled Software reset
 * \var UBXControlledResetGNSSOnly
 * Controlled Software reset (GNSS only)
 * \var UBXHardwareResetAfterShutdown
 * Hardware reset (Watchdog) after
 * \var UBXControlledGNSSStop
 * Controlled GNSS stop
 * \var UBXControlledGNSSStart
 * Controlled GNSS start
 */

/*!
 * \enum UBXBBRSpecialSets
 * \brief UBXBBRSpecialSets implements special sets for
 * #UBXCFG_RST message.
 * \var UBXBBRHotstart
 * Hotstart
 * \var UBXBBRWarmstart
 * Warmstart
 * \var UBXBBRColdstart
 * Coldstart
 */

/*!
 * \enum UBXBBRMask
 * \brief UBXBBRMask implements members for BBR(built-in battery-backed RAM) bitmask
 * for #UBXCFG_RST message.
 * \var UBXBBReph
 * Ephemeris
 * \var UBXBBRalm
 * Almanac
 * \var UBXBBRhealth
 * Health
 * \var UBXBBRklob
 * Klobuchar parameters
 * \var UBXBBRpos
 * Position
 * \var UBXBBRclkd
 * Clock Drift
 * \var UBXBBRosc
 * Oscillator Parameter
 * \var UBXBBRutc
 * UTC Correction + GPS Leap Seconds Parameters
 * \var UBXBBRrtc
 * RTC
 * \var UBXBBRsfdr
 * SFDR Parameters
 * \var UBXBBRvmon
 * SFDR Vehicle Monitoring Parameters
 * \var UBXBBRtct
 * TCT Parameters
 * \var UBXBBRaop
 * Autonomous Orbit Parameters
 */

/*!
 * \enum UBXHUIFlags
 * \brief UBXHUIFlags implements HUI flags bitmask
 * for #UBXAID_HUI message.
 * \var UBXHUIHealthValid
 * Healthmask field in this message is valid
 * \var UBXHUIUTCValid
 * UTC parameter fields in this message are valid
 * \var UBXHUIKlobValid
 * Klobuchar parameter fields in this message are valid
 */

/*!
 * \enum UBXINItmCfg
 * \brief UBXINItmCfg implements Time mark configuration bitmask
 * for #UBXAID_INI message.
 * \var UBXINIfEdge
 * Use falling edge (default rising)
 * \var UBXINItm1
 * Time mark on extint 1 (default extint 0)
 * \var UBXINIf1
 * Frequency on extint 1 (default extint 0)
 */

/*!
 * \enum UBXINIFlags
 * \brief UBXINIFlags implements INI flags bitmask
 * for #UBXAID_INI message.
 * \var UBXINIpos
 * Position is valid
 * \var UBXINItime
 * Time is valid
 * \var UBXINIclockD
 * Clock drift data contains valid clock drift, must not be set together with clockF
 * \var UBXINItp
 * Use time pulse
 * \var UBXINIclockF
 * Clock drift data contains valid frequency, must not be set together with clockD
 * \var UBXINIlla
 * Position is given in lat/long/alt (default is ECEF)
 * \var UBXINIaltInv
 * Altitude is not valid, in case lla was set
 * \var UBXINIprevTm
 * Use time mark received before AID-INI message (default uses mark received after message)
 * \var UBXINIutc
 * Time is given as UTC date/time (default is GPS wno/tow)
 */

/*!
 * \enum UBXANTFlags
 * \brief UBXANTFlags implements ANT flags bitmask
 * for #UBXCFG_ANT message.
 * \var UBXANTsvcs
 * Enable Antenna Supply Voltage Control Signal
 * \var UBXANTscd
 * Enable Short Circuit Detection
 * \var UBXANTocd
 * Enable Open Circuit Detection
 * \var UBXANTpdwnOnSCD
 * Power Down Antenna supply if Short Circuit is detected. (only in combination with Bit 1)
 * \var UBXANTrecovery
 * Enable automatic recovery from short state
 */

/*!
 * \enum UBXCFGMask
 * \brief UBXCFGMask implements CFG flags bitmask
 * for #UBXCFG_CFG message and #UBXCFG_NVS.
 * \var UBXCFGioPort
 * Port Settings\n
 * \note #UBXCFG_CFG only
 * \var UBXCFGmsgConf
 * Message Configuration\n
 * \note#UBXCFG_CFG only
 * \var UBXCFGinfMsg
 * INF Message Configuration\n
 * \note#UBXCFG_CFG only
 * \var UBXCFGnavConf
 * Navigation Configuration\n
 * \note#UBXCFG_CFG only
 * \var UBXCFGrxmConf
 * Receiver Manager Configuration\n
 * \note#UBXCFG_CFG only
 * \var UBXCFGrinvConf
 * Remote Inventory Configuration\n
 * \note#UBXCFG_CFG only
 * \var UBXCFGantConf
 * Antenna Configuration\n
 * \note#UBXCFG_CFG only
 * \var UBXCFGalm
 * GPS Almanac data\n
 * \note#UBXCFG_NVS only
 * \var UBXCFGaopConf
 * AOP data\n
 * \note#UBXCFG_NVS only
 */

/*!
 * \enum UBXCFGMask
 * \brief UBXCFGMask implements CFG flags bitmask
 * for #UBXCFG_CFG message and #UBXCFG_NVS.
 * Mask selects the devices for #UBXCFG_CFG
 * and #UBXCFG_NVS commands.
 * \var UBXCFGdevBBR
 * built-in battery-backed RAM
 * \var UBXCFGdevFlash
 * external flash memory
 * \var UBXCFGdevEEPROM
 * external EEPROM
 * \var UBXCFGdevSpiFlash
 * external SPI Flash
 */

/*!
 * \enum UBXCFGTimepulses
 * \brief UBXCFGTimepulses contains possible timepulse
 * selections for #UBXCFG_TP5 message.
 * \var UBXCFGTimepulse
 * TIMEPULSE selection
 * \var UBXCFGTimepulse2
 * TIMEPULSE2 selection
 */

/*!
 * \enum UBXCFGTimepulseFlags
 * \brief UBXCFGTimepulseFlags implements timepulse flags
 * for #UBXCFG_TP5 message.
 * \var UBXCFGTimepulseActive
 * if set enable time pulse; if pin assigned to another function, other function takes precedence
 * \var UBXCFGTimepulseLockGpsFreq
 * if set synchronize time pulse to GPS as soon as GPS time is valid, otherwise use local clock
 * \var UBXCFGTimepulseLockedOtherSet
 * if set use 'freqPeriodLock' and 'pulseLenRatioLock' as soon as GPS time is valid and 'freqPeriod' and
 * 'pulseLenRatio' if GPS time is invalid,
 * if flag is cleared 'freqPeriod' and 'pulseLenRatio' used regardless of GPS time
 * \var UBXCFGTimepulseIsFreq
 * if set 'freqPeriodLock' and 'freqPeriod' interpreted as frequency, otherwise interpreted as period
 * \var UBXCFGTimepulseIsLenght
 * if set 'pulseLenRatioLock' and 'pulseLenRatio' interpreted as pulse length, otherwise interpreted as duty cycle
 * \var UBXCFGTimepulseAlignToTow
 * align pulse to top of second (period time must be integer fraction of 1s)
 * \var UBXCFGTimepulsePolarity
 * pulse polarity:\n
 * 0 = falling edge at top of second\n
 * 1 = rising edge at top of second
 * \var UBXCFGTimepulseGridUTSGPS
 * timegrid to use:\n
 * 0 = UTC\n
 * 1 = GPS
 */

/*!
 * \enum UBXCFGProtocolIds
 * \brief UBXCFGProtocolIds
 * Protocol Identifiers, identifying the output
 * protocol for #UBXCFG_INF.
 * \var UBXProtocol
 * UBX Protocol
 * \var UBXNMEAProtocol
 * NMEA Protocol
 */

/*!
 * \enum UBXGNSSIds
 * \brief UBXGNSSIds
 * GNSS identificators used for #UBXCFG_GNSS in #UBXCFG_GNSS_PART
 * \var UBXGPS
 * GPS
 * \var UBXSBAS
 * SBAS
 * \var UBXQZSS
 * QZSS
 * \var UBXGLONASS
 * GLONASS
 */

/*!
 * \enum UBXCFGInfMsgMask
 * \brief UBXCFGInfMsgMask implements
 * bit mask, saying which information messages are enabled on each I/O port
 * \var UBXInfError
 * Error
 * \var UBXInfWarning
 * Warning
 * \var UBXInfNotice
 * Notice
 * \var UBXInfDebug
 * Debug
 * \var UBXInfTest
 * Test
 */

/*!
 * \enum UBXITFMAntSetting
 * \brief UBXITFMAntSetting implements possible antenna settings for
 * #UBXCFG_ITFM in UBXCFG_ITFM::config2 part.
 * \var UBXITFMAntUnknown
 * Unknown
 * \var UBXITFMAntPassive
 * Passive
 * \var UBXITFMAntActive
 * Active
 */

/*!
 * \enum UBXLOGFILTERFlags
 * \brief UBXLOGFILTERFlags implements log filter flags bitmask for
 * #UBXCFG_LOGFILTER
 * \var UBXLOGFILTERRecordEnabled
 * 1 = enable recording\n
 * 0 = disable recording
 * \var UBXLOGFILTERPsmOncePerWakupEnabled
 * 1 = enable recording only one single position per PSM on/off mode wake up period\n
 * 0 = disable once per wake up
 * \var UBXLOGFILTERApplyAllFilterSettings
 * 1 = apply all filter settings\n
 * 0 = only apply recordEnabled
 */

/*!
 * \enum UBXNAV5Mask
 * \brief UBXNAV5Mask implements flags bitmask
 * for #UBXCFG_NAV5 message
 * \var UBXNAV5Dyn
 * Apply dynamic model settings
 * \var UBXNAV5MinEl
 * Apply minimum elevation settings
 * \var UBXNAV5PosFixMode
 * Apply fix mode settings
 * \var UBXNAV5DrLim
 * Reserved
 * \var UBXNAV5PosMask
 * Apply position mask settings
 * \var UBXNAV5TimeMask
 * Apply time mask settings
 * \var UBXNAV5StaticHoldMask
 * Apply static hold settings
 * \var UBXNAV5DgpsMask
 * Apply DGPS settings.
 */

/*!
 * \enum UBXNAV5Model
 * \brief UBXNAV5Model
 * enum describes dynamic platform models
 * for #UBXCFG_NAV5 message
 * \var UBXNAV5ModelPortable
 * Portable
 * \var UBXNAV5ModelStationary
 * Stationary
 * \var UBXNAV5ModelPedestrian
 * Pedestrian
 * \var UBXNAV5ModelAutomotive
 * Automotive
 * \var UBXNAV5ModelSea
 * Sea
 * \var UBXNAV5ModelAirborne1g
 * Airborne1g
 * \var UBXNAV5ModelAirborne2g
 * Airborne2g
 * \var UBXNAV5ModelAirborne4g
 * Airborne4g
 */

/*!
 * \enum UBXNAV5FixMode
 * \brief UBXNAV5FixMode
 * enum describes position fixing mode
 * for #UBXCFG_NAV5 message
 * \var UBXNAV5Fix2DOnly
 * 2D Only
 * \var UBXNAV5Fix3DOnly
 * 3D Only
 * \var UBXNAV5FixAuto2D3D
 * Auto 2D/3D
 */

/*!
 * \enum UBXNAVX5Mask
 * \brief UBXNAVX5Mask
 * implements flags bitmask
 * for #UBXCFG_NAVX5 message
 * \var UBXNAVX5AopMinMax
 * Apply min/max SVs settings
 * \var UBXNAVX5AopMinCno
 * Apply minimum C/N0 setting
 * \var UBXNAVX5AopInitial3dfix
 * Apply initial 3D fix settings
 * \var UBXNAVX5AopWknRoll
 * Apply GPS weeknumber rollover settings
 * \var UBXNAVX5AopPPP
 * Apply PPP flag\n
 * \note Only supported on certain product variants
 * \var UBXNAVX5Aop
 * Apply useAOP flag and aopOrbMaxErr setting (AssistNow Autonomous)
 */

/*!
 * \enum UBXNMEAFilter
 * \brief UBXNMEAFilter
 * implements NMEA message filter bitmask
 * for #UBXCFG_NMEA
 * \var UBXNMEAPosFilter
 * Enable position output for failed or invalid fixes
 * \var UBXNMEAMskPosFilter
 * Enable position output for invalid fixes
 * \var UBXNMEATimeFilter
 * Enable time output for invalid times
 * \var UBXNMEADateFilter
 * Enable date output for invalid dates
 * \var UBXNMEAGPSOnlyFilter
 * Restrict output to GPS satellites only
 * \var UBXNMEATrackFilter
 * Enable COG output even if COG is frozen
 */

/*!
 * \enum UBXNMEAVersion
 * \brief UBXNMEAVersion
 * describes supported NMEA protocol version
 * \var UBXNMEAVersion23
 * NMEA version 2.3
 * \var UBXNMEAVersion21
 * NMEA version 2.1
 */

/*!
 * \enum UBXNMEAFlags
 * \brief UBXNMEAFlags
 * implements flags bitmask for #UBXCFG_NMEA
 * \var UBXNMEACompatFlag
 * enable compatibility mode.\n
 * This might be needed for certain applications when customer's NMEA parser expects a fixed number of digits in
 * position coordinates
 * \var UBXNMEAConsiderFlag
 * enable considering mode.
 */

/*!
 * \enum UBXNMEAGNSSToFilter
 * \brief UBXNMEAGNSSToFilter
 * implements bitmask to filters out satellites based
 * on their GNSS.
 * \var UBXNMEAGPSFilter
 * Disable reporting of GPS satellites
 * \var UBXNMEASBASFilter
 * Disable reporting of SBAS satellites
 * \var UBXNMEAQZSSFilter
 * Disable reporting of QZSS satellites
 * \var UBXNMEAGLONASSFilter
 * Disable reporting of GLONASS satellites
 */

/*!
 * \enum UBXNMEASVNumbering
 * \brief UBXNMEASVNumbering
 *
 * \var UBXNMEASVNumStrict
 * Satellites are not output
 * \var UBXNMEASVNumExtended
 * Use UBX proprietary numbering
 */

/*!
 * \enum UBXNMEATalkerIds
 * \brief UBXNMEATalkerIds
 *
 * \var UBXNMEATalkerNotOverriden
 * Main Talker ID is not overridden
 * \var UBXNMEATalkerGP
 * Set main Talker ID to 'GP'
 * \var UBXNMEATalkerGL
 * Set main Talker ID to 'GL'
 * \var UBXNMEATalkerGN
 * Set main Talker ID to 'GN'
 */

/*!
 * \enum UBXNMEAGSVTalkerIds
 * \brief UBXNMEAGSVTalkerIds
 *
 * \var UBXNMEAGSVTalkerGNSSSpecific
 * Use GNSS specific Talker ID (as defined by NMEA)
 * \var UBXNMEAGSVTalkerMain
 * Use the main Talker ID
 */

/*!
 * \enum UBXPM2LimitPeakCurrent
 * \brief UBXPM2LimitPeakCurrent
 *
 * \var UBXPM2LimitCurrentDisabled
 * disabled
 * \var UBXPM2LimitCurrentEnabled
 * enabled, peak current is limited
 */

/*!
 * \enum UBXPM2Mode
 * \brief UBXPM2Mode
 *
 * \var UBXPM2OnOffOperation
 * ON/OFF operation
 * \var UBXPM2CyclicTrackOperation
 * Cyclic tracking operation
 */


/*!
 * \enum UBXPRTModeCharLen
 * \brief UBXPRTModeCharLen used for #UBXCFG_PRT
 *
 * \var UBXPRTMode5BitCharLen
 * 5bit \n
 * \note Not supported
 * \var UBXPRTMode6BitCharLen
 * 6bit\n
 * \note Not supported
 * \var UBXPRTMode7BitCharLen
 * 7bit\n
 * \note Supported only with parity
 * \var UBXPRTMode8BitCharLen
 * 8bit
 */

/*!
 * \enum UBXPRTModeParity
 * \brief UBXPRTModeParity used for #UBXCFG_PRT
 *
 * \var UBXPRTModeEvenParity
 * Even Parity
 * \var UBXPRTModeOddParity
 * Odd Parity
 * \var UBXPRTModeNoParity
 * No Parity
 * \var UBXPRTModeReserved
 * Reserved\n
 * \note Exclude this member from target value
 */

/*!
 * \enum UBXPRTModeStopBits
 * \brief UBXPRTModeStopBits used for #UBXCFG_PRT
 *
 * \var UBXPRTMode1StopBit
 * 1 Stop Bit
 * \var UBXPRTMode1dot5StopBit
 * 1.5 Stop Bit
 * \var UBXPRTMode2StopBit
 * 2 Stop Bit
 * \var UBXPRTMode0dot5StopBit
 * 0.5 Stop Bit
 */

/*!
 * \enum UBXPRTInProtoMask
 * \brief UBXPRTInProtoMask used for #UBXCFG_PRT
 *
 * \var UBXPRTInProtoInUBX
 * UBX protocol
 * \var UBXPRTInProtoInNMEA
 * NMEA protocol
 * \var UBXPRTInProtoInRTCM
 * RTCM protocol
 */

/*!
 * \enum UBXPRTOutProtoMask
 * \brief UBXPRTOutProtoMask used for #UBXCFG_PRT
 *
 * \var UBXPRTOutProtoOutUBX
 * UBX protocol
 * \var UBXPRTOutProtoOutNMEA
 * NMEA protocol
 */

/*!
 * \enum UBXPRTFlags
 * \brief UBXPRTFlags used for #UBXCFG_PRT
 *
 * \var UBXPRTExtendedTxTimeout
 * Extended TX timeout: if set, the port will timeout if allocated TX memory >=4 kB and no activity for 1.5s
 */

/*!
 * \enum UBXPRTSPIMode
 * \brief UBXPRTSPIMode used for #UBXCFG_PRT
 *
 * \var UBXPRTSPIMode0
 * 0 SPI Mode 0: CPOL = 0, CPHA = 0
 * \var UBXPRTSPIMode1
 * SPI Mode 1: CPOL = 0, CPHA = 1
 * \var UBXPRTSPIMode2
 * SPI Mode 2: CPOL = 1, CPHA = 0
 * \var UBXPRTSPIMode3
 * SPI Mode 3: CPOL = 1, CPHA = 1
 */

/*!
 * \enum UBXRINVFlags
 * \brief UBXRINVFlags used for #UBXCFG_RINV
 *
 * \var UBXRINVDump
 * Dump data at startup. Does not work if flag binary is set.
 * \var UBXRINVBinary
 * Data is binary
 */

/*!
 * \enum UBXRXMLowPowerModes
 * \brief UBXRXMLowPowerModes used for #UBXCFG_RXM
 *
 * \var UBXRXMContinousMode
 * Continous Mode
 * \var UBXRXMPowerSaveMode
 * Power Save Mode
 */

/*!
 * \enum UBXSBASModes
 * \brief UBXSBASModes used for #UBXCFG_SBAS
 *
 * \var UBXSBASModeEnabled
 * SBAS Enabled (1) / Disabled (0)
 * \var UBXSBASModeTest
 * SBAS Testbed: Use data anyhow (1) / Ignore data when in Test Mode (SBAS Msg 0)
 */

/*!
 * \enum UBXSBASUsage
 * \brief UBXSBASUsage
 *
 * \var UBXSBASUsageRange
 * Use SBAS GEOs as a ranging source (for navigation)
 * \var UBXSBASUsageDiffCorr
 * Use SBAS Differential Corrections
 * \var UBXSBASUsageIntegrity
 * Use SBAS Integrity Information
 */

/*!
 * \enum UBXSBASScanModes2
 * \brief UBXSBASScanModes2
 *
 * \var UBXSBASScanModePRN152
 * \var UBXSBASScanModePRN153
 * \var UBXSBASScanModePRN154
 * \var UBXSBASScanModePRN155
 * \var UBXSBASScanModePRN156
 * \var UBXSBASScanModePRN157
 * \var UBXSBASScanModePRN158
 */

/*!
 * \enum UBXSBASScanModes1
 * \brief UBXSBASScanModes1
 *
 * \var UBXSBASScanModePRN120
 * \var UBXSBASScanModePRN121
 * \var UBXSBASScanModePRN122
 * \var UBXSBASScanModePRN123
 * \var UBXSBASScanModePRN124
 * \var UBXSBASScanModePRN125
 * \var UBXSBASScanModePRN126
 * \var UBXSBASScanModePRN127
 * \var UBXSBASScanModePRN128
 * \var UBXSBASScanModePRN129
 * \var UBXSBASScanModePRN130
 * \var UBXSBASScanModePRN131
 * \var UBXSBASScanModePRN132
 * \var UBXSBASScanModePRN133
 * \var UBXSBASScanModePRN134
 * \var UBXSBASScanModePRN135
 * \var UBXSBASScanModePRN136
 * \var UBXSBASScanModePRN137
 * \var UBXSBASScanModePRN138
 * \var UBXSBASScanModePRN139
 * \var UBXSBASScanModePRN140
 * \var UBXSBASScanModePRN141
 * \var UBXSBASScanModePRN142
 * \var UBXSBASScanModePRN143
 * \var UBXSBASScanModePRN144
 * \var UBXSBASScanModePRN145
 * \var UBXSBASScanModePRN146
 * \var UBXSBASScanModePRN147
 * \var UBXSBASScanModePRN148
 * \var UBXSBASScanModePRN149
 * \var UBXSBASScanModePRN150
 * \var UBXSBASScanModePRN151
 */

/*!
 * \enum UBXUSBFlags
 * \brief UBXUSBFlags
 *
 * \var USBFlagReEnum
 * force re-enumeration
 * \var USBFlagPowerMode
 * self-powered (1), bus-powered (0)
 */

/*!
 * \enum UBXLOGCfg
 * \brief UBXLOGCfg
 * \var UBXLOGCfgCircular
 * Log is circular (new entries overwrite old ones in a full log) if this bit set
 */

/*!
 * \enum UBXLOGSize
 * \brief UBXLOGSize
 *
 * \var UBXLOGMaximumSafeSize
 * Maximum safe size\n
 * \note Ensures that logging will not be interupted and enough space
 * will be left avaiable for all other uses of the filestore
 * \var UBXLOGMinimunSize
 * Minimun size
 * \var UBXLOGUserDefined
 * User defined\n
 * \note See UBXLOG_CREATE::userDefinedSize
 */

/*!
 * \enum UBXLOGStatus
 * \brief UBXLOGStatus
 *
 * \var UBXLOGStatusRecording
 * Log entry recording is currently turned on
 * \var UBXLOGStatusInactive
 * Logging system not active - no log present
 * \var UBXLOGStatusCircular
 * The current log is circular
 */

/*!
 * \enum UBXRETRIEVEPOSFixType
 * \brief UBXRETRIEVEPOSFixType
 *
 * \var UBXRETRIEVEPOS2DFix
 * 2D-Fix
 * \var UBXRETRIEVEPOS3DFix
 * 3D-Fix
 */

/*!
 * \enum UBXRXRFlags
 * \brief UBXRXRFlags
 *
 * \var UBXRXRAwake
 * not in Backup mode
 */

/*!
 * \enum UBXAOPStatus
 * \brief UBXAOPStatus
 *
 * \var UBXAOPStatusIdle
 * AOP idle
 * \var UBXAOPStatusRunning
 * AOP running
 */

/*!
 * \enum UBXAOPCfg
 * \brief UBXAOPCfg
 *
 * \var UBXAOPCfgUseAOP
 * AOP enabled flag
 */

/*!
 * \enum UBXGPSFix
 * \brief UBXGPSFix
 *
 * \var UBXGPSNoFix
 * No Fix
 * \var UBXGPSDeadReckoning
 * Dead Reckoning only
 * \var UBXGPS2DFix
 * 2D-Fix
 * \var UBXGPS3DFix
 * 3D-Fix
 * \var UBXGPSGNSSDeadReckoning
 * GNSS + dead reckoning combined
 * \var UBXGPSTimeOnlyFix
 * Time only fix
 */

/*!
 * \enum UBXPVTValid
 * \brief UBXPVTValid
 *
 * \var UBXPVTValidDate
 * Valid UTC Date
 * \var UBXPVTValidTime
 * Valid UTC Time of Day
 * \var UBXPVTFullyResolved
 * UTC Time of Day has been fully resolved (no seconds uncertainty)
 */

/*!
 * \enum UBXPVTPSMStates
 * \brief UBXPVTPSMStates
 *
 * \var UBXPVTPSMStateNA
 * n/a (i.e no PSM is active)
 * \var UBXPVTPSMStateEnabled
 * Enabled (an intermediate state before Acquisition state)
 * \var UBXPVTPSMStateAcquisition
 * Acquisition
 * \var UBXPVTPSMStateTracking
 * Tracking
 * \var UBXPVTPSMStatePowerOptim
 * Power optimized tracking
 * \var UBXPVTPSMStateInactive
 * Inactive
 */

/*!
 * \enum UBXSBASService
 * \brief UBXSBASService
 *
 * \var UBXSBASServiceRanging
 * Service ranging
 * \var UBXSBASServiceCorrections
 * Service corrections
 * \var UBXSBASServiceIntegrity
 * Service integrity
 * \var UBXSBASServiceTestmode
 * Service in testmode
 */

/*!
 * \enum UBXSBASSOLFlags
 * \brief UBXSBASSOLFlags
 *
{
 * \var UBXSBASSOLGPSfixOK
 * Fix within limits. Position and velocity valid and within DOP and ACC Masks.
 * \var UBXSBASSOLDiffSoln
 * DGPS used
 * \var UBXSBASSOLWKNSet
 * Valid GPS week number
 * \var UBXSBASSOLTOWSet
 * Valid GPS time of week (iTOW & fTOW)
 */

/*!
 * \enum UBXSVINFOChipGen
 * \brief UBXSVINFOChipGen
 *
 * \var UBXSVINFOAntarisChip
 * Antaris, Antaris 4
 * \var UBXSVINFOUBlox5Chip
 * u-blox 5
 * \var UBXSVINFOUBlox6Chip
 * u-blox 6
*/

/*!
 * \enum UBXSVINFOFlags
 * \brief UBXSVINFOFlags
 *
{
 * \var UBXSVINFOFlagsSVUsed
 * SV is used for navigation
 * \var UBXSVINFOFlagsDiffCorr
 * Differential correction data is available for this SV
 * \var UBXSVINFOFlagsOrbitAvail
 * Orbit information is available for this SV (Ephemeris or Almanac)
 * \var UBXSVINFOFlagsOrbitEph
 * Orbit information is Ephemeris
 * \var UBXSVINFOFlagsUnhealthy
 * SV is unhealthy / shall not be used
 * \var UBXSVINFOFlagsOrbitAlm
 * Orbit information is Almanac Plus
 * \var UBXSVINFOFlagsOrbitAop
 * Orbit information is AssistNow Autonomous
 * \var UBXSVINFOFlagsSmoothed
 * Carrier smoothed pseudorange used
 */

/*!
 * \enum UBXSVINFOQualityId
 * \brief UBXSVINFOQualityId
 *
 * \var UBXSVINFOQualityChannelIdle
 * This channel is idle
 * \var UBXSVINFOQualityChannelSearching
 * Channel is searching
 * \var UBXSVINFOQualitySignalAquired
 * Signal aquired
 * \var UBXSVINFOQualitySignalDetected
 * Signal detected but unusable
 * \var UBXSVINFOQualityCodeLockOnSignal
 * Code Lock on Signal
 * \var UBXSVINFOQualityCodeCarrierLocked
 * Code and Carrier locked
 */

/*!
 * \enum UBXTIMEGPSValidityFlags
 * \brief UBXTIMEGPSValidityFlags
 *
 * \var UBXTIMEGPSTowValid
 * Valid GPS time of week (iTOW & fTOW)
 * \var UBXTIMEGPSWeekValid
 * Valid GPS week number
 * \var UBXTIMEGPSLeapSValid
 * Valid GPS leap seconds
 */

/*!
 * \enum UBXTIMEUTCValidityFlags
 * \brief UBXTIMEUTCValidityFlags
 *
 * \var UBXTIMEUTCValidTOW
 * Valid Time of Week
 * \var UBXTIMEUTCValidWKN
 * Valid Week Number
 * \var UBXTIMEUTCValidUTC
 * Valid UTC Time
 */

/*!
 * \enum UBXPMREQFlags
 * \brief UBXPMREQFlags
 *
 * \var UBXPMREQBackup
 * The receiver goes into backup mode for a time period defined by duration
 */

/*!
 * \enum UBXTM2FlagsMode
 * \brief UBXTM2FlagsMode
 *
 * \var UBXTM2FlagsModeSingle
 * Single mode
 * \var UBXTM2FlagsModeRunning
 * Running mode
 */

/*!
 * \enum UBXTM2FlagsRun
 * \brief UBXTM2FlagsRun
 *
 * \var UBXTM2FlagsRunArmed
 * Armed
 * \var UBXTM2FlagsRunStopped
 * Stopped
 */

/*!
 * \enum UBXTM2FlagsTimeBase
 * \brief UBXTM2FlagsTimeBase
 *
 * \var UBXTM2FlagsTimeBaseReceiverTime
 * Time base is Receiver Time
 * \var UBXTM2FlagsTimeBaseGPS
 * Time base is GPS
 * \var UBXTM2FlagsTimeBaseUTC
 * Time base is UTC
 */

/*!
 * \enum UBXTM2FlagsUTC
 * \brief UBXTM2FlagsUTC
 *
 * \var UBXTM2FlagsUTCNotAvailable
 * UTC not available
 * \var UBXTM2FlagsUTCAvailable
 * UTC available
 */

/*!
 * \enum UBXTM2FlagsTime
 * \brief UBXTM2FlagsTime
 *
 * \var UBXTM2FlagsTimeInvalid
 * Time is not valid
 * \var UBXTM2FlagsTimeValid
 * Time is valid (Valid GPS fix)
 */

/*!
 * \enum UBXTPFlags
 * \brief UBXTPFlags
 *
 * \var UBXTPTimeBaseUTC
 *  If is set time base is UTC. If not set time base is GPS
 * \var UBXTPUTCAvailable
 * UTC available
 */

/*!
 * \enum UBXVRFYFlagsSource
 * \brief UBXVRFYFlagsSource
 *
 * \var UBXVRFYNoTimeAidingDone
 * No time aiding done
 * \var UBXVRFYSourceRTC
 * Source was RTC
 * \var UBXVRFYSourceAID_INI
 * Source was AID-INI
 */

/*! \struct UBXHdr
 *  \brief This structure is UBX message header
 *  \var UBXHdr::msgClass
 *       Message class. A class is a grouping of messages which are related to each other.
 *       See #UBXMessageClass
 *  \var UBXHdr::msgId
 *       Message identificator. See #UBXMessageId
 *  \var UBXHdr::length
 *       Length is defined as being the length of the payload, only. It does not include
 *       Sync Chars, Length Field, Class, ID or CRC fields. The number format of the
 *       length field is an unsigned 16-Bit integer in Little Endian Format.
 */

/*! \struct UBXMsg
 *  \brief This structure is base for every message in
 *       UBX protocol
 *  \var UBXMsg::preamble
 *       preable of UBX message is allways 0xB562
 *  \var UBXMsg::hdr
 *       UBX message header of #UBXHdr type.
 *  \var UBXMsg::payload
 *       Union of #UBXMsgs type that contains all possible payloads
 *       for current protocol version
 */

/*! \struct UBXMsgBuffer
 *  \brief This structure is used by every message getters
 *         to wrap returned message data.
 *  \var UBXMsgBuffer::size
 *       size of buffer
 *  \var UBXMsgBuffer::data
 *       pointer to data heap
 */

/*! \union UBXMsgs
 *  \brief This union contains all possible payloads
 *         in current protocol version
 */

/*! \struct UBXAlpFileInfo
 *  \brief This structure contains data of alpfile
 *       It's simple wrapper on byte array with file id
 *       specification
 *  \var UBXAlpFileInfo::fileId
 *       File id used to indetificate part appurtenance
 *  \var UBXMsg::alpData
 *       Part of file data
 *  \var UBXMsg::dataSize
 *       Size of part in bytes
 */

/*!
 * \struct UBXAID_ALPSRV
 * This message is sent by the ALP client to the ALP server in order to request data. The given
 * identifier must be prepended to the requested data when submitting the data.
 * \brief ALP client requests AlmanacPlus data from server
 * \var UBXAID_ALPSRV::idSize
 * Identifier size. This data, beginning at message start, must prepend the returned data.
 * \var UBXAID_ALPSRV::type
 * Requested data type. Must be different from 0xff, otherwise this is not a data request.
 * \var UBXAID_ALPSRV::offset
 * Requested data offset [16bit words]
 * \var UBXAID_ALPSRV::size
 * Requested data size [16bit words]
 * \var UBXAID_ALPSRV::fileId
 * Unused when requesting data, filled in when sending back the data
 * \var UBXAID_ALPSRV::dataSize
 * Actual data size. Unused when requesting data, filled in when sending back the data.
 * \var UBXAID_ALPSRV::id1
 * Identifier data
 * \var UBXAID_ALPSRV::id2
 * Identifier data
 * \var UBXAID_ALPSRV::id3
 * Identifier data
*/

/*!
 * \struct UBXACK_ACK
 * Output upon processing of an input message
 * \brief Message acknowledged
 * \var UBXACK_ACK::msgClass
 * Class ID of the Acknowledged Message
 * \note See #UBXMessageClass to fill this field
 * \var UBXACK_ACK::msgId
 * Message ID of the Acknowledged Message
 * \note See #UBXMessageId to fill this field
*/

/*!
 * \struct UBXACK_NACK
 * Output upon processing of an input message
 * \brief Message Not-Acknowledged
 * \var UBXACK_NACK::msgClass
 * Class ID of the Acknowledged Message
 * \note See #UBXMessageClass to fill this field
 * \var UBXACK_NACK::msgId
 * Message ID of the Acknowledged Message
 * \note See #UBXMessageId to fill this field
*/

/*!
 * \struct UBXAID_ALM_POLL
 * Poll GPS Aiding Data (Almanac) for all 32 SVs by sending this message to the receiver
 * without any payload. The receiver will return 32 messages of type AID-ALM as defined
 * below.
 * \note No payload
 * \brief Poll GPS Aiding Almanac Data
*/

/*!
 * \struct UBXAID_ALM_POLL_OPT
 * Poll GPS Aiding Data (Almanac) for an SV by sending this message to the receiver. The
 * receiver will return one message of type AID-ALM as defined below.
 * \brief Poll GPS Aiding Almanac Data for a SV
 *
 * \var UBXAID_ALM_POLL_OPT::svid
 * SV ID for which the receiver shall return its
 * Almanac Data (Valid Range: 1 .. 32 or 51, 56,
 * 63).
*/

/*!
 * \struct UBXAID_ALM
 * - If the WEEK Value is 0, DWRD0 to DWRD7 are not sent as the Almanac is not available
 * for the given SV. This may happen even if NAV-SVINFO and RXM-SVSI are indicating
 * almanac availability as the internal data may not represent the content of an original
 * broadcast almanac (or only parts thereof).
 * - DWORD0 to DWORD7 contain the 8 words following the Hand-Over Word ( HOW )
 * from the GPS navigation message, either pages 1 to 24 of sub-frame 5 or pages 2 to 10
 * of subframe 4. See IS-GPS-200 for a full description of the contents of the Almanac
 * pages.
 * - In DWORD0 to DWORD7, the parity bits have been removed, and the 24 bits of data are
 * located in Bits 0 to 23. Bits 24 to 31 shall be ignored.
 * - Example: Parameter e (Eccentricity) from Almanac Subframe 4/5, Word 3, Bits 69-84
 * within the subframe can be found in DWRD0, Bits 15-0 whereas Bit 0 is the LSB.
 * \brief GPS Aiding Almanac Input/Output Message
 *
 * \var UBXAID_ALM::svid
 *  SV ID for which this
 * Almanac Data is (Valid Range: 1 .. 32 or 51, 56,
 * 63).
 * \var UBXAID_ALM::week
 * Issue Date of Almanac (GPS week number)
*/

/*!
 * \struct UBXAID_ALM_OPT
 * - If the WEEK Value is 0, DWRD0 to DWRD7 are not sent as the Almanac is not available
 * for the given SV. This may happen even if NAV-SVINFO and RXM-SVSI are indicating
 * almanac availability as the internal data may not represent the content of an original
 * broadcast almanac (or only parts thereof).
 * - DWORD0 to DWORD7 contain the 8 words following the Hand-Over Word ( HOW )
 * from the GPS navigation message, either pages 1 to 24 of sub-frame 5 or pages 2 to 10
 * of subframe 4. See IS-GPS-200 for a full description of the contents of the Almanac
 * pages.
 * - In DWORD0 to DWORD7, the parity bits have been removed, and the 24 bits of data are
 * located in Bits 0 to 23. Bits 24 to 31 shall be ignored.
 * - Example: Parameter e (Eccentricity) from Almanac Subframe 4/5, Word 3, Bits 69-84
 * within the subframe can be found in DWRD0, Bits 15-0 whereas Bit 0 is the LSB.
 * \brief GPS Aiding Almanac Input/Output Message
 *
 * \var UBXAID_ALM_OPT::svid
 *  SV ID for which this
 * Almanac Data is (Valid Range: 1 .. 32 or 51, 56,
 * 63).
 * \var UBXAID_ALM_OPT::week
 * Issue Date of Almanac (GPS week number)
 * \var UBXAID_ALM_OPT::dwrd[8]
 * Almanac Words
*/

/*!
 * \struct UBXAID_ALP
 * This message is used to transfer a chunk of data from the AlmanacPlus file to the receiver.
 * Upon reception of this message, the receiver will write the payload data to its internal
 * non-volatile memory, eventually also erasing that part of the memory first. Make sure that
 * the payload size is even sized (i.e. always a multiple of 2). Do not use payloads larger than
 * ~ 700 bytes, as this would exceed the receiver's internal buffering capabilities. The receiver
 * will (not-) acknowledge this message using the message alternatives given below. The host
 * shall wait for an acknowledge message before sending the next chunk.
 * \note This structure contains variable payload:\n
 * UBXU2_t alpData ALP file data
 * \brief  ALP file data transfer to the receiver
*/

/*!
 * \struct UBXAID_ALP_END
 * This message is used to indicate that all chunks have been transferred, and normal receiver
 * operation can resume. Upon reception of this message, the receiver will verify all chunks
 * received so far, and enable AssistNow Offline and GPS receiver operation if successful. This
 * message could also be sent to cancel an incomplete download.
 * \brief Mark end of data transfer
 * \var UBXAID_ALP_END::dummy
 * - Value is ignored if it's end of data transfer
 * - If value is set to 0x01 message acknowledges a data transfer
 * - If value is set to 0x00 message indicates problems with a data transfer
*/

/*!
 * \struct UBXAID_ALP_POLL
 * \brief   Poll the AlmanacPlus status
 *
 * \var UBXAID_ALP_POLL::predTow
 * Prediction start time of week
 * \var UBXAID_ALP_POLL::predDur
 * Prediction duration from start of first data set to
 * end of last data set
 * \var UBXAID_ALP_POLL::age
 * Current age of ALP data
 * \var UBXAID_ALP_POLL::predWno
 * Prediction start week number
 * \var UBXAID_ALP_POLL::almWno
 * Truncated week number of reference almanac
 * \var UBXAID_ALP_POLL::reserved1
 * Reserved
 * \var UBXAID_ALP_POLL::svs
 * Number of satellite data sets contained in the
 * ALP UBXAID_ALP_POLL::data
 * \var UBXAID_ALP_POLL::reserved2
 * Reserved
 * \var UBXAID_ALP_POLL::reserved3
 * Reserved
*/

/*!
 * \struct UBXAID_AOP_POLL
 * Poll AssistNow Autonomous aiding data for all satellits by sending this empty message. The
 * receiver will return an AID-AOP message (see definition below) for each satellite for which
 * data is available. For satellites for which no data is available it will return a corresponding
 * AID-AOP poll request message.
 * \note No payload
 * \brief Poll AssistNow Autonomous data
*/

/*!
 * \struct UBXAID_AOP_POLL_OPT
 * Poll the AssistNow Autonomous data for the specified satellite. The receiver will return a
 * AID-AOP message (see definition below) if data is available for the requested satellite. If no
 * data is available it will return corresponding AID-AOP poll request message (i.e. this
 * message).
 * \brief Poll AssistNow Autonomous data for one satellite
 * \var UBXAID_AOP_POLL_OPT::svid
 * GPS SV id for which the data is requested (valid range: 1..32).
*/

/*!
 * \struct UBXAID_AOP
 * If enabled, this message is output at irregular intervals. It is output whenever AssistNow
 * Autonomous has produced new data for a satellite. Depending on the availability of the
 * optional data the receiver will output either version of the message. If this message is
 * polled using one of the two poll requests described above the receiver will send this
 * message if AOP data is available or the corresponding poll request message if no AOP data
 * is available for each satellite (i.e. svid 1..32). At the user's choice the optional data may be
 * chopped from the payload of a previously polled message when sending the message back
 * to the receiver. Sending a valid AID-AOP message to the receiver will automatically enable
 * the AssistNow Autonomous feature on the receiver. See the section AssistNow
 * Autonomous in the receiver description for details on this feature.
 * \brief AssistNow Autonomous data
 * \var UBXAID_AOP::svid
 * GPS SV id
 * \var UBXAID_AOP::data[59]
 * AssistNow Autonomous data
*/

/*!
 * \struct UBXAID_AOP_OPT
 * If enabled, this message is output at irregular intervals. It is output whenever AssistNow
 * Autonomous has produced new data for a satellite. Depending on the availability of the
 * optional data the receiver will output either version of the message. If this message is
 * polled using one of the two poll requests described above the receiver will send this
 * message if AOP data is available or the corresponding poll request message if no AOP data
 * is available for each satellite (i.e. svid 1..32). At the user's choice the optional data may be
 * chopped from the payload of a previously polled message when sending the message back
 * to the receiver. Sending a valid AID-AOP message to the receiver will automatically enable
 * the AssistNow Autonomous feature on the receiver. See the section AssistNow
 * Autonomous in the receiver description for details on this feature.
 * \brief AssistNow Autonomous data
 * \var UBXAID_AOP_OPT::svid
 * GPS SV id
 * \var UBXAID_AOP_OPT::data[59]
 * AssistNow Autonomous data
 * \var UBXAID_AOP_OPT::optional0[48]
 * Optional data chunk 1/3
 * \var UBXAID_AOP_OPT::optional1[48]
 * Optional data chunk 2/3
 * \var UBXAID_AOP_OPT::optional2[48]
 * Optional data chunk 3/3
*/

/*!
 * \struct UBXAID_DATA_POLL
 * If this poll is received, the messages AID-INI, AID-HUI, AID-EPH and AID-ALM are sent.
 * \note No payload
 * \brief Polls all GPS Initial Aiding Data
*/

/*!
 * \struct UBXAID_EPH_POLL
 * \note No payload
 * \brief The UBXAID_EPH_POLL structure is
 *
*/

/*!
 * \struct UBXAID_EPH_POLL_OPT
 * \brief The UBXAID_EPH_POLL_OPT structure is
 *
 * \var svid
 *
*/

/*!
 * \struct UBXAID_EPH
 * \brief The UBXAID_EPH structure is
 *
 * \var svid
 *
 * \var how
 *
*/

/*!
 * \struct UBXAID_EPH_OPT
 * \brief The UBXAID_EPH_OPT structure is
 *
 * \var svid
 *
 * \var how
 *
 * \var sf1d[8]
 *
 * \var sf2d[8]
 *
 * \var sf3d[8]
 *
*/

/*!
 * \struct UBXAID_HUI_POLL
 * \note No payload
 * \brief The UBXAID_HUI_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXAID_HUI
 * \brief The UBXAID_HUI structure is
 *
 * \var health
 *
 * \var utcA0
 *
 * \var utcA1
 *
 * \var utcTOW
 *
 * \var utcWNT
 *
 * \var utcLS
 *
 * \var utcWNF
 *
 * \var utcDN
 *
 * \var utcLSF
 *
 * \var utcSpare
 *
 * \var klobA0
 *
 * \var klobA1
 *
 * \var klobA2
 *
 * \var klobA3
 *
 * \var klobB0
 *
 * \var klobB1
 *
 * \var klobB2
 *
 * \var klobB3
 *
 * \var flags
 *
*/

/*!
 * \struct UBXAID_INI_POLL
 * \brief The UBXAID_INI_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXAID_INI
 * \brief The UBXAID_INI structure is
 *
 * \var ecefXOrLat
 *
 * \var ecefYOrLat
 *
 * \var ecefZOrLat
 *
 * \var posAcc
 *
 * \var tmCfg
 *
 * \var wnoOrDate
 *
 * \var towOrDate
 *
 * \var towNs
 *
 * \var tAccMS
 *
 * \var tAccNS
 *
 * \var clkDOrFreq
 *
 * \var clkDAccOrFreqAcc
 *
 * \var flags
 *
*/

/*!
 * \struct UBXAID_REQ
 * \brief The UBXAID_REQ structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_ANT_POLL
 * \brief The UBXCFG_ANT_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXANTPins
{
 * \var UBXANTpinSwitch:5
 *
 * \var UBXANTpinSCD:5
 *
 * \var UBXANTpinOCD:5
 *
 * \var UBXANTreconfig:1
 *
*/

/*!
 * \struct UBXCFG_ANT
 * \brief The UBXCFG_ANT structure is
 *
 * \var flags
 *
 * \note See #UBXANTFlags to fill this field
    struct UBXANTPins pins;
*/

/*!
 * \struct UBXCFG_CFG
 * \brief The UBXCFG_CFG structure is
 *
 * \var clearMask
 *
 * \note See #UBXCFGMask to fill this field
 * \var saveMask
 *
 * \note See #UBXCFGMask to fill this field
 * \var loadMask
 *
 * \note See #UBXCFGMask to fill this field
*/

/*!
 * \struct UBXCFG_CFG_OPT
 * \brief The UBXCFG_CFG_OPT structure is
 *
 * \var clearMask
 *
 * \note See #UBXCFGMask to fill this field
 * \var saveMask
 *
 * \note See #UBXCFGMask to fill this field
 * \var loadMask
 *
 * \note See #UBXCFGMask to fill this field
 * \var deviceMask
 *
 * \note See #UBXCFGDeviceMask to fill this field
*/

/*!
 * \struct UBXCFG_DAT_POLL
 * \brief The UBXCFG_DAT_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_DAT_IN
 * \brief The UBXCFG_DAT_IN structure is
 *
 * \var majA
 *
 * \var flat
 *
 * \var dX
 *
 * \var dY
 *
 * \var dZ
 *
 * \var rotX
 *
 * \var rotY
 *
 * \var rotZ
 *
 * \var scale
 *
*/

/*!
 * \struct UBXCFG_DAT_OUT
 * \brief The UBXCFG_DAT_OUT structure is
 *
 * \var datumNum
 *
 * \var datumName[6]
 *
 * \var majA
 *
 * \var flat
 *
 * \var dX
 *
 * \var dY
 *
 * \var dZ
 *
 * \var rotX
 *
 * \var rotY
 *
 * \var rotZ
 *
 * \var scale
 *
*/

/*!
 * \struct UBXCFG_GNSS_POLL
 * \brief The UBXCFG_GNSS_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_GNSS
 * \brief The UBXCFG_GNSS structure is
 *
 * \var msgVer
 *
 * \var numTrkChHw
 *
 * \var numTrkChUse
 *
 * \var numConfigBlocks
 *
    //Variable addition here
    //See structure below
*/

/*!
     * \struct  UBXCFG_GNSS_PART
{
 * \var gnssId
 *
 * \note See #UBXGNSSIds to fill this field
 * \var resTrkCh
 *
 * \var maxTrkCh
 *
 * \var reserved1
 *
 * \var flags
 *  //0 - disabled, 1 - enabled
*/

/*!
 * \struct UBXCFG_INF_POLL
 * \brief The UBXCFG_INF_POLL structure is
 *
 * \var protocolId
 *
*/

/*!
 * \struct UBXCFG_INF
 * \brief The UBXCFG_INF structure is
 *
    //Variable payload
    //See structure UBXCFG_INF_PART below
*/

/*!
 * \struct UBXCFG_INF_PART
 * \brief The UBXCFG_INF_PART structure is
 *
 * \var protocolId
 *
 * \var reserved0
 *
 * \var reserved1
 *
 * \var infMsgMask[6]
 *
 * \note See #UBXCFGInfMsgMask to fill this field
*/

/*!
 * \struct UBXCFG_ITFM_POLL
 * \brief The UBXCFG_ITFM_POLL structure is
 *
    //No payload
*/

/*!
             * \struct  UBXITFMConfig
{
 * \var bbThreshold:4
 *
 * \var cwThreshold:5
 *
 * \var reserved1:22
 *  //Should be 0x16B156
 * \var enbled:1
 *
*/

/*!
             * \struct  UBXITFMConfig2
{
 * \var reserved2:12
 *  //Should be 0x31E
 * \var antSetting:2
 *
 * \note See #UBXITFMAntSetting to fill this field
 * \var reserved3:18
 *  //Should be 0x00
*/

/*!
 * \struct UBXCFG_ITFM
 * \brief The UBXCFG_ITFM structure is
 *
        struct UBXITFMConfig config;
        struct UBXITFMConfig2 config2;
    */

/*!
 * \struct UBXCFG_LOGFILTER_POLL
 * \brief The UBXCFG_LOGFILTER_POLL structure is
 *
    //No payload data
*/

/*!
 * \struct UBXCFG_LOGFILTER
 * \brief The UBXCFG_LOGFILTER structure is
 *
 * \var version
 *
 * \var flags
 *
 * \note See #UBXLOGFILTERFlags to fill this field
 * \var minIterval
 *
 * \var timeThreshold
 *
 * \var speedThreshold
 *
 * \var positionThreshold
 *
*/

/*!
 * \struct UBXCFG_MSG_POLL
 * \brief The UBXCFG_MSG_POLL structure is
 *
 * \var msgClass
 *
 * \var msgId
 *
*/

/*!
 * \struct UBXCFG_MSG_RATES
 * \brief The UBXCFG_MSG_RATES structure is
 *
 * \var msgClass
 *
 * \var msgId
 *
 * \var rate[6]
 *
*/

/*!
 * \struct UBXCFG_MSG_RATE
 * \brief The UBXCFG_MSG_RATE structure is
 *
 * \var msgClass
 *
 * \var msgId
 *
 * \var rate
 *
*/

/*!
 * \struct UBXCFG_NAV5_POLL
 * \brief The UBXCFG_NAV5_POLL structure is
 *
    //No payload data
*/

/*!
 * \struct UBXCFG_NAV5
 * \brief The UBXCFG_NAV5 structure is
 *
 * \var mask
 *
 * \note See #UBXNAV5Mask to fill this field
 * \var dynModel
 *
 * \note See #UBXNAV5Model to fill this field
 * \var fixMode
 *
 * \note See #UBXNAV5FixMode to fill this field
 * \var fixedAlt
 *
 * \var fixedAltVar
 *
 * \var minElev
 *
 * \var drLimit
 *
 * \var pDop
 *
 * \var tDop
 *
 * \var pAcc
 *
 * \var tAcc
 *
 * \var staticHoldThresh
 *
 * \var dgpsTimeOut
 *
 * \var cnoThreshNumSVs
 *
 * \var cnoThresh
 *
 * \var reserved2
 *  //Set to 0
 * \var reserved3
 *  //Set to 0
 * \var reserved4
 *  //Set to 0
*/

/*!
 * \struct UBXCFG_NAVX5_POLL
 * \brief The UBXCFG_NAVX5_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_NAVX5
 * \brief The UBXCFG_NAVX5 structure is
 *
 * \var version
 *
 * \var mask1
 *
 * \note See #UBXNAVX5Mask to fill this field
 * \var reserved0
 *  //Set 0
 * \var reserved1
 *  //Set 0
 * \var reserved2
 *  //Set 0
 * \var minSVs
 *
 * \var maxSVs
 *
 * \var minCNO
 *
 * \var reserved5
 *  //Set 0
 * \var iniFix3D
 *
 * \var reserved6
 *  //Set 0
 * \var reserved7
 *  //Set 0
 * \var reserved8
 *  //Set 0
 * \var wknRollover
 *
 * \var reserved9
 *  //Set 0
 * \var reserved10
 *  //Set 0
 * \var reserved11
 *  //Set 0
 * \var usePPP
 *
 * \var aopCFG
 *  // 0-disabled, 1 - enabled
 * \var reserved12
 *  //Set 0
 * \var reserved13
 *  //Set 0
 * \var aopOrbMaxErr
 *
 * \var reserved14
 *  //Set 0
 * \var reserved15
 *  //Set 0
 * \var reserved3
 *  //Set 0
 * \var reserved4
 *  //Set 0
*/

/*!
 * \struct UBXCFG_NMEA_POLL
 * \brief The UBXCFG_NMEA_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_NMEA
 * \brief The UBXCFG_NMEA structure is
 *
 * \var filter
 *
 * \note See #UBXNMEAFilter to fill this field
 * \var nmeaVersion
 *
 * \var numSV
 *
 * \var flags
 *
 * \note See #UBXNMEAFlags to fill this field
 * \var gnssToFilter
 *
 * \note See #UBXNMEAGNSSToFilter to fill this field
 * \var svNumbering
 *
 * \var mainTalkerId
 *
 * \var gsvTalkerId
 *
 * \var reserved
 *
*/

/*!
 * \struct UBXCFG_NVS
 * \brief The UBXCFG_NVS structure is
 *
 * \var clearMask
 *
 * \note See #UBXCFGMask CFG_NVS section to fill this field
 * \var saveMask
 *
 * \note See #UBXCFGMask CFG_NVS section to fill this field
 * \var loadMask
 *
 * \note See #UBXCFGMask CFG_NVS section to fill this field
 * \var deviceMask
 *
 * \note See #UBXCFGDeviceMask to fill this field
*/

/*!
 * \struct UBXCFG_PM2_POLL
 * \brief The UBXCFG_PM2_POLL structure is
 *
    //No payload
*/

/*!
             * \struct UBXCFG_PM2Flags
{
 * \var blank1:1
 *
 * \var reserved:3
 *
 * \var extIntSelect:1
 *
 * \var extIntWake:1
 *
 * \var extIntBackup:1
 *
 * \var blank2:1
 *
 * \var limitPeakCurr:2
 *
 * \note See #UBXPM2LimitPeakCurrent to fill this field
 * \var waitTimeFix:1
 *
 * \var updateRTC:1
 *
 * \var updateEPH:1
 *
 * \var blank3:3
 *
 * \var doNotEnterOff:1
 *
 * \var mode:2
 *
 * \note See #UBXPM2Mode to fill this field
*/

/*!
 * \struct UBXCFG_PM2
 * \brief The UBXCFG_PM2 structure is
 *
 * \var version
 *
 * \var reserved1
 *
 * \var reserved2
 *
 * \var reserved3
 *
    struct UBXCFG_PM2Flags flags;
 * \var updatePeriod
 *
 * \var searchPeriod
 *
 * \var gridOffset
 *
 * \var onTime
 *
 * \var minAcqTime
 *
 * \var reserved4
 *
 * \var reserved5
 *
 * \var reserved6
 *
 * \var reserved7
 *
 * \var reserved8
 *
 * \var reserved9
 *
 * \var reserved10
 *
 * \var reserved11
 *
*/

/*!
 * \struct UBXCFG_PRT_POLL
 * \brief The UBXCFG_PRT_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_PRT_POLL_OPT
 * \brief The UBXCFG_PRT_POLL_OPT structure is
 *
 * \var portId
 *
*/

/*!
 * \struct UBXCFG_PRTTxReady
 * \var en:1
 *  //0 - disabled, 1 - enabled
 * \var pol:1
 *  //0 - High-active, 1 - Low-active
 * \var pin:5
 *
 * \var thres:9
 *  //Given value is multiplied by 8 bytes
*/

/*!
 * \struct UBXCFG_PRTUARTMode
 * \var blank0:4
 *
 * \var reserved1:1
 *
 * \var blank1:1
 *
 * \var charLen:2
 *
 * \note See #UBXPRTModeCharLen to fill this field
 * \var blank2:1
 *
 * \var parity:3
 *
 * \note See #UBXPRTModeParity to fill this field
 * \var nStopBits:2
 *
 * \note See #UBXPRTModeStopBits to fill this field
 * \var blank3:18
 *
*/

/*!
 * \struct UBXCFG_PRTSPIMode
 * \var blank0:1
 *
 * \var spiMode:2
 *
 * \note See #UBXPRTSPIMode to fill this field
 * \var blank1:3
 *
 * \var flowControl:1
 *  //0 - disabled, 1 - enabled
 * \var blank2:1
 *
 * \var ffCnt:8
 *
 * \var blank3:16
 *
*/

/*!
 * \struct UBXCFG_PRTDDCMode
 * \var blank0:1
 *
 * \var slaveAddr:7
 *  //Range: 0x07 < slaveAddr < 0x78. Bit 0 shall be 0
 * \var blank1:24
 *
*/

/*!
 * \union UBXCFG_PRTMode
    struct UBXCFG_PRTUARTMode UART;
    struct UBXCFG_PRTSPIMode SPI;
    struct UBXCFG_PRTDDCMode DDC;
 * \var USB
 *  //reserved
*/

/*!
             * \union UBXCFG_PRT5Option
{
 * \var UARTbaudRate
 *
 * \var OtherReserved
 *
*/

/*!
             * \struct UBXCFG_PRT
{
 * \var portID
 *
 * \var reserved0
 *
    struct UBXCFG_PRTTxReady txReady;
    union UBXCFG_PRTMode mode;
    union UBXCFG_PRT5Option option;
 * \var inProtoMask
 *
 * \note See #UBXPRTInProtoMask to fill this field
 * \var outProtoMask
 *
 * \note See #UBXPRTOutProtoMask to fill this field
 * \var flags
 *
 * \note See #UBXPRTFlags to fill this field, shall be 0 for USB
 * \var reserved5
 *
*/

/*!
 * \struct UBXCFG_RATE_POLL
 * \brief The UBXCFG_RATE_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_RATE
 * \brief The UBXCFG_RATE structure is
 *
 * \var measRate
 *
 * \var navRate
 *
 * \var timeRef
 *
*/

/*!
 * \struct UBXCFG_RINV_POLL
 * \brief The UBXCFG_RINV_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_RINV
 * \brief The UBXCFG_RINV structure is
 *
 * \var flags
 *
 * \note See #UBXRINVFlags to fill this field
    //Variable payload size
*/

/*!
 * \struct UBXCFG_RST
 * \brief The UBXCFG_RST structure is
 *
 * \var navBBRMask
 *
 * \var resetMode
 *
 * \var reserved1
 *
*/

/*!
 * \struct UBXCFG_RXM_POLL
 * \brief The UBXCFG_RXM_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_RXM
 * \brief The UBXCFG_RXM structure is
 *
 * \var reserved1
 *  //Shall be set to 8
 * \var lpMode
 *
 * \note See #UBXRXMLowPowerModes to fill this field
*/

/*!
 * \struct UBXCFG_SBAS_POLL
 * \brief The UBXCFG_SBAS_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_SBAS
 * \brief The UBXCFG_SBAS structure is
 *
 * \var mode
 *
 * \note See #UBXSBASModes to fill this field
 * \var usage
 *
 * \note See #UBXSBASUsage to fill this field
 * \var maxSBAS
 *
 * \var scanmode2
 *
 * \note See #UBXSBASScanModes2 to fill this field
 * \var scanmode1
 *
 * \note See #UBXSBASScanModes1 to fill this field
*/

/*!
 * \struct UBXCFG_TP5_POLL
 * \brief The UBXCFG_TP5_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_TP5_POLL_OPT
 * \brief The UBXCFG_TP5_POLL_OPT structure is
 *
 * \var tpIdx
 *
*/

/*!
 * \struct UBXCFG_TP5
 * \brief The UBXCFG_TP5 structure is
 *
 * \var tpIdx
 *
 * \var reserved0
 *
 * \var reserved1
 *
 * \var antCableDelay
 *
 * \var rfGroupDelay
 *
 * \var freqPeriod
 *
 * \var freqPeriodLock
 *
 * \var pulseLenRatio
 *
 * \var pulseLenRatioLock
 *
 * \var userConfigDelay
 *
 * \var flags
 *
 * \note See #UBXCFGTimepulseFlags to fill this field
*/

/*!
 * \struct UBXCFG_USB_POLL
 * \brief The UBXCFG_USB_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXCFG_USB
 * \brief The UBXCFG_USB structure is
 *
 * \var vendorId
 *
 * \var productId
 *
 * \var reserved1
 * //Set to 0
 * \var reserved2
 * //Set to 1
 * \var powerConsumption
 *
 * \var flags
 *
 * \var vendorString[32]
 *
 * \var productString[32]
 *
 * \var serialNumber[32]
 *
*/

/*!
 * \struct UBXINF_DEBUG
 * \brief The UBXINF_DEBUG structure is
 *
    //Variable payload of UBXCH_t
*/

/*!
 * \struct UBXINF_ERROR
 * \brief The UBXINF_ERROR structure is
 *
    //Variable payload of UBXCH_t
*/

/*!
 * \struct UBXINF_NOTICE
 * \brief The UBXINF_NOTICE structure is
 *
    //Variable payload of UBXCH_t
*/

/*!
 * \struct UBXINF_TEST
 * \brief The UBXINF_TEST structure is
 *
    //Variable payload of UBXCH_t
*/

/*!
 * \struct UBXINF_WARNING
 * \brief The UBXINF_WARNING structure is
 *
    //Variable payload of UBXCH_t
*/

/*!
 * \struct UBXLOG_CREATE
 * \brief The UBXLOG_CREATE structure is
 *
 * \var version
 *
 * \var logCfg
 *  //See UBXLOGCfg
 * \var reserved
 *  //Set to 0
 * \var logSize
 *
 * \var userDefinedSize
 *
*/

/*!
 * \struct UBXLOG_ERASE
 * \brief The UBXLOG_ERASE structure is
 *
    //No payload
*/

/*!
 * \struct UBXLOG_FINDTIME_IN
 * \brief The UBXLOG_FINDTIME_IN structure is
 *
 * \var version
 *  //Shall be 0
 * \var type
 *  //Shall be 0
 * \var reserved1
 *
 * \var year
 *
 * \var month
 *
 * \var day
 *
 * \var hour
 *
 * \var minute
 *
 * \var second
 *
 * \var reserved2
 *
*/

/*!
 * \struct UBXLOG_FINDTIME_OUT
 * \brief The UBXLOG_FINDTIME_OUT structure is
 *
 * \var version
 *  //Shall be 1
 * \var type
 *  //Shall be 1
 * \var reserved1
 *
 * \var entryNumber
 *
*/

/*!
 * \struct UBXLOG_INFO_POLL
 * \brief The UBXLOG_INFO_POLL structure is
 *
    //No payload
*/

/*!
             * \struct UBXLOG_INFO
{
 * \var version
 *
 * \var reserved1[3]
 *
 * \var filestoreCapacity
 *
 * \var reserved2
 *
 * \var reserved3
 *
 * \var currentMaxLogSize
 *
 * \var currentLogSize
 *
 * \var entryCount
 *
 * \var oldestYear
 *
 * \var oldestMonth
 *
 * \var oldestDay
 *
 * \var oldestHour
 *
 * \var oldestMinute
 *
 * \var oldestSecond
 *
 * \var reserved4
 *
 * \var newestYear
 *
 * \var newestMonth
 *
 * \var newestDay
 *
 * \var newestHour
 *
 * \var newestMinute
 *
 * \var newestSecond
 *
 * \var reserved5
 *
 * \var status
 *
 * \note See #UBXLOGStatus to fill this field
 * \var reserved6[3]
 *
*/

/*!
 * \struct UBXLOG_RETRIEVEPOS
 * \brief The UBXLOG_RETRIEVEPOS structure is
 *
 * \var entryIndex
 *
 * \var lon
 *
 * \var lat
 *
 * \var hMSL
 *
 * \var hAcc
 *
 * \var gSpeed
 *
 * \var heading
 *
 * \var version
 *  //Shall be 0
 * \var fixType
 *
 * \note See #UBXRETRIEVEPOSFixType to fill this field
 * \var year
 *
 * \var month
 *
 * \var day
 *
 * \var hour
 *
 * \var minute
 *
 * \var second
 *
 * \var reserved1
 *
 * \var numSV
 *
 * \var reserved2
 *
*/

/*!
 * \struct UBXLOG_RETRIEVESTRING
 * \brief The UBXLOG_RETRIEVESTRING structure is
 *
 * \var entryIndex
 *
 * \var version
 *  //Shall be 0
 * \var reserved1
 *
 * \var year
 *
 * \var month
 *
 * \var day
 *
 * \var hour
 *
 * \var minute
 *
 * \var second
 *
 * \var reserved2
 *
 * \var byteCount
 *
    //Variable payload according byteCount
*/

/*!
 * \struct UBXLOG_RETRIEVE
 * \brief The UBXLOG_RETRIEVE structure is
 *
 * \var startNumber
 *
 * \var entryCount
 *
 * \var version
 *
 * \var reserved[3]
 *
*/

/*!
 * \struct UBXLOG_STRING
 * \brief The UBXLOG_STRING structure is
 *
    //Variable payload UBXU1_t
*/

/*!
 * \struct UBXMON_HW2
 * \brief The UBXMON_HW2 structure is
 *
 * \var ofsI
 *
 * \var magI
 *
 * \var ofsQ
 *
 * \var magQ
 *
 * \var cfgSource
 *
 * \var reserved0[3]
 *
 * \var lowLevCfg
 *
 * \var reserved1[2]
 *
 * \var postStatus
 *
 * \var reserved2
 *
*/

/*!
             * \struct UBXHWFlags
{
 * \var UBXHWFlagsRTCCalib:1
 *
 * \var UBXHWFlagsSafeBoot:1
 *
 * \var UBXHWFlagsJammingState:2
 *
*/

/*!
 * \struct UBXMON_HW
 * \brief The UBXMON_HW structure is
 *
 * \var pinSel
 *
 * \var pinBank
 *
 * \var pinDir
 *
 * \var pinVal
 *
 * \var noisePerMS
 *
 * \var agcCnt
 *
 * \var aStatus
 *
 * \var aPower
 *
    struct UBXHWFlags flags;
 * \var reserved1
 *
 * \var usedMask
 *
 * \var VP[17]
 *
 * \var jamInd
 *
 * \var reserved3
 *
 * \var pinIrq
 *
 * \var pullH
 *
 * \var pullL
 *
*/

/*!
             * \struct UBXMON_IO_PART
{
 * \var rxBytes
 *
 * \var txBytes
 *
 * \var parityErrs
 *
 * \var framingErrs
 *
 * \var overrunErrs
 *
 * \var breakCond
 *
 * \var rxBusy
 *
 * \var txBusy
 *
 * \var reserved1
 *
*/

/*!
 * \struct UBXMON_IO
 * \brief The UBXMON_IO structure is
 *
    struct UBXMON_IO_PART ioPortInfo[UBX_IO_PORTS_NUM];
*/

/*!
 * \struct UBXMON_MSGPP
 * \brief The UBXMON_MSGPP structure is
 *
 * \var msg1[8]
 *
 * \var msg2[8]
 *
 * \var msg3[8]
 *
 * \var msg4[8]
 *
 * \var msg5[8]
 *
 * \var msg6[8]
 *
 * \var skipped[6]
 *
*/

/*!
 * \struct UBXMON_RXBUF
 * \brief The UBXMON_RXBUF structure is
 *
 * \var pending[6]
 *
 * \var usage[6]
 *
 * \var peakUsage[6]
 *
*/

/*!
 * \struct UBXMON_RXR
 * \brief The UBXMON_RXR structure is
 *
 * \var flags
 *
 * \note See #UBXRXRFlags to fill this field
*/

/*!
 * \struct UBXMON_TXBUF
 * \brief The UBXMON_TXBUF structure is
 *
 * \var pending[6]
 *
 * \var usage[6]
 *
 * \var peakUsage[6]
 *
*/

/*!
 * \struct UBXMON_VER_POLL
 * \brief The UBXMON_VER_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXMON_VER
 * \brief The UBXMON_VER structure is
 *
 * \var swVersion[30]
 *
 * \var hwVersion[10]
 *
    //Variable payload of UBXMON_VER_PART type
*/
/*!
             * \struct UBXMON_VER_PART
{
 * \var extension[30]
 *
*/

/*!
 * \struct UBXNAV_AOPSTATUS
 * \brief The UBXNAV_AOPSTATUS structure is
 *
 * \var iTOW
 *
 * \var aopCfg
 *
 * \note See #UBXAOPCfg to fill this field
 * \var status
 *
 * \note See #UBXAOPStatus to fill this field
 * \var reserved0
 *
 * \var reserved1
 *
 * \var availGPS
 *
 * \var reserved2
 *
 * \var reserved3
 *
*/

/*!
 * \struct UBXNAV_CLOCK
 * \brief The UBXNAV_CLOCK structure is
 *
 * \var iTOW
 *
 * \var clkB
 *
 * \var clkD
 *
 * \var tAcc
 *
 * \var fAcc
 *
*/

/*!
 * \struct UBXNAV_DGPS
 * \brief The UBXNAV_DGPS structure is
 *
 * \var iTOW
 *
 * \var age
 *
 * \var baseId
 *
 * \var baseHealth
 *
 * \var numCh
 *
 * \var status
 *
 * \var reserved1
 *
*/
/*!
             * \struct UBXDGPSFlags
{
 * \var channel:4
 *
 * \var dgpsUsed:1
 *
*/

/*!
 * \struct UBXNAV_DGPS_PART
 * \brief The UBXNAV_DGPS_PART structure is
 *
 * \var svid
 *
    struct UBXDGPSFlags flags;
 * \var ageC
 *
 * \var prc
 *
 * \var prrc
 *
*/

/*!
 * \struct UBXNAV_DOP
 * \brief The UBXNAV_DOP structure is
 *
 * \var iTOW
 *
 * \var gDOP
 *
 * \var pDOP
 *
 * \var tDOP
 *
 * \var vDOP
 *
 * \var hDOP
 *
 * \var nDOP
 *
 * \var eDOP
 *
*/

/*!
 * \struct UBXNAV_POSECEF
 * \brief The UBXNAV_POSECEF structure is
 *
 * \var iTOW
 *
 * \var ecefX
 *
 * \var ecefY
 *
 * \var ecefZ
 *
 * \var pAcc
 *
*/

/*!
 * \struct UBXNAV_POSLLH
 * \brief The UBXNAV_POSLLH structure is
 *
 * \var iTOW
 *
 * \var lon
 *
 * \var lat
 *
 * \var height
 *
 * \var hMSL
 *
 * \var hAcc
 *
 * \var vAcc
 *
*/

/*!
             * \struct UBXPVTFlags
{
 * \var gnssFixOk:1
 *
 * \var diffSoln:1
 *
 * \var psmState:3
 *
 * \note See #UBXPVTPSMStates to fill this field
*/

/*!
 * \struct UBXNAV_PVT
 * \brief The UBXNAV_PVT structure is
 *
 * \var iTOW
 *
 * \var year
 *
 * \var month
 *
 * \var day
 *
 * \var hour
 *
 * \var min
 *
 * \var sec
 *
 * \var valid
 *
 * \note See #UBXPVTValid to fill this field
 * \var tAcc
 *
 * \var nano
 *
 * \var fixType
 *
 * \note See #UBXGPSFix to fill this field
    struct UBXPVTFlags flags;
 * \var reserved1
 *
 * \var numSV
 *
 * \var lon
 *
 * \var lat
 *
 * \var height
 *
 * \var hMSL
 *
 * \var hAcc
 *
 * \var vAcc
 *
 * \var velN
 *
 * \var velE
 *
 * \var velD
 *
 * \var gSpeed
 *
 * \var heading
 *
 * \var sAcc
 *
 * \var headingAcc
 *
 * \var pDOP
 *
 * \var reserved2
 *
 * \var reserved3
 *
*/

/*!
 * \struct UBXNAV_SBAS
 * \brief The UBXNAV_SBAS structure is
 *
 * \var iTOW
 *
 * \var geo
 *
 * \var mode
 *
 * \var sys
 *
 * \var service
 *
 * \note See #UBXSBASService to fill this field
 * \var cnt
 *
 * \var reserved0[3]
 *
    //Variable payload of UBXNAV_SBAS_PART type
*/

/*!
 * \struct UBXNAV_SBAS_PART
 * \brief The UBXNAV_SBAS_PART structure is
 *
 * \var svid
 *
 * \var flags
 *
 * \var udre
 *
 * \var svSys
 *
 * \var svService
 *
 * \var reserved1
 *
 * \var prc
 *
 * \var reserved2
 *
 * \var ic
 *
*/

/*!
 * \struct UBXNAV_SOL
 * \brief The UBXNAV_SOL structure is
 *
 * \var iTOW
 *
 * \var fTOW
 *
 * \var week
 *
 * \var gpsFix
 *
 * \note See #UBXGPSFix to fill this field
 * \var flags
 *
 * \note See #UBXSBASSOLFlags to fill this field
 * \var ecefX
 *
 * \var ecefY
 *
 * \var ecefZ
 *
 * \var pAcc
 *
 * \var ecefVX
 *
 * \var ecefVY
 *
 * \var ecefVZ
 *
 * \var sAcc
 *
 * \var pDOP
 *
 * \var reserved1
 *
 * \var numSV
 *
 * \var reserved2
 *
*/

/*!
 * \struct UBXNAV_STATUS
 * \brief The UBXNAV_STATUS structure is
 *
 * \var iTOW
 *
 * \var gpsFix
 *
 * \var flags
 *
 * \note See #UBXGPSFix to fill this field
 * \var fixStat
 *
 * \note See #UBXSBASSOLFlags to fill this field
 * \var flags2
 *
 * \var ttff
 *
 * \var msss
 *
*/

/*!
             * \struct UBXSVINFOGlobalFlags
{
 * \var chipGen:3
 *
 * \note See #UBXSVINFOChipGen to fill this field
*/

/*!
 * \struct UBXNAV_SVINFO
 * \brief The UBXNAV_SVINFO structure is
 *
 * \var iTOW
 *
 * \var numCh
 *
    struct UBXSVINFOGlobalFlags globalFlags;
 * \var reserved2
 *
    //Variable payload of UBXNAV_SVINFO_PART type
*/

/*!
             * \struct UBXSVINFOQuality
{
 * \var qualityInd:4
 *
 * \note See #UBXSVINFOQualityId to fill this field
*/

/*!
 * \struct UBXNAV_SVINFO_PART
 * \brief The UBXNAV_SVINFO_PART structure is
 *
 * \var chn
 *
 * \var svid
 *
 * \var flags
 *
 * \note See #UBXSVINFOFlags to fill this field
 * \var quality
 *
 * \var cno
 *
 * \var elev
 *
 * \var azim
 *
 * \var prRes
 *
*/

/*!
 * \struct UBXNAV_TIMEGPS
 * \brief The UBXNAV_TIMEGPS structure is
 *
 * \var iTOW
 *
 * \var fTOW
 *
 * \var week
 *
 * \var leapS
 *
 * \var valid
 *
 * \note See #UBXTIMEGPSValidityFlags to fill this field
 * \var tAcc
 *
*/

/*!
 * \struct UBXNAV_TIMEUTC
 * \brief The UBXNAV_TIMEUTC structure is
 *
 * \var iTOW
 *
 * \var tAcc
 *
 * \var nano
 *
 * \var year
 *
 * \var month
 *
 * \var day
 *
 * \var hour
 *
 * \var min
 *
 * \var sec
 *
 * \var valid
 *
 * \note See #UBXTIMEUTCValidityFlags to fill this field
*/

/*!
 * \struct UBXNAV_VELECEF
 * \brief The UBXNAV_VELECEF structure is
 *
 * \var iTOW
 *
 * \var ecefVX
 *
 * \var ecefVY
 *
 * \var ecefVZ
 *
 * \var sAcc
 *
*/

/*!
 * \struct UBXNAV_VELNED
 * \brief The UBXNAV_VELNED structure is
 *
 * \var iTOW
 *
 * \var velN
 *
 * \var velE
 *
 * \var velD
 *
 * \var speed
 *
 * \var gSpeed
 *
 * \var heading
 *
 * \var sAcc
 *
 * \var cAcc
 *
*/

/*!
 * \struct UBXRXM_ALM_POLL
 * \brief The UBXRXM_ALM_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXRXM_ALM_POLL_OPT
 * \brief The UBXRXM_ALM_POLL_OPT structure is
 *
 * \var svid
 *
*/

/*!
 * \struct UBXRXM_ALM
 * \brief The UBXRXM_ALM structure is
 *
 * \note This RMX messages marked as obsolete API use AID instead
 * \var svid
 *
 * \var week
 *
*/

/*!
 * \struct UBXRXM_ALM_OPT
 * \brief The UBXRXM_ALM_OPT structure is
 * \note This RMX messages marked as obsolete API use AID instead
 * \var svid
 *
 * \var week
 *
 * \var dwrd[8]
 *
*/

/*!
 * \struct UBXRXM_EPH_POLL
 * \brief The UBXRXM_EPH_POLL structure is
 *
    //No payload
*/

/*!
 * \struct UBXRXM_EPH_POLL_OPT
 * \brief The UBXRXM_EPH_POLL_OPT structure is
 *
 * \var svid
 *
*/

/*!
 * \struct UBXRXM_EPH
 * \brief The UBXRXM_EPH structure is
 *
 * \note This RMX messages marked as obsolete API use AID instead
 * \var svid
 *
 * \var how
 *
*/

/*!
 * \struct UBXRXM_EPH_OPT
 * \brief The UBXRXM_EPH_OPT structure is
 *
 * \note This RMX messages marked as obsolete API use AID instead
 * \var svid
 *
 * \var how
 *
 * \var sf1d[8]
 *
 * \var sf2d[8]
 *
 * \var sf3d[8]
 *
*/

/*!
 * \struct UBXRXM_PMREQ
 * \brief The UBXRXM_PMREQ structure is
 *
 * \var duration
 *
 * \var flags
 *
 * \note See #UBXPMREQFlags to fill this field
*/

/*!
 * \struct UBXRXM_RAW
 * \brief The UBXRXM_RAW structure is
 *
 * \var rcvTow
 *
 * \var week
 *
 * \var numSV
 *
 * \var reserved1
 *
 //Variable payload of UBXRXM_RAW_PART type
*/

/*!
 * \struct UBXRXM_RAW_PART
 * \brief The UBXRXM_RAW_PART structure is
 * \var cpMes
 *
 * \var prMes
 *
 * \var doMes
 *
 * \var sv
 *
 * \var mesQI
 *
 * \var cno
 *
 * \var lli
 *
*/

/*!
 * \struct UBXRXM_SFRB
 * \brief The UBXRXM_SFRB structure is
 *
 * \var chn
 *
 * \var svid
 *
 * \var dwrd[10]
 *
*/

/*!
 * \struct UBXRXM_SVSI
 * \brief The UBXRXM_SVSI structure is
 *
 * \var iTOW
 *
 * \var week
 *
 * \var numVis
 *
 * \var numSV
 *
 * Variable payload of UBXRXM_SVSI_PART type
*/

/*!
 * \struct UBXSVSISVFlags
{
 * \var ura:4
 *
 * \var healthy:1
 *
 * \var ephVal:1
 *
 * \var almVal:1
 *
 * \var notAvail:1
 *
*/

/*!
* \struct UBXSVSIAge
* \var almAge:4
*
* \var ephAge:4
*
*/

/*!
* \struct UBXRXM_SVSI_PART
* \var svid
*
struct UBXSVSISVFlags svFlag;
* \var azim
*
* \var elev
*
struct UBXSVSIAge age;
*/


/*!
* \struct UBXTM2Flags
* \var mode:1
*
* \note See #UBXTM2FlagsMode to fill this field
* \var run:1
*
* \note See #UBXTM2FlagsRun to fill this field
* \var newFallingEdge:1
*
* \var timeBase:2
*
* \note See #UBXTM2FlagsTimeBase to fill this field
* \var utc:1
*
* \note See #UBXTM2FlagsUTC to fill this field
* \var time:1
*
* \note See #UBXTM2FlagsTime to fill this field
* \var newRisingEdge:1
*
*/

/*!
* \struct UBXTIM_TM2
* \brief The UBXTIM_TM2 structure is
*
* \var ch
*
struct UBXTM2Flags flags;
* \var count
*
* \var wnR
*
* \var wnF
*
* \var towMsR
*
* \var towSubMsR
*
* \var towMsF
*
* \var towSubMsF
*
* \var accEst
*
*/

/*!
* \struct UBXTIM_TP
* \brief The UBXTIM_TP structure is
*
* \var towMS
*
* \var towSubMS
*
* \var qErr
*
* \var week
*
* \var flags
*
* \var reserved1
*
*/

/*!
* \struct UBXVRFYFlags
* \brief The UBXVRFYFlags structure is
*
* \var UBXVRFYFlags::src
* Blabla variable
* \note See #UBXVRFYFlagsSource to fill this field
*/

/*!
* \struct UBXTIM_VRFY
* \brief The UBXTIM_VRFY structure is
*
* \var itow
*
* \var frac
*
* \var deltaMs
*
* \var deltaNs
*
* \var wno
*
* \var flags;
*
* \var reserved1
*
*/







