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

struct UBXMsgBuffer getCFG_MSG_RATE(enum UBXMessageClass msgClass, enum UBXMessageId msgId, u_int8_t rate)
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

struct UBXMsgBuffer getCFG_MSG_RATES(enum UBXMessageClass msgClass, enum UBXMessageId msgId, u_int8_t rate[6])
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

struct UBXMsgBuffer getCFG_ANT(){}
struct UBXMsgBuffer getCFG_ANT_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_ANT_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_ANT);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_CFG(){}
struct UBXMsgBuffer getCFG_CFG_OPT(){}
struct UBXMsgBuffer getCFG_DAT_IN(){}

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

struct UBXMsgBuffer getCFG_GNSS(){}
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

struct UBXMsgBuffer getCFG_INF(){}
struct UBXMsgBuffer getCFG_ITFM_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_ITFM_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_ITFM);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_ITFM(){}
struct UBXMsgBuffer getCFG_LOGFILTER_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_LOGFILTER_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_LOGFILTER);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_LOGFILTER(){}
struct UBXMsgBuffer getCFG_NAV5_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_NAV5_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_NAV5);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_NAV5(){}
struct UBXMsgBuffer getCFG_NAVX5_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_NAVX5_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_NAVX5);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_NAVX5(){}
struct UBXMsgBuffer getCFG_NMEA_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_NMEA_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_NMEA);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_NMEA(){}
struct UBXMsgBuffer getCFG_NVS(){}
struct UBXMsgBuffer getCFG_PM2_POLL()
{
    int payloadSize = sizeof(struct UBXCFG_PM2_POLL);
    struct UBXMsgBuffer buffer = createBuffer(payloadSize);
    struct UBXMsg* msg = (struct UBXMsg*)buffer.data;
    initMsg(msg, payloadSize, UBXMsgClassCFG, UBXMsgIdCFG_PM2);
    completeMsg(&buffer, payloadSize);
    return buffer;
}

struct UBXMsgBuffer getCFG_PM2(){}
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

struct UBXMsgBuffer getCFG_PRT(){}
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
