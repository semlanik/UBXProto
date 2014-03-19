#-------------------------------------------------
#
# Project created by QtCreator 2014-02-26T14:20:56
#
#-------------------------------------------------
CONFIG -= qt

TARGET = ubloxproto
TEMPLATE = lib

DEFINES += UBLOXPROTO_LIBRARY

SOURCES += \
    ublox.c

HEADERS +=\
        qublox_global.h \
    ubxmessage.h \
    ublox.h \
    portable_endian.h \
    ubloxaid.h \
    ubloxcfg.h \
    ubloxlog.h \
    ubloxmon.h \
    ubloxrxm.h \
    ubloxutils.h

header_files.files = $$HEADERS
header_files.path = ../include
target.path = ../lib
DEPLOYMENT += header_files target
INSTALLS += header_files
INSTALLS += target
