#-------------------------------------------------
#
# Project created by QtCreator 2014-02-26T14:20:56
#
#-------------------------------------------------
CONFIG -= qt

TARGET = ubxproto
TEMPLATE = lib

DEFINES += UBLOXPROTO_LIBRARY

SOURCES += \
    ubx.c

HEADERS +=\
    ubxmessage.h \
    ubx.h \
    portable_endian.h \
    ubxaid.h \
    ubxcfg.h \
    ubxlog.h \
    ubxmon.h \
    ubxrxm.h \
    ubxutils.h

OBJECTS_DIR = .obj
MOC_DIR = .moc

linux|linux-g++ {
    header_files.files = $$HEADERS
    header_files.path = $$PREFIX/usr/include
    target.path = $$PREFIX/usr/lib
    DEPLOYMENT += header_files target
    INSTALLS += header_files
    INSTALLS += target
}

win32 {
    LIBS += ws2_32
}
