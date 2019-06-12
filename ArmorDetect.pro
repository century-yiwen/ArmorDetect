QT += core
QT -= gui

CONFIG += c++11

TARGET = ArmorDetect
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app
QMAKE_CXXFLAGS_RELEASE += -O3 # Release -O3
SOURCES +=     Anglesolve.cpp \
    ArmorDetect.cpp \
    base_thread.cpp \
    UART.cpp \
    usb_capture_with_thread.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    Anglesolve.h \
    base_thread.h \
    UART.h \
    usb_capture_with_thread.h


INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
               /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_* \
