# -------------------------------------------------
# Project created by QtCreator 2012-03-21T20:02:35
# -------------------------------------------------
QT += core
QT -= gui
TARGET = Multicopter_Simulator
CONFIG += console
CONFIG += qdbus
CONFIG -= app_bundle
TEMPLATE = app
SOURCES += main.cpp \
    MulticopterSimulator.cpp \
    aiobject.cpp
HEADERS += MulticopterSimulator.h \
    aiobject.h
OTHER_FILES += README
