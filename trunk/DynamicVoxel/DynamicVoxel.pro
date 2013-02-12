load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])

TEMPLATE = lib
CONFIG += staticlib
QT += opengl

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# Library name and destination
TARGET = DynamicVoxel
DESTDIR = $$PWD/$$CFG/lib

SOURCES += DynamicVoxel.cpp
HEADERS += Voxel.h DynamicVoxel.h DoubleTupleMap.h
