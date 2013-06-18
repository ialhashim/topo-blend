include($$[STARLAB])
include($$[SURFACEMESH])
include($$[CHOLMOD])

include($$[NANOFLANN])

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
TARGET = Voxeler
DESTDIR = $$PWD/$$CFG/lib

# Source code
SOURCES += Voxeler.cpp
HEADERS += Voxeler.h Voxel.h

SOURCES += BoundingBox.cpp
HEADERS += BoundingBox.h
