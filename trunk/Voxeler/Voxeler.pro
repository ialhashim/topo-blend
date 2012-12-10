load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
load($$[NANOFLANN])

TEMPLATE = lib
CONFIG += staticlib
QT += opengl

# Library name and destination
TARGET = Voxeler
DESTDIR = $$PWD/lib

# Source code
SOURCES += Voxeler.cpp
HEADERS += Voxeler.h Voxel.h

SOURCES += BoundingBox.cpp
HEADERS += BoundingBox.h
