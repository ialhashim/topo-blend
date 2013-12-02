include($$[STARLAB])
include($$[SURFACEMESH])
include($$[CHOLMOD])

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

unix:!mac:QMAKE_CXXFLAGS = $$QMAKE_CFLAGS -fpermissive
