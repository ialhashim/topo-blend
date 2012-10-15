QT += opengl
CONFIG += starlab

STARLAB_DEPENDS *= common
STARLAB_DEPENDS *= ../../starlab/plugins-surfacemesh/surfacemesh
STARLAB_EXTERNAL += nanoflann eigen cholmod

TARGET = DynamicVoxel
TEMPLATE = lib
CONFIG += staticlib
DESTDIR = $$PWD/lib

SOURCES += DynamicVoxel.cpp
HEADERS += Voxel.h DynamicVoxel.h DoubleTupleMap.h
