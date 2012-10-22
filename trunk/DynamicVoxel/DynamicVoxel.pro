load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
StarlabTemplate(plugin)

TARGET = DynamicVoxel
TEMPLATE = lib
CONFIG += staticlib
DESTDIR = $$PWD/lib

SOURCES += DynamicVoxel.cpp
HEADERS += Voxel.h DynamicVoxel.h DoubleTupleMap.h
