load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
load($$[NANOFLANN])
StarlabTemplate(plugin)

# DynamicVoxel library
LIBS += -L$$PWD/../DynamicVoxel/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

HEADERS += dynamic_voxel.h
SOURCES += dynamic_voxel.cpp
RESOURCES += dynamic_voxel.qrc
