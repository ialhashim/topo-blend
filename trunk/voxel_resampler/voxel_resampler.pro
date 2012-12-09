load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
load($$[NANOFLANN])
StarlabTemplate(plugin)

# Voxeler library
LIBS += -L$$PWD/../Voxeler/lib -lVoxeler
INCLUDEPATH += ../Voxeler

# DynamicVoxel library
LIBS += -L$$PWD/../DynamicVoxel/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

HEADERS += voxel_resampler.h
SOURCES += voxel_resampler.cpp
