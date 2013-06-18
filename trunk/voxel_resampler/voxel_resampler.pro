include($$[STARLAB])
include($$[SURFACEMESH])
include($$[CHOLMOD])

include($$[NANOFLANN])
StarlabTemplate(plugin)

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# Voxeler library
LIBS += -L$$PWD/../Voxeler/$$CFG/lib -lVoxeler
INCLUDEPATH += ../Voxeler

# DynamicVoxel library
LIBS += -L$$PWD/../DynamicVoxel/$$CFG/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

HEADERS += voxel_resampler.h
SOURCES += voxel_resampler.cpp
