load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
StarlabTemplate(plugin)

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# DynamicVoxel library
LIBS += -L$$PWD/../DynamicVoxel/$$CFG/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

HEADERS += dynamic_voxel.h
SOURCES += dynamic_voxel.cpp
RESOURCES += dynamic_voxel.qrc
