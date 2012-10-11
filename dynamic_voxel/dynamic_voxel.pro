CONFIG += starlab
STARLAB_TEMPLATE += plugin 
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh
STARLAB_EXTERNAL += nanoflann eigen cholmod

# DynamicVoxel library
LIBS += -L$$PWD/../DynamicVoxel/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

HEADERS += dynamic_voxel.h
SOURCES += dynamic_voxel.cpp
RESOURCES += dynamic_voxel.qrc
