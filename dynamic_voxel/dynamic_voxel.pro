CONFIG += starlab 
STARLAB_TEMPLATE += plugin 
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh
STARLAB_EXTERNAL += nanoflann

HEADERS += dynamic_voxel.h dynamic_voxel.h \
    DynamicVoxel.h \
    Voxel.h
SOURCES += dynamic_voxel.cpp \
    DynamicVoxel.cpp
RESOURCES += dynamic_voxel.qrc
