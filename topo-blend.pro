TEMPLATE = subdirs
CONFIG += ordered

# Libraries
SUBDIRS += NURBS
SUBDIRS += DynamicVoxel
SUBDIRS += Voxeler
SUBDIRS += TopoBlenderLib
SUBDIRS += Reconstruction
SUBDIRS += GlSplatRendererLib

# Plugins
SUBDIRS += resample
SUBDIRS += segment
SUBDIRS += dynamic_voxel
SUBDIRS += voxel_resampler
SUBDIRS += nurbs_plugin
#SUBDIRS += geometry_morph

SUBDIRS += topo-blend # Main UI for topo-blending

# Dependecy map
nurbs_plugin.depends = NURBS
dynamic_voxel.depends = DynamicVoxel
geometry_morph.depends = MorpherLib
topo-blend.depends = GlSplatRendererLib NURBS DynamicVoxel TopoBlenderLib MorpherLib 

# OpenMP
win32 {
    QMAKE_CXXFLAGS += /openmp
    export(QMAKE_CXXFLAGS_DEBUG)
    export(QMAKE_CXXFLAGS_RELEASE)
}
unix {
    QMAKE_CXXFLAGS += -fopenmp
    LIBS += -lgomp
}
