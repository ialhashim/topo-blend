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
SUBDIRS += voxel_resampler
SUBDIRS += nurbs_plugin

# Legacy
#SUBDIRS += resample
#SUBDIRS += segment
#SUBDIRS += visiblity_resampler
#SUBDIRS += dynamic_voxel

SUBDIRS += demo         # Standalone demo
SUBDIRS += topo-blend   # Main plugin for topo-blending

#SUBDIRS += test         # Performance test

# Dependecy map
#dynamic_voxel.depends = DynamicVoxel
nurbs_plugin.depends = NURBS
TopoBlenderLib.depends = GlSplatRendererLib
topo-blend.depends = GlSplatRendererLib NURBS DynamicVoxel TopoBlenderLib 
demo.depends = GlSplatRendererLib NURBS DynamicVoxel TopoBlenderLib
