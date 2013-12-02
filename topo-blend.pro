TEMPLATE = subdirs
CONFIG += ordered

# Libraries
SUBDIRS += NURBS
SUBDIRS += DynamicVoxel
SUBDIRS += Voxeler
SUBDIRS += ScorerLib
SUBDIRS += TopoBlenderLib
SUBDIRS += Reconstruction
SUBDIRS += GlSplatRendererLib

# Plugins
SUBDIRS += voxel_resampler
#SUBDIRS += nurbs_plugin

# Legacy
#SUBDIRS += resample
#SUBDIRS += segment
#SUBDIRS += visiblity_resampler
#SUBDIRS += dynamic_voxel

SUBDIRS += demo         # Standalone demo
SUBDIRS += topo-blend   # Main plugin for topo-blending

#SUBDIRS += test        # Performance test

# Dependecy map
nurbs_plugin.depends = NURBS
TopoBlenderLib.depends = GlSplatRendererLib
topo-blend.depends = GlSplatRendererLib NURBS DynamicVoxel ScorerLib TopoBlenderLib 
demo.depends = GlSplatRendererLib NURBS DynamicVoxel ScorerLib TopoBlenderLib
