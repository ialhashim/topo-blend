TEMPLATE = subdirs
CONFIG += ordered

# Libraries
SUBDIRS += NURBS
SUBDIRS += DynamicVoxel
SUBDIRS += Voxeler
SUBDIRS += TopoBlenderLib

# Plugins
SUBDIRS += resample
SUBDIRS += segment
SUBDIRS += dynamic_voxel
SUBDIRS += voxel_resampler
SUBDIRS += nurbs_plugin
SUBDIRS += geometry_morph

SUBDIRS += topo-blend # Main UI for topo-blending

# Dependecy map
topo-blend.depends = NURBS DynamicVoxel
nurbs_plugin.depends = NURBS
dynamic_voxel.depends = DynamicVoxel
