system(qmake -set QMAKEFEATURES $$PWD/../starlab/starlab-core)
TEMPLATE = subdirs
CONFIG += ordered

# Libraries
SUBDIRS += NURBS
SUBDIRS += DynamicVoxel

# Plugins
SUBDIRS += resample
SUBDIRS += segment
SUBDIRS += dynamic_voxel
SUBDIRS += nurbs_plugin
SUBDIRS += topo-blend

# Dependecy map
topo-blend.depends = NURBS
nurbs_plugin.depends = NURBS
dynamic_voxel.depends = DynamicVoxel
