system(qmake -set QMAKEFEATURES $$PWD/../starlab/starlab-core)
TEMPLATE = subdirs
CONFIG += ordered

SUBDIRS += NURBS
SUBDIRS += resample
SUBDIRS += segment
SUBDIRS += dynamic_voxel
SUBDIRS += nurbs_plugin
SUBDIRS += topo-blend

topo-blend.depends = NURBS
nurbs_plugin.depends = NURBS
