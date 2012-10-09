system(qmake -set QMAKEFEATURES $$PWD/../starlab/starlab-core)
TEMPLATE = subdirs
CONFIG += ordered

SUBDIRS += NURBS
SUBDIRS += segment
SUBDIRS += resample
SUBDIRS += dynamic_voxel
SUBDIRS += nurbs_plugin
SUBDIRS += topo-blend
