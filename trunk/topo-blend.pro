system(qmake -set QMAKEFEATURES $$PWD/../starlab/starlab-core)
TEMPLATE = subdirs
CONFIG += ordered

SUBDIRS += segment
SUBDIRS += surfacemesh_mode_info

