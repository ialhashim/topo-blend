system(qmake -set QMAKEFEATURES $$PWD/../starlab/starlab-core)
TEMPLATE = subdirs
CONFIG += ordered

SUBDIRS += segment \ 
    geodistance
SUBDIRS += resample
