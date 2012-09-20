CONFIG += starlab
STARLAB_TEMPLATE += plugin
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh
STARLAB_EXTERNAL += cholmod eigen

HEADERS += geodistance.h
SOURCES += geodistance.cpp

HEADERS += GeoHeatHelper.h
