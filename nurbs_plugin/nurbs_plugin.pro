CONFIG += starlab 
STARLAB_TEMPLATE += plugin 
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh

HEADERS += nurbs_plugin.h \
    NURBS/KnotVector.h \
    NURBS/NurbsCurve.h \
    NURBS/NurbsDraw.h
SOURCES += nurbs_plugin.cpp
RESOURCES += nurbs_plugin.qrc
