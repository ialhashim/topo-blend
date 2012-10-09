CONFIG += starlab 
STARLAB_TEMPLATE += plugin 
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh

# NURBS library
LIBS += -L$$PWD/../NURBS/lib -lNURBS
INCLUDEPATH += ../NURBS

HEADERS += nurbs_plugin.h
SOURCES += nurbs_plugin.cpp
RESOURCES += nurbs_plugin.qrc
