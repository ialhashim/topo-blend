CONFIG += starlab
STARLAB_TEMPLATE += plugin
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh

QT += opengl

HEADERS += resample.h
SOURCES += resample.cpp
RESOURCES += resample.qrc
