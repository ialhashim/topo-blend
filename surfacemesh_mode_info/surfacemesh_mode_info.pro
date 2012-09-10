CONFIG += starlab 
STARLAB_TEMPLATE += plugin 
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh

QT += opengl

HEADERS += surfacemesh_mode_info.h 
SOURCES += surfacemesh_mode_info.cpp
RESOURCES += surfacemesh_mode_info.qrc

# Windows warnings
win32: DEFINES += _CRT_SECURE_NO_WARNINGS
