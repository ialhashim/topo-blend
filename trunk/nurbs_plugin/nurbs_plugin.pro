load($$[STARLAB])
load($$[SURFACEMESH])
StarlabTemplate(plugin)

# NURBS library
LIBS += -L$$PWD/../NURBS/lib -lNURBS
INCLUDEPATH += ../NURBS

HEADERS += nurbs_plugin.h
SOURCES += nurbs_plugin.cpp
RESOURCES += nurbs_plugin.qrc
