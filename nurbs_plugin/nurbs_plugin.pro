load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
load($$[NANOFLANN])
StarlabTemplate(plugin)

# NURBS library
LIBS += -L$$PWD/../NURBS/lib -lNURBS
INCLUDEPATH += ../NURBS

HEADERS += nurbs_plugin.h \
    nurbstools.h \
    OBB_Volume.h

SOURCES += nurbs_plugin.cpp \
    nurbstools.cpp
RESOURCES += nurbs_plugin.qrc

FORMS += \
    nurbstools.ui
