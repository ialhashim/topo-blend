load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
load($$[NANOFLANN])
StarlabTemplate(plugin)

# NURBS library
LIBS += -L$$PWD/../NURBS/lib -lNURBS
INCLUDEPATH += ../NURBS

# VOXEL library - Should not be used..
LIBS += -L$$PWD/../DynamicVoxel/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

# TopoBlender library
LIBS += -L$$PWD/../TopoBlenderLib/lib -lTopoBlenderLib
INCLUDEPATH += ../TopoBlenderLib

HEADERS += nurbs_plugin.h \
    nurbstools.h \
    OBB_Volume.h \
    PCA3.h \
    LSCM.h

SOURCES += nurbs_plugin.cpp \
    nurbstools.cpp
RESOURCES += nurbs_plugin.qrc

FORMS += \
    nurbstools.ui
