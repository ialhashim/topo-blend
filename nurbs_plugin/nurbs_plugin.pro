load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
load($$[NANOFLANN])
StarlabTemplate(plugin)

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# NURBS library
LIBS += -L$$PWD/../NURBS/$$CFG/lib -lNURBS
INCLUDEPATH += ../NURBS

# VOXEL library
LIBS += -L$$PWD/../DynamicVoxel/$$CFG/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

# Surface Reconstruction library
LIBS += -L$$PWD/../Reconstruction/$$CFG/lib -lReconstruction
INCLUDEPATH += ../Reconstruction

# Splat Rendering library
LIBS += -L$$PWD/../GLSplatRendererLib/$$CFG/lib -lGLSplatRendererLib
INCLUDEPATH += ../GLSplatRendererLib

# TopoBlender library
LIBS += -L$$PWD/../TopoBlenderLib/$$CFG/lib -lTopoBlenderLib
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

# OpenMP
win32 {
    QMAKE_CXXFLAGS += /openmp
    export(QMAKE_CXXFLAGS_DEBUG)
    export(QMAKE_CXXFLAGS_RELEASE)
}
unix {
    QMAKE_CXXFLAGS += -fopenmp
    LIBS += -lgomp
}
mac {
    LIBS += -framework CoreFoundation
}
