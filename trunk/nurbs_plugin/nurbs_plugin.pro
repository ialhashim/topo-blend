include($$[STARLAB])
include($$[SURFACEMESH])
include($$[CHOLMOD])

include($$[NANOFLANN])
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
    LSCM.h \
	BoundaryFitting.h

SOURCES += nurbs_plugin.cpp \
    nurbstools.cpp \
	BoundaryFitting.cpp
	
RESOURCES += nurbs_plugin.qrc

FORMS += nurbstools.ui

mac:LIBS += -framework CoreFoundation # We need this for GLee
