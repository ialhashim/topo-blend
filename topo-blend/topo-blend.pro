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

# TopoBlender library
LIBS += -L$$PWD/../TopoBlenderLib/$$CFG/lib -lTopoBlenderLib
INCLUDEPATH += ../TopoBlenderLib

# Splat Rendering library
LIBS += -L$$PWD/../GLSplatRendererLib/$$CFG/lib -lGLSplatRendererLib
INCLUDEPATH += ../GLSplatRendererLib

HEADERS += topo-blend.h \ 
    QuickMesh.h \
    QuickViewer.h \
    topo_blend_widget.h \
    graph_modify_dialog.h \
    landmarks_dialog.h \
    QuickAlignment.h \
    QuickGroup.h

SOURCES +=  topo-blend.cpp \
            topo_blend_widget.cpp \
            graph_modify_dialog.cpp \
			landmarks_dialog.cpp \
    QuickAlignment.cpp \
    QuickGroup.cpp
	
RESOURCES += topo-blend.qrc

FORMS +=    topo_blend_widget.ui \
            animationWidget.ui \
            graph_modify_dialog.ui \
			landmarks_dialog.ui \
    QuickAlignment.ui \
    QuickGroup.ui

mac:LIBS += -framework CoreFoundation # We need this for GLee
