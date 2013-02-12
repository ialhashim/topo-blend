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

# TopoBlender library
LIBS += -L$$PWD/../TopoBlenderLib/$$CFG/lib -lTopoBlenderLib
INCLUDEPATH += ../TopoBlenderLib

HEADERS += topo-blend.h \ 
    QuickMesh.h \
    QuickViewer.h \
    topo_blend_widget.h \
    graph_modify_dialog.h \
    landmarks_dialog.h

SOURCES +=  topo-blend.cpp \
            topo_blend_widget.cpp \
            graph_modify_dialog.cpp \
			landmarks_dialog.cpp
	
RESOURCES += topo-blend.qrc

FORMS +=    topo_blend_widget.ui \
            animationWidget.ui \
            graph_modify_dialog.ui \
			landmarks_dialog.ui

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
