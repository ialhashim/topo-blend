load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
load($$[NANOFLANN])
StarlabTemplate(plugin)

# NURBS library
LIBS += -L$$PWD/../NURBS/lib -lNURBS
INCLUDEPATH += ../NURBS

# VOXEL library
LIBS += -L$$PWD/../DynamicVoxel/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

# TopoBlender library
LIBS += -L$$PWD/../TopoBlenderLib/lib -lTopoBlenderLib
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
