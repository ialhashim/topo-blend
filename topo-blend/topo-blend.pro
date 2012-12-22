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
    topo_blend_widget.h

SOURCES += topo-blend.cpp \ 
    topo_blend_widget.cpp
	
RESOURCES += topo-blend.qrc

FORMS += topo_blend_widget.ui \
         animationWidget.ui
