load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
load($$[NANOFLANN])
StarlabTemplate(plugin)

# NURBS library
LIBS += -L$$PWD/../NURBS/lib -lNURBS
INCLUDEPATH += ../NURBS

# NURBS library
LIBS += -L$$PWD/../DynamicVoxel/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

HEADERS += topo-blend.h \ 
    topo_blend_widget.h \
    topo/StructureNode.h \
    topo/StructureGraph.h \
    topo/StructureCurve.h \
    topo/StructureSheet.h \
    topo/StructureLink.h \
    topo/GraphEmbed.h \
    topo/TopoBlender.h \
    topo/DynamicGraph.h \
    topo/DynamicGraphGlobal.h \
    topo/GraphDistance.h \
    topo/ExportDynamicGraph.h \
    QuickMesh.h \
    QuickViewer.h \
    topo/GraphCorresponder.h

SOURCES += topo-blend.cpp \ 
    topo_blend_widget.cpp \
    topo/StructureGraph.cpp \
    topo/StructureCurve.cpp \
    topo/StructureSheet.cpp \
    topo/StructureLink.cpp \
    topo/TopoBlender.cpp \
    topo/DynamicGraph.cpp \
    topo/GraphDistance.cpp \
    topo/GraphCorresponder.cpp
	
RESOURCES += topo-blend.qrc

FORMS += \ 
    topo_blend_widget.ui \
    animationWidget.ui
