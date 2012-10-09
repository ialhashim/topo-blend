CONFIG += starlab
STARLAB_TEMPLATE += plugin
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh
STARLAB_EXTERNAL += nanoflann eigen cholmod

# NURBS library
LIBS += -L$$PWD/../NURBS/lib -lNURBS
INCLUDEPATH += ../NURBS

HEADERS += topo-blend.h \ 
    topo_blend_widget.h \
    topo/StructureNode.h \
    topo/StructureGraph.h \
    topo/StructureCurve.h \
    topo/StructureSheet.h \
    topo/StructureLink.h
SOURCES += topo-blend.cpp \ 
    topo_blend_widget.cpp \
    topo/StructureGraph.cpp \
    topo/StructureCurve.cpp \
    topo/StructureSheet.cpp
RESOURCES += topo-blend.qrc
FORMS += \ 
    topo_blend_widget.ui
