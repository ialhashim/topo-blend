CONFIG += starlab
STARLAB_TEMPLATE += plugin
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh
STARLAB_EXTERNAL += nanoflann eigen cholmod

HEADERS += topo-blend.h \ 
    topo_blend_widget.h
SOURCES += topo-blend.cpp \ 
    topo_blend_widget.cpp
RESOURCES += topo-blend.qrc
FORMS += \ 
    topo_blend_widget.ui
