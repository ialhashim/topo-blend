include($$[STARLAB])
include($$[SURFACEMESH])
include($$[CHOLMOD])
include($$[NANOFLANN])
StarlabTemplate(plugin)

# UI related
include(QtAwesome/QtAwesome.pri)

QT += opengl xml svg

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# NURBS library
LIBS += -L$$PWD/../NURBS/$$CFG/lib -lNURBS
INCLUDEPATH += ../NURBS

# Surface Reconstruction library
LIBS += -L$$PWD/../Reconstruction/$$CFG/lib -lReconstruction
INCLUDEPATH += ../Reconstruction

# TopoBlender library
LIBS += -L$$PWD/../TopoBlenderLib/$$CFG/lib -lTopoBlenderLib
INCLUDEPATH += ../TopoBlenderLib

# Splat Rendering library
LIBS += -L$$PWD/../GlSplatRendererLib/$$CFG/lib -lGlSplatRendererLib
INCLUDEPATH += ../GlSplatRendererLib

# Blend path scoring library
LIBS += -L$$PWD/../ScorerLib/$$CFG/lib -lScorerLib
INCLUDEPATH += ../ScorerLib

HEADERS +=  topo-blend.h \
            QuickMesh.h \
            QuickViewer.h \
            topo_blend_widget.h \
            landmarks_dialog.h \
            QuickAlignment.h \
            QuickGroup.h \
            graphs-manager.h \
            correspondence-manager.h \
            wizard.h

SOURCES +=  topo-blend.cpp \
            topo_blend_widget.cpp \
            landmarks_dialog.cpp \
            QuickAlignment.cpp \
            QuickGroup.cpp \
            graphs-manager.cpp \
            correspondence-manager.cpp \
            wizard.cpp
	
RESOURCES += topo-blend.qrc

FORMS +=    topo_blend_widget.ui \
            animationWidget.ui \
            landmarks_dialog.ui \
            QuickAlignment.ui \
            QuickGroup.ui \
            wizard.ui

mac:LIBS += -framework CoreFoundation # We need this for GLee..
mac:QMAKE_LFLAGS += -fopenmp
