load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
load($$[NANOFLANN])
load($$[OCTREE])
StarlabTemplate(none)

TEMPLATE = lib
CONFIG += staticlib
QT += opengl xml

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# Library name and destination
TARGET = TopoBlenderLib
DESTDIR = $$PWD/$$CFG/lib

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


HEADERS += StructureNode.h \
    StructureGraph.h \
    StructureCurve.h \
    StructureSheet.h \
    StructureLink.h \
    GraphEmbed.h \
    TopoBlender.h \
    DynamicGraph.h \
    DynamicGraphGlobal.h \
    GraphDistance.h \
    ExportDynamicGraph.h \
    GraphCorresponder.h \
    Scheduler.h \
    Task.h \
    SchedulerWidget.h \
    FFD.h \
    ARAPCurveDeformer.h \
    ARAPCurveHandle.h \
    TimelineSlider.h \
    StructureGlobal.h \
    Synthesizer.h \
    Sampler.h \
    SpherePackSampling.h \
    AbsoluteOrientation.h \
    TaskCurve.h \
    TaskSheet.h \
    Relink.h

SOURCES += StructureGraph.cpp \
    StructureCurve.cpp \
    StructureSheet.cpp \
    StructureLink.cpp \
    TopoBlender.cpp \
    DynamicGraph.cpp \
    GraphDistance.cpp \
    GraphCorresponder.cpp \
    Scheduler.cpp \
    Task.cpp \
    SchedulerWidget.cpp \
    FFD.cpp \
    ARAPCurveDeformer.cpp \
    TimelineSlider.cpp \
    Synthesizer.cpp \
    Sampler.cpp \
    AbsoluteOrientation.cpp \
    TaskCurve.cpp \
    TaskSheet.cpp \
    Relink.cpp
	
FORMS +=  SchedulerWidget.ui
