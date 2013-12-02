include($$[STARLAB])
include($$[SURFACEMESH])
include($$[CHOLMOD])

include($$[NANOFLANN])
include($$[OCTREE])
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

# Surface Reconstruction library
LIBS += -L$$PWD/../Reconstruction/$$CFG/lib -lReconstruction
INCLUDEPATH += ../Reconstruction

# Splat Rendering library
LIBS += -L$$PWD/../GlSplatRendererLib/$$CFG/lib -lGlSplatRendererLib
INCLUDEPATH += ../GlSplatRendererLib

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
    TimelineSlider.h \
    StructureGlobal.h \
    Synthesizer.h \
    SynthesisManager.h \
    Sampler.h \
    SimilarSampling.h \
    SpherePackSampling.h \
    AbsoluteOrientation.h \
    TaskCurve.h \
    TaskSheet.h \
    Relink.h \
    GraphModifyWidget.h \
    GraphDissimilarity.h

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
    TimelineSlider.cpp \
    Synthesizer.cpp \
    SynthesisManager.cpp \
    Sampler.cpp \
    SimilarSampling.cpp \
    AbsoluteOrientation.cpp \
    TaskCurve.cpp \
    TaskSheet.cpp \
    Relink.cpp \
    GraphModifyWidget.cpp \
    GraphDissimilarity.cpp
	
FORMS +=  SchedulerWidget.ui GraphModifyWidget.ui

mac:QMAKE_CXXFLAGS += -fopenmp
mac:QMAKE_LFLAGS += -fopenmp

unix:!mac:QMAKE_CXXFLAGS = $$QMAKE_CFLAGS -fpermissive
unix:!mac:LIBS += -lGLU
