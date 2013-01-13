load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
load($$[NANOFLANN])

# NURBS library
LIBS += -L$$PWD/../NURBS/lib -lNURBS
INCLUDEPATH += ../NURBS

# VOXEL library
LIBS += -L$$PWD/../DynamicVoxel/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

TEMPLATE = lib
CONFIG += staticlib
QT += opengl xml

# Library name and destination
TARGET = TopoBlenderLib
DESTDIR = $$PWD/lib

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
	Synthesizer2.h \
	Sampler.h \
    SynthesizerOld.h

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
	Synthesizer2.cpp \
	Sampler.cpp
	
	
FORMS += \
    SchedulerWidget.ui

# Morpher related
HEADERS += Morpher.h BoundingBox.h Octree.h
SOURCES += Morpher.cpp Octree.cpp
