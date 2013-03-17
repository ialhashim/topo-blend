load($$[STARLAB])
load($$[SURFACEMESH])

TEMPLATE = lib
CONFIG += staticlib
QT += opengl

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# Library name and destination
TARGET = NURBS
DESTDIR = $$PWD/$$CFG/lib

SOURCES += \
    ParametricSurface.cpp \
    NURBSRectangle.cpp \
    NURBSCurve.cpp \
    BSplineRectangle.cpp \
    BSplineCurve.cpp \
    SingleCurve.cpp \
    Surface.cpp \
    Curve.cpp \
    BSplineBasis.cpp \
    LineSegment.cpp \
    NurbsDraw.cpp

HEADERS += \
    ParametricSurface.h \
    NURBSRectangle.h \
    NURBSCurve.h \
    BSplineRectangle.h \
    BSplineCurve.h \
    SingleCurve.h \
    Surface.h \
    NURBSGlobal.h \
    Curve.h \
    BSplineBasis.h \
    Integrate1.h \
    LineSegment.h \
    NurbsDraw.h
	
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
