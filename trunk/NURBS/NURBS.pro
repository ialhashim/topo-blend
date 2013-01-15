load($$[STARLAB])
load($$[SURFACEMESH])

TEMPLATE = lib
CONFIG += staticlib

# Library name and destination
TARGET = NURBS
DESTDIR = $$PWD/lib

SOURCES += \
    ParametricSurface.cpp \
    NURBSRectangle.cpp \
    Matrix2.inl \
    Matrix2.cpp \
    Integrate1.cpp \
    BSplineBasis.cpp \
    NURBSCurve.cpp \
    SingleCurve.cpp \
    Curve.cpp \
    LineSegment.cpp

HEADERS += \
    ParametricSurface.h \
    NURBSRectangle.h \
    NurbsDraw.h \
    Matrix2.h \
    Integrate1.h \
    BSplineBasis.h \
    NURBSCurve.h \
    Curve.h \
    SingleCurve.h \
    LineSegment.h \
    NURBSGlobal.h
	
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