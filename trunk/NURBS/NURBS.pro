CONFIG += starlab

STARLAB_DEPENDS *= common
STARLAB_DEPENDS *= ../../starlab/plugins-surfacemesh/surfacemesh


TARGET = NURBS
TEMPLATE = lib
CONFIG += staticlib
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
