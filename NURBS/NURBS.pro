CONFIG += starlab staticlib
STARLAB_TEMPLATE += plugin
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh

TEMPLATE = lib
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
    Curve.cpp

HEADERS += \
    ParametricSurface.h \
    NURBSRectangle.h \
    NurbsDraw.h \
    Matrix2.h \
    Integrate1.h \
    BSplineBasis.h \
    NURBSCurve.h \
    Curve.h \
    SingleCurve.h
