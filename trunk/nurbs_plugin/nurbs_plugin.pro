CONFIG += starlab 
STARLAB_TEMPLATE += plugin 
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh

HEADERS += nurbs_plugin.h \
    NURBS/NurbsDraw.h \
    NURBS/SingleCurve3.h \
    NURBS/ParametricSurface.h \
    NURBS/NURBSRectangle.h \
    NURBS/NurbsDraw.h \
    NURBS/NURBSCurve3.h \
    NURBS/Matrix2.h \
    NURBS/Integrate1.h \
    NURBS/Curve3.h \
    NURBS/BSplineBasis.h
SOURCES += nurbs_plugin.cpp \
    NURBS/SingleCurve3.cpp \
    NURBS/ParametricSurface.cpp \
    NURBS/NURBSRectangle.cpp \
    NURBS/NURBSCurve3.cpp \
    NURBS/Matrix2.cpp \
    NURBS/Integrate1.cpp \
    NURBS/Curve3.cpp \
    NURBS/BSplineBasis.cpp
RESOURCES += nurbs_plugin.qrc
