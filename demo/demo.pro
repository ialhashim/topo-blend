include($$[STARLAB])
include($$[SURFACEMESH])
StarlabTemplate(appbundle)

# Build flag for the static libraries
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# NURBS library
LIBS += -L$$PWD/../NURBS/$$CFG/lib -lNURBS
INCLUDEPATH += ../NURBS

# VOXEL library
LIBS += -L$$PWD/../DynamicVoxel/$$CFG/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

# Surface Reconstruction library
LIBS += -L$$PWD/../Reconstruction/$$CFG/lib -lReconstruction
INCLUDEPATH += ../Reconstruction

# TopoBlender library
LIBS += -L$$PWD/../TopoBlenderLib/$$CFG/lib -lTopoBlenderLib
INCLUDEPATH += ../TopoBlenderLib

# Splat Rendering library
LIBS += -L$$PWD/../GLSplatRendererLib/$$CFG/lib -lGLSplatRendererLib
INCLUDEPATH += ../GLSplatRendererLib

QT += core gui opengl

TARGET = demo

SOURCES +=  main.cpp\
            mainwindow.cpp \
            Scene.cpp \
            ShapesGallery.cpp \
            ShapeItem.cpp \
            Session.cpp \
    Controls.cpp \
    GraphItem.cpp \
    Matcher.cpp \
    Blender.cpp \
    ProgressItem.cpp \
    BlendPathRenderer.cpp \
    BlendPathSub.cpp \
    BlenderRenderItem.cpp

HEADERS  += mainwindow.h \
            Scene.h \
            ShapesGallery.h \
            ShapeItem.h \
            Session.h \
            DemoGlobal.h \
    Controls.h \
    GraphItem.h \
    Matcher.h \
    Blender.h \
    DemoPage.h \
    ProgressItem.h \
    SpinnerItem.h \
    BlendPathRenderer.h \
    BlendPathSub.h \
    BlenderRenderItem.h

FORMS    += mainwindow.ui \
    Controls.ui

mac:LIBS += -framework CoreFoundation # We need this for GLee..
mac:QMAKE_LFLAGS += -fopenmp
