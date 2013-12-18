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

QT += core gui opengl network

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
            BlendPathSubButton.cpp \
            BlendRenderItem.cpp \
            ShapeRenderer.cpp \
            BlendPathWidget.cpp \
            PathEvaluator.cpp \
            json.cpp \
            ExporterWidget.cpp

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
            BlendPathSubButton.h \
            BlendRenderItem.h \
            ShapeRenderer.h \
            BlendPathWidget.h \
            PathEvaluator.h \
            json.h \
            ExporterWidget.h \
            HttpUploader.h

FORMS    += mainwindow.ui \
            Controls.ui \
            ExporterWidget.ui

# Icons and images
RESOURCES += resources.qrc
win32:RC_FILE = demo.rc
mac:ICON = images/appIcon.icns

# Mac specific
mac:LIBS += -framework CoreFoundation # We need this for GLee..
mac:QMAKE_LFLAGS += -fopenmp

unix:!mac:QMAKE_CXXFLAGS = $$QMAKE_CFLAGS -fpermissive
unix:!mac:LIBS += -lGLU

# This is some weird linker issue..
unix:!mac:LIBS += $$PWD/../NURBS/$$CFG/lib/libNURBS.a
unix:!mac:LIBS += $$PWD/../Reconstruction/$$CFG/lib/libReconstruction.a
