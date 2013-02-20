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
TARGET = GlSplatRendererLib
DESTDIR = $$PWD/$$CFG/lib

SOURCES += GlSplatRenderer.cpp
HEADERS += GlSplatRenderer.h

# GlSplat files
INCLUDEPATH += GlSplat

HEADERS +=  GlSplat/GlSplat.h \
            GlSplat/Shader.h \
            GlSplat/GLee.h

SOURCES +=  GlSplat/GlSplat.cpp \
            GlSplat/Shader.cpp \
            GlSplat/GLee.c

RESOURCES += \
    shaders.qrc
