include($$[STARLAB])
include($$[SURFACEMESH])

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
TARGET = ScorerLib
DESTDIR = $$PWD/$$CFG/lib


# NURBS library
LIBS += -L$$PWD/../NURBS/$$CFG/lib -lNURBS
INCLUDEPATH += ../NURBS

# TopoBlender library
LIBS += -L$$PWD/../TopoBlenderLib/$$CFG/lib -lTopoBlenderLib
INCLUDEPATH += ../TopoBlenderLib


HEADERS += \
    transform3d.h \
    ScorerWidget.h \
    ScorerManager.h \
    Scorer.h \
    RelationDetector.h \
    GlobalReflectionSymmScorer.h \
    ConnectivityScorer.h \
    Geometry.h

SOURCES += \
    transform3d.cpp \
    ScorerWidget.cpp \
    ScorerManager.cpp \
    Scorer.cpp \
    RelationDetector.cpp \
    GlobalReflectionSymmScorer.cpp \
    ConnectivityScorer.cpp

FORMS += \
    ScorerWidget.ui
