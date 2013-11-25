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


HEADERS += ./ConnectivityScorer.h \
    ./Geometry.h \
    ./GlobalReflectionSymmScorer.h \
    ./PairRelationDetector.h \
    ./RelationDetector.h \
	./PairRelationScorer.h \
    ./transform3d.h \
    ./ScorerManager.h \
    ./ScorerWidget.h
SOURCES += ./ConnectivityScorer.cpp \
    ./GlobalReflectionSymmScorer.cpp \
    ./PairRelationDetector.cpp \
    ./RelationDetector.cpp \
	./PairRelationScorer.cpp \
    ./ScorerManager.cpp \
    ./ScorerWidget.cpp \
    ./transform3d.cpp
FORMS += ./ScorerWidget.ui
