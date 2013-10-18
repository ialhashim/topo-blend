include($$[STARLAB])
include($$[SURFACEMESH])
StarlabTemplate(none)

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# NURBS library
LIBS += -L$$PWD/../NURBS/$$CFG/lib -lNURBS
INCLUDEPATH += ../NURBS

# TopoBlender library
LIBS += -L$$PWD/../TopoBlenderLib/$$CFG/lib -lTopoBlenderLib
INCLUDEPATH += ../TopoBlenderLib

TEMPLATE = lib
CONFIG += staticlib
QT += opengl xml

# Library name and destination
TARGET = RelationDetectorLib
DESTDIR = $$PWD/$$CFG/lib

SOURCES +=  RelationDetector.cpp  \
            transform3d.cpp \
            RelationManager.cpp \
            RelationWidget.cpp

HEADERS +=  RelationDetector.h \
            transform3d.h \
            RelationManager.h \
            RelationWidget.h \
            Geometry.h

FORMS +=    RelationWidget.ui
