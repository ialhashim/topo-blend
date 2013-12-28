include($$[STARLAB])
StarlabTemplate(none)

TEMPLATE = lib
CONFIG += staticlib

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# Library name and destination
TARGET = Reconstruction
DESTDIR = $$PWD/$$CFG/lib

HEADERS += poissonrecon.h

SOURCES += poissonrecon.cpp

unix:!mac:QMAKE_CXXFLAGS = $$QMAKE_CFLAGS -fpermissive
mac:QMAKE_LFLAGS += -fopenmp
