load($$[STARLAB])
load($$[SURFACEMESH])
load($$[NANOFLANN])
load($$[OCTREE])
StarlabTemplate(plugin)

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

HEADERS += visiblity_resampler.h
SOURCES += visiblity_resampler.cpp

# OpenMP
win32 {
    QMAKE_CXXFLAGS += /openmp
    export(QMAKE_CXXFLAGS_DEBUG)
    export(QMAKE_CXXFLAGS_RELEASE)
}
unix {
    QMAKE_CXXFLAGS += -fopenmp
    LIBS += -lgomp
}
