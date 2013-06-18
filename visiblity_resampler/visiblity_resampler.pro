include($$[STARLAB])
include($$[SURFACEMESH])
include($$[NANOFLANN])
include($$[OCTREE])
StarlabTemplate(plugin)

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

HEADERS += visiblity_resampler.h
SOURCES += visiblity_resampler.cpp
