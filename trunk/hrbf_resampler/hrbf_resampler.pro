include($$[STARLAB])
include($$[SURFACEMESH])
include($$[NANOFLANN])
StarlabTemplate(plugin)

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# Plugin code
HEADERS += hrbf_resampler.h
SOURCES += hrbf_resampler.cpp

# "Implicit surface reconstruction with HRBF" from Rodolphe Vaillant
# http://www.irit.fr/~Rodolphe.Vaillant/?e=12
HEADERS +=  hrbf/hrbf_core.h \
            hrbf/hrbf_phi_funcs.h

# Standard Marching Cubes
# http://graphics.stanford.edu/~mdfisher/MarchingCubes.html
HEADERS +=  mc/MarchingCubes.h

