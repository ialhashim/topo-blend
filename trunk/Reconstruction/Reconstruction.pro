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

HEADERS += \
    Src/Vector.h \
    Src/Time.h \
    Src/SparseMatrix.h \
    Src/PPolynomial.h \
    Src/Polynomial.h \
    Src/PointStream.h \
    Src/PlyFile.h \
    Src/ply.h \
    Src/Octree.h \
    Src/MultiGridOctreeData.h \
    Src/MemoryUsage.h \
    Src/MAT.h \
    Src/MarchingCubes.h \
    Src/Hash.h \
    Src/Geometry.h \
    Src/FunctionData.h \
    Src/Factor.h \
    Src/CmdLineParser.h \
    Src/BSplineData.h \
    Src/BinaryNode.h \
    Src/Allocator.h \
    Src/MultiGridOctest.h \
    poissonrecon.h

SOURCES += \
    Src/Vector.inl \
    Src/Time.cpp \
    Src/SparseMatrix.inl \
    Src/PPolynomial.inl \
    Src/Polynomial.inl \
    Src/PointStream.inl \
    Src/plyfile.cpp \
    Src/ply.cpp \
    Src/Octree.inl \
    Src/MultiGridOctreeData.inl \
    Src/MultiGridOctest.cpp \
    Src/MAT.inl \
    Src/MarchingCubes.cpp \
    Src/Geometry.inl \
    Src/Geometry.cpp \
    Src/FunctionData.inl \
    Src/Factor.cpp \
    Src/CmdLineParser.inl \
    Src/CmdLineParser.cpp \
    Src/BSplineData.inl \
    poissonrecon.cpp

unix:!mac:QMAKE_CXXFLAGS = $$QMAKE_CFLAGS -fpermissive
mac:QMAKE_LFLAGS += -fopenmp
