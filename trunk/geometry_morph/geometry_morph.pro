load($$[STARLAB])
load($$[SURFACEMESH])
load($$[NANOFLANN])
load($$[CHOLMOD])
load($$[EIGEN])
StarlabTemplate(plugin)

# NURBS library
LIBS += -L$$PWD/../NURBS/lib -lNURBS
INCLUDEPATH += ../NURBS

# VOXEL library
LIBS += -L$$PWD/../DynamicVoxel/lib -lDynamicVoxel
INCLUDEPATH += ../DynamicVoxel

# TopoBlender library
LIBS += -L$$PWD/../TopoBlenderLib/lib -lTopoBlenderLib
INCLUDEPATH += ../TopoBlenderLib

SOURCES +=	geometry_morph.cpp \
			geometry_morph_widget.cpp \
			Morpher.cpp

HEADERS  += geometry_morph.h \
			geometry_morph_widget.h \
			Morpher.h

FORMS    += geometry_morph_widget.ui
