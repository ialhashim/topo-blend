CONFIG += starlab
STARLAB_TEMPLATE += plugin
STARLAB_DEPENDS += ../../starlab/plugins-surfacemesh/surfacemesh
STARLAB_EXTERNAL += nanoflann eigen

QT += opengl

HEADERS += resample.h \
    resamplewidget.h \
    SpherePackSampling.h \
    Sampler.h \
	PCA3.h
SOURCES += resample.cpp \
    resamplewidget.cpp \
    Sampler.cpp
RESOURCES += resample.qrc

FORMS += \
    resamplewidget.ui
