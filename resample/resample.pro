load($$[STARLAB])
load($$[SURFACEMESH])
load($$[CHOLMOD])
load($$[EIGEN])
load($$[NANOFLANN])
StarlabTemplate(plugin)

QT += opengl

HEADERS += resample.h \
    resamplewidget.h \
    SpherePackSampling.h \
    Sampler.h
SOURCES += resample.cpp \
    resamplewidget.cpp \
    Sampler.cpp
RESOURCES += resample.qrc

FORMS += \
    resamplewidget.ui
