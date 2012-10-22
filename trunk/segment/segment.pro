load($$[STARLAB])
load($$[SURFACEMESH])
StarlabTemplate(plugin)

HEADERS += segment.h
SOURCES += segment.cpp

# Skeleton stuff
HEADERS += curveskel/CurveskelModel.h
SOURCES += curveskel/CurveskelModel.cpp

DEFINES *= EXPORTFLAG
INCLUDEPATH += curveskel
