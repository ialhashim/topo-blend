#pragma once

#include "Src/MultiGridOctest.h"

#include <QString>
#include <QStringList>

class PoissonRecon
{
public:
    static char** convertArguments(QStringList args);

    static void makeFromCloud(QString filename, QString out_filename, int depth = 7);
};

