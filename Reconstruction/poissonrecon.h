#pragma once
#include <vector>

#include "Src/MultiGridOctest.h"

#include <QString>
#include <QStringList>

class PoissonRecon
{
public:
    static char** convertArguments(QStringList args);

    static void makeFromCloudFile(QString filename, QString out_filename, int depth = 7);
	static void makeFromCloud( std::vector< std::vector<float> > & p, std::vector< std::vector<float> > & n, QString out_filename, int depth = 7);
};

