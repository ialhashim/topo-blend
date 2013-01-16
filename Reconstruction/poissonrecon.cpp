#include "poissonrecon.h"

char **PoissonRecon::convertArguments(QStringList args)
{
    char ** argsv = new char*[args.size()];

    for(int i = 0; i < (int)args.size(); i++)
    {
        argsv[i] = new char[args[i].size() + 1];

        for(int j = 0; j < args[i].size(); j++)
            argsv[i][j] = args[i].at(j).toAscii();

		argsv[i][args[i].size()] = '\0';
    }

    return argsv;
}

void PoissonRecon::makeFromCloud(QString filename, QString out_filename, int depth)
{
    QStringList args;

    args << "program_name";
    args << "--in" << filename << "--out" << out_filename;
    args << "--depth" << QString::number(depth);

    Execute< 2 >(args.size(), convertArguments(args) );
}
