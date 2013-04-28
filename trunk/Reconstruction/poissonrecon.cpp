#include "poissonrecon.h"

// This is needed for template issue in non-windows
#include "Src/MultiGridOctest.cpp"

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

void PoissonRecon::makeFromCloudFile(QString filename, QString out_filename, int depth)
{
    QStringList args;

    args << "program_name";
    args << "--in" << filename << "--out" << out_filename;
    args << "--depth" << QString::number(depth);

    Execute< 2 >(args.size(), convertArguments(args) );
}

void PoissonRecon::makeFromCloud( std::vector< std::vector<float> > p, std::vector< std::vector<float> > n, QString out_filename, int depth /*= 7*/ )
{
	QStringList args;

	args << "program_name";
	args << "--in" << "dummy" << "--out" << out_filename;
	args << "--depth" << QString::number(depth);

	ExecuteMemory< 2 >(args.size(), convertArguments(args), p, n );
}
