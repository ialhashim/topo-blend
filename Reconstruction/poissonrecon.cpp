#include "poissonrecon.h"

#include <QFileInfo>
#include <QDir>
#include <QTextStream>

#include "Src/PoissonRecon.cpp"

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

void PoissonRecon::makeFromCloud( std::vector< std::vector<float> > p, std::vector< std::vector<float> > n, SimpleMesh & mesh, int depth /*= 7*/ )
{
	QStringList args;

	args << "program_name";
	args << "--in" << "dummy" << "--out" << "output";
	args << "--depth" << QString::number(depth);

	std::vector< std::vector<float> > mesh_verts;
	std::vector< std::vector<int> > mesh_faces;

    ExecuteMemory< 2 >(args.size(), convertArguments(args), p, n, mesh.vertices, mesh.faces);

	// DEBUG: output point cloud
	if( mesh.faces.size() == 0 )
	{
		QFile file( "cloud.xyz" );
		QFileInfo fileInfo(file.fileName());
		QDir d(""); d.mkpath(fileInfo.absolutePath());
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
		QTextStream out(&file);

		for(int i = 0; i < (int)p.size(); i++){
			for(int v = 0; v < 3; v++)	out << p[i][v] << " ";
			for(int v = 0; v < 3; v++)	out << n[i][v] << " ";
			out << "\n";
		}

		file.close();
	}
}

void PoissonRecon::makeFileFromCloud( std::vector< std::vector<float> > p, std::vector< std::vector<float> > n, QString out_filename, int depth /*= 7*/ )
{
	QStringList args;

	args << "program_name";
	args << "--in" << "dummy" << "--out" << "output";
	args << "--depth" << QString::number(depth);

	std::vector< std::vector<float> > mesh_verts;
	std::vector< std::vector<int> > mesh_faces;

    ExecuteMemory< 2 >(args.size(), convertArguments(args), p, n, mesh_verts, mesh_faces);

	// Write OBJ
	writeOBJ(out_filename, mesh_verts, mesh_faces);
}

void PoissonRecon::writeOBJ(QString out_filename, std::vector< std::vector<float> > & mesh_verts, std::vector< std::vector<int> > & mesh_faces)
{
	QFile file(out_filename);

	// Create folder
	QFileInfo fileInfo(file.fileName());
	QDir d(""); d.mkpath(fileInfo.absolutePath());

	// Open for writing
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;

	QTextStream out(&file);
	out << "# NV = " << mesh_verts.size() << " NF = " << mesh_faces.size() << "\n";

	// Vertices
	foreach( std::vector<float> v, mesh_verts )
	{
		out << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
	}

	// Faces
	foreach( std::vector<int> f, mesh_faces )
	{
		out << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << "\n";
	}
	file.close();
}
