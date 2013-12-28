#pragma once
#include <vector>

#include <QString>
#include <QStringList>

#ifdef WIN32
namespace std{  static inline bool isnan(double x){ return _isnan(x); } }
#else
#include <cmath>
#endif


struct SimpleMesh{
	std::vector< std::vector<float> > vertices;
	std::vector< std::vector<int> > faces;
};

class PoissonRecon
{
public:
    static char** convertArguments(QStringList args);

    //static void makeFromCloudFile(QString filename, QString out_filename, int depth = 7);
	static void makeFileFromCloud( std::vector< std::vector<float> > p, std::vector< std::vector<float> > n, QString out_filename, int depth = 7);
	static void makeFromCloud(std::vector< std::vector<float> > p, std::vector< std::vector<float> > n, SimpleMesh & mesh, int depth = 7);

	static void writeOBJ(QString out_filename, std::vector< std::vector<float> > & mesh_verts, std::vector< std::vector<int> > & mesh_faces);
};

// Helper functions
template<typename Vector3>
static inline std::vector< std::vector<float> > pointCloudf( std::vector<Vector3> points ){
	std::vector< std::vector<float> > cloud(points.size(), std::vector<float>(3,0));
	for(int i = 0; i < (int)points.size(); i++)
	{
        if( std::isnan(cloud[i][0]) || std::isnan(cloud[i][1]) || std::isnan(cloud[i][2]) ) continue;

		cloud[i][0] = points[i][0];
		cloud[i][1] = points[i][1];
		cloud[i][2] = points[i][2];
	}
	return cloud;
}
