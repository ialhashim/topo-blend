#pragma once

#include <assert.h>
#include <QVariant>
#include <QMap>
#include <QSet>
#include <QFileInfo>
#include <QDir>

#include "SurfaceMeshHelper.h"
#include "NurbsDraw.h"
#include "../CustomDrawObjects.h"

#undef max

static inline std::vector<Vector3> noFrame(){
	return std::vector<Vector3>();
}

static inline Vector3 orthogonalVector(const Vector3& n) {
	if ((abs(n.y()) >= 0.9 * abs(n.x())) &&
		abs(n.z()) >= 0.9 * abs(n.x())) return Vector3(0.0, -n.z(), n.y());
	else if ( abs(n.x()) >= 0.9 * abs(n.y()) &&
		abs(n.z()) >= 0.9 * abs(n.y()) ) return Vector3(-n.z(), 0.0, n.x());
	else return Vector3(-n.y(), n.x(), 0.0);
}

#define qRanged(min, v, max) ( qMax(min, qMin(v, max)) )

// Coordinates utility functions
static inline Vec4d coord(Scalar u = 0, Scalar v = 0)	{ return Vec4d(u, v, 0, 0); }
static inline Vec4d inverseCoord(const Vec4d& c)		{ return Vec4d(1 - c.x(), 1 - c.y(), 0,0); }
static inline std::vector<Vec4d> inverseCoords(const std::vector<Vec4d>& fromCoords)	{ 
	std::vector<Vec4d> invertedCoords;
	foreach(Vec4d coord, fromCoords) invertedCoords.push_back( inverseCoord(coord) );
	return invertedCoords; 
}

static inline double signedAngle(const Vector3 &a, const Vector3 &b, const Vector3 &axis)
{
	if(axis.norm() == 0.0) qDebug() << "warning: zero axis";
	double cosAngle = dot(a.normalized(), b.normalized());
	double angle = acos( qRanged(-1.0, cosAngle, 1.0) );
	Vector3 c = cross(a, b);
	if (dot(c, axis) < 0) return -angle;
	return angle;
}

static inline Vec3d rotatedVec(const Vec3d & v, double theta, const Vec3d & axis)
{
	if(theta == 0.0) return v;
	return (v * cos(theta) + cross(axis, v) * sin(theta) + axis * dot(axis, v) * (1 - cos(theta)));
}

/// Spherical Coordinates:
/* From Spherical-coordinates to Vector 'v' */
static inline void localSphericalToGlobal( Vector3 X, Vector3 Y, Vector3 Z, double theta, double psi, Vector3 &v )
{
	Q_UNUSED(X);
	v = rotatedVec(Z, theta, Y);
	v = rotatedVec(v, psi, Z);
}

/* Encode vector 'v' to theta-psi in spherical coordinates of XYZ  */
static inline void globalToLocalSpherical( Vector3 X, Vector3 Y, Vector3 Z, double &theta, double &psi, Vector3 v )
{
	// Theta: angle from Z [0, PI]
	// Psi: angle from X on XY plane [0, 2*PI)
	double dotZ = dot(v, Z);
	theta = acos( qRanged(-1.0, dotZ, 1.0) );

	double dotX = dot(v, X);
	double dotY = dot(v, Y);
	Vec3d proj_v = (dotX * X + dotY * Y).normalized();
	psi = signedAngle(X, proj_v, Z);
}

#define	POINT_ID_RANGE 1000
#define NODE_ID_RANGE	100

static void getIndicesFromSelectedName(int selectedName, int &gID, int &nID, int &pID)
{
	pID = selectedName % POINT_ID_RANGE;
	selectedName /= POINT_ID_RANGE;

	nID = selectedName % NODE_ID_RANGE;
	gID = selectedName / NODE_ID_RANGE;
}

typedef std::pair< QVector<QString>, QVector<QString> > PART_LANDMARK;
typedef std::pair< int, int > POINT_ID;
typedef std::pair< QVector<POINT_ID>, QVector<POINT_ID> > POINT_LANDMARK;

#define AlphaBlend(alpha, start, end) ( ((1-alpha) * start) + (alpha * end) )

static std::vector<Vec3d> refineByNumber(const std::vector<Vec3d> & fromPnts, int targetNumber)
{
	// Refine until targetNumber is achieved
	std::vector<Vec3d> newPnts = fromPnts;
	while(targetNumber > (int)newPnts.size())
	{
		// Find index of largest edge and split
		double maxDist = -DBL_MAX;
		int idx = -1;
		for(int i = 1; i < (int) newPnts.size(); i++){
			double dist = (newPnts[i] - newPnts[i-1]).norm();
			if(dist > maxDist){
				maxDist = dist;
				idx = i;
			}
		}

		Vec3d midPoint = (newPnts[idx] + newPnts[idx-1]) / 2.0;

		// Insert new point
		newPnts.insert( newPnts.begin() + (idx), midPoint );
	}

	return newPnts;
}

static std::vector<Vec3d> refineByResolution(const std::vector<Vec3d> & fromPnts, double resolution)
{
	std::vector<Vec3d> resampled_polyline;

	// Parameterize line by finding segments
	double totalLineLength = 0.0;

	typedef std::pair<double,int> LengthSegment;
	std::vector< LengthSegment > segments;

	// Add a start segment
	segments.push_back(LengthSegment(totalLineLength, 1));

	for(int i = 1; i < (int)fromPnts.size(); i++)
	{
		double curLen = (fromPnts[i] - fromPnts[i-1]).norm();
		totalLineLength += curLen;
		segments.push_back( LengthSegment(totalLineLength, i) );
	}

	if(totalLineLength == 0.0)
		return resampled_polyline;

	double numSegments = totalLineLength / resolution;

	// Re-sample line
	for(int i = 0; i < numSegments; i++)
	{
		double t = totalLineLength * (double(i) / (numSegments-1));

		std::vector< LengthSegment >::iterator it = lower_bound(segments.begin(), 
			segments.end(), LengthSegment(qMin(t, totalLineLength), -1));

		// Find start
		int idx = it->second;

		double seg_range = segments[idx].first - segments[idx-1].first;
		double seg_start = segments[idx-1].first;

		double alpha = (t - seg_start) / seg_range;

		resampled_polyline.push_back( ( (1-alpha) * fromPnts[idx-1] ) + (alpha * fromPnts[idx]) );
	}

	return resampled_polyline;
}

static std::vector<Vec3d> smoothPolyline(const std::vector<Vec3d> & fromPnts, int num_iterations)
{
	Array1D_Vector3 pnts = fromPnts;

	// Laplacian smoothing - fixed ends
	for(int itr = 0; itr < num_iterations; itr++)
	{
		Array1D_Vector3 newPos(pnts.size(), Vector3(0));
		newPos[0] = pnts[0];

		for(int i = 1; i < (int)pnts.size() - 1; i++)
			newPos[i] = (pnts[i-1] + pnts[i+1]) * 0.5;

		newPos.back() = pnts.back();
		pnts = newPos;
	}

	return pnts;
}

/* Generic vector operations */
template<typename T>
static inline std::vector<T> concat(const std::vector<T> & A, const std::vector<T> & B){
	std::vector<T> dest;
	dest.insert(dest.end(), A.begin(), A.end());
	dest.insert(dest.end(), B.begin(), B.end());
	return dest;
}
template<typename T>
static inline std::vector<T> concat(const std::vector<T> & A, const std::vector<T> & B, const std::vector<T> & C){
	return concat(concat(A,B), C);
}

template<typename T>
static inline std::vector<T> subvec(const std::vector<T> & V, int start, int length = -1){
	if(length < 0) length = V.size() - start;
    typename std::vector<T>::const_iterator first = V.begin() + start;
    typename std::vector<T>::const_iterator last = V.begin() + qMin(start + length, (int)V.size());
	return std::vector<T> (first, last);
}

template<typename T>
static inline std::vector<T> leftvec(const std::vector<T> & V, int length){
	return subvec(V, 0, qMin((int)V.size(), length));
}
template<typename T>
static inline std::vector<T> rightvec(const std::vector<T> & V, int length){
	return subvec(V, qMax(0, (int)V.size() - length));
}
template<typename T>
static inline std::vector<T> chopvec(const std::vector<T> & V, int length){
	return subvec(V, 0, V.size() - length);
}

template<typename T>
static inline std::vector<T> reversedvec(const std::vector<T> & V){
	std::vector<T> reversed = V;
	std::reverse(reversed.begin(), reversed.end());
	return reversed;
}

static void saveOBJ(SurfaceMesh::Model * mesh, QString filename)
{
	QFile file(filename);
	QFileInfo fileInfo(file.fileName());
	QDir d; d.mkpath(fileInfo.absolutePath());

	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	
	QTextStream out(&file);
	out << "# NV = " << mesh->n_vertices() << " NF = " << mesh->n_faces() << "\n";
	Vector3VertexProperty points = mesh->vertex_property<Vec3d>("v:point");
	foreach( Vertex v, mesh->vertices() )
		out << "v " << points[v][0] << " " << points[v][1] << " " << points[v][2] << "\n";
	foreach( Face f, mesh->faces() ){
		out << "f ";
		Surface_mesh::Vertex_around_face_circulator fvit=mesh->vertices(f), fvend=fvit;
		do{	out << (((Surface_mesh::Vertex)fvit).idx()+1) << " ";} while (++fvit != fvend);
		out << "\n";
	}
	file.close();
}
