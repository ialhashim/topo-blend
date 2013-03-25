#pragma once

#define PREV(i, N) ((i + N-1) % N)
#define NEXT(i, N) ((i + 1) % N)

#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616

static inline Scalar deg_to_rad(const Scalar& _angle)
{ return M_PI*(_angle/180.0); }

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

double inline gaussianFunction(double x, double mu = 0.0, double sigma = 1.0){
	//double a = 1.0 / (sigma * sqrt(2 * M_PI));
	double b = mu;
	double c = sigma;

	// normalized Gaussian with N(0, \sigma) = 1
	return exp( - (pow(x - b, 2) / (2 * pow(c, 2)) ) );
}

static inline std::vector<Vec3d> equidistLine(std::vector<Vec3d> & l, int numSegments)
{
	std::vector<Vec3d> resampled_line;

	// check 
	numSegments = qMax(numSegments, 2);

	// Parameterize line by finding segments
	double totalLineLength = 0.0;

	typedef std::pair<double,int> LengthSegment;
	std::vector< LengthSegment > segments;

	// Add a start segment
	segments.push_back(LengthSegment(totalLineLength, 1));

	for(int i = 1; i < (int)l.size(); i++)
	{
		double curLen = (l[i] - l[i-1]).norm();
		totalLineLength += curLen;
		segments.push_back( LengthSegment(totalLineLength, i) );
	}

	if(totalLineLength == 0.0)
		return resampled_line;

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

		resampled_line.push_back( ( (1-alpha) * l[idx-1] ) + (alpha * l[idx]) );
	}

	return resampled_line;
}

static inline bool between(double & val, double minVal, double maxVal) 
{
	return ((val-minVal) >= 0) && ((val-maxVal) <= 0);
}
