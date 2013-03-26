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

static inline Vec3d barycentric(Vec3d p, Vec3d a, Vec3d b, Vec3d c)
{
	Vec3d v0 = b - a, v1 = c - a, v2 = p - a;
	double d00 = dot(v0, v0);
	double d01 = dot(v0, v1);
	double d11 = dot(v1, v1);
	double d20 = dot(v2, v0);
	double d21 = dot(v2, v1);
	double denom = d00 * d11 - d01 * d01;
	double v = (d11 * d20 - d01 * d21) / denom;
	double w = (d00 * d21 - d01 * d20) / denom;
	double u = 1.0 - v - w;
	return Vec3d(u,v,w);
}

static inline Vec3d get_barycentric(Vec3d coord, Vec3d a, Vec3d b, Vec3d c)
{
	Q_UNUSED(c);
	return (coord[0] * a) + ((1.0 - coord[0]) * b);
}

#define BARY_THRESHOLD 1e-4
static inline bool outside(Vec3d coord, double threshold = BARY_THRESHOLD)
{
	return coord[0] < -threshold || coord[1] < -threshold || coord[2] < -threshold;
}

static inline bool atBorder(Vec3d coord, double threshold = BARY_THRESHOLD)
{
	return abs(coord[0]) < threshold || abs(coord[1]) < threshold || abs(coord[2]) < threshold;
}

static inline Vec3d pullInside(Vec3d coord, double threshold = BARY_THRESHOLD)
{
	Vec3d result = coord;
	if(coord[0] < threshold) result[0] = threshold;
	if(coord[1] < threshold) result[1] = threshold;
	if(coord[2] < threshold) result[2] = threshold;
	return result;
}

// Sort by QMap second value
template<class F, class S>
static inline bool sortByFirst(const QPair<F,S>& e1, const QPair<F,S>& e2) {
	return e1.first < e2.first;
}

template<class F, class S>
static QList< QPair<S, F> > sortQMapByValue(const QMap<F,S> & map)
{
	QList< QPair<S, F> > result;

	// Append items to a list
	QMapIterator<F, S> i(map);
	while (i.hasNext()) {
		i.next();
		result.push_back(qMakePair(i.value(), i.key()));
	}

	// Sort that list
	qSort(result.begin(), result.end(), sortByFirst<S,F>);

	return result;
}
