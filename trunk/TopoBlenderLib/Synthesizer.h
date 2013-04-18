#pragma once

#include "StructureGraph.h"
using namespace Structure;

#include "RMF.h"

extern int randomCount;
extern int uniformTriCount;

struct ParameterCoord{
	double u, v;
	double theta, psi;

	double origOffset;
	Structure::Node * origNode;
	Vector3 origNormal;

	ParameterCoord(){ u = v = -1; theta = psi = 0; origOffset = 0; origNode = NULL; }
	ParameterCoord(double theta, double psi, double u, double v = 0, double offset = 0, Structure::Node * node = NULL){
		this->u = u;
		this->v = v;

		this->theta = theta;
		this->psi = psi;

		this->origOffset = offset;
		this->origNode = node;
	}

	bool operator < (const ParameterCoord& other) const{
		return (u < other.u);
	}
};
static inline QDebug operator<<(QDebug dbg, const ParameterCoord &c){
	dbg.nospace() << QString("[ %1, %2] - theta = %3 \tpsi = %4").arg(c.u).arg(c.v).arg(c.theta).arg(c.psi);
	return dbg.space();
}

struct Synthesizer{

	// Generate sample points in the parameter domain
	static QVector<ParameterCoord> genPointCoordsCurve( Structure::Curve * curve, const std::vector<Vec3d> & points, const std::vector<Vec3d> & normals );
	static QVector<ParameterCoord> genPointCoordsSheet( Structure::Sheet * sheet, const std::vector<Vec3d> & points, const std::vector<Vec3d> & normals );

	static QVector<ParameterCoord> genFeatureCoords( Structure::Node * node );
	static QVector<ParameterCoord> genEdgeCoords( Structure::Node * node, double sampling_resolution = -1 );
    static QVector<ParameterCoord> genRandomCoords( Structure::Node * node, int samples_count );
	static QVector<ParameterCoord> genUniformCoords( Structure::Node * node, double sampling_resolution = -1);
	static QVector<ParameterCoord> genRemeshCoords( Structure::Node * node );
	static QVector<ParameterCoord> genUniformTrisCoords( Structure::Node * node );

    // Compute the geometry on given samples in the parameter domain
	static void sampleGeometryCurve( QVector<ParameterCoord> samples, Structure::Curve * curve, QVector<double> &offsets, QVector<Vec2d> &normals);
	static void sampleGeometrySheet( QVector<ParameterCoord> samples, Structure::Sheet * sheet, QVector<double> &offsets, QVector<Vec2d> &normals );

	// Blend skeleton bases
	static void blendCurveBases(Structure::Curve * curve1, Structure::Curve * curve2, double alpha);
	static void blendSheetBases(Structure::Sheet * sheet1, Structure::Sheet * sheet2, double alpha);

	// Reconstruction on given base skeleton
	static void reconstructGeometryCurve( Structure::Curve * base_curve, QVector<ParameterCoord> in_samples, QVector<double> &in_offsets, 
											QVector<Vec2d> &in_normals, QVector<Vector3> &out_points, QVector<Vector3> &out_normals);
	static void reconstructGeometrySheet( Structure::Sheet * base_sheet, QVector<ParameterCoord> in_samples, QVector<double> &in_offsets, 
											QVector<Vec2d> &in_normals, QVector<Vector3> &out_points, QVector<Vector3> &out_normals);
	// Preparation
	enum SamplingType{ Features = 1, Edges = 2, Random = 4, Uniform = 8, All = 16, AllNonUniform = 32, Remeshing = 64, TriUniform = 128 };

	static void prepareSynthesizeCurve( Structure::Curve * curve1, Structure::Curve * curve2, int samplingType = Features | Random );
	static void prepareSynthesizeSheet( Structure::Sheet * sheet1, Structure::Sheet * sheet2, int samplingType = Features | Random );
	
	// Blend geometries
	static void blendGeometryCurves( Structure::Curve * curve1, Structure::Curve * curve2, double alpha, QVector<Vector3> &points, QVector<Vector3> &normals);
	static void blendGeometrySheets( Structure::Sheet * sheet1, Structure::Sheet * sheet2, double alpha, QVector<Vector3> &points, QVector<Vector3> &normals);

	// Helper functions
	static RMF consistentFrame( Structure::Curve * curve, Array1D_Vec4d & coords );

	// IO
	static void saveSynthesisData(Structure::Node *node, QString prefix = "");
	static void loadSynthesisData(Structure::Node *node, QString prefix = "");
	static void writeXYZ( QString filename, QVector<Vector3> &points, QVector<Vector3> &normals );

	static void copySynthData( Structure::Node * fromNode, Structure::Node * toNode );
	static void clearSynthData( Structure::Node * fromNode );
};

Q_DECLARE_METATYPE(QVector<double>)
Q_DECLARE_METATYPE(QVector<Vec2d>)
Q_DECLARE_METATYPE(QVector<Vector3>)
Q_DECLARE_METATYPE(QVector<ParameterCoord>)
