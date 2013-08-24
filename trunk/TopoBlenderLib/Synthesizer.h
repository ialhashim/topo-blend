#pragma once

#include "StructureGraph.h"
using namespace Structure;

#include "RMF.h"

extern int randomCount;
extern int uniformTriCount;

struct ParameterCoord{
	float u, v;
	float theta, psi;

	float origOffset;
	Eigen::Vector3f origPoint;
	Eigen::Vector3f origNormal;
	Structure::Node * origNode;

	ParameterCoord(){ u = v = -1; theta = psi = 0; origOffset = 0; origNode = NULL; }
	ParameterCoord(float theta, float psi, float u, float v = 0, float offset = 0, Structure::Node * node = NULL){
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

typedef QMap<QString, QMap<QString, QVariant> > SynthData;

struct Synthesizer{

	// Generate sample points in the parameter domain
	static QVector<ParameterCoord> genPointCoordsCurve( Structure::Curve * curve, const std::vector<Eigen::Vector3f> & points, const std::vector<Eigen::Vector3f> & normals );
	static QVector<ParameterCoord> genPointCoordsSheet( Structure::Sheet * sheet, const std::vector<Eigen::Vector3f> & points, const std::vector<Eigen::Vector3f> & normals );

	static QVector<ParameterCoord> genFeatureCoords( Structure::Node * node );
	static QVector<ParameterCoord> genEdgeCoords( Structure::Node * node );
    static QVector<ParameterCoord> genRandomCoords( Structure::Node * node, int samples_count );
	static QVector<ParameterCoord> genUniformCoords( Structure::Node * node, float sampling_resolution = -1);
	static QVector<ParameterCoord> genRemeshCoords( Structure::Node * node );
	static QVector<ParameterCoord> genUniformTrisCoords( Structure::Node * node );

	static QVector<ParameterCoord> genSampleCoordsCurve(Structure::Curve * curve, int samplingType = Features | Random);
	static QVector<ParameterCoord> genSampleCoordsSheet(Structure::Sheet * sheet, int samplingType = Features | Random);

    // Compute the geometry on given samples in the parameter domain
	static void sampleGeometryCurve( QVector<ParameterCoord> samples, Structure::Curve * curve, QVector<float> &offsets, QVector<Vec2f> &normals);
	static void sampleGeometrySheet( QVector<ParameterCoord> samples, Structure::Sheet * sheet, QVector<float> &offsets, QVector<Vec2f> &normals );

	// Preparation
	enum SamplingType{ Features = 1, Edges = 2, Random = 4, Uniform = 8, All = 16, AllNonUniform = 32, Remeshing = 64, TriUniform = 128 };

	static void prepareSynthesizeCurve( Structure::Curve * curve1, Structure::Curve * curve2, int samplingType, SynthData & output );
	static void prepareSynthesizeSheet( Structure::Sheet * sheet1, Structure::Sheet * sheet2, int samplingType, SynthData & output );
	
	// Blend geometries
	static void blendGeometryCurves( Structure::Curve * curve, float alpha, const SynthData & data, QVector<Eigen::Vector3f> &points, QVector<Eigen::Vector3f> &normals, bool isApprox);
	static void blendGeometrySheets( Structure::Sheet * sheet, float alpha, const SynthData & data, QVector<Eigen::Vector3f> &points, QVector<Eigen::Vector3f> &normals, bool isApprox);

	// Reconstruction on given base skeleton
	static void reconstructGeometryCurve( Structure::Curve * base_curve, const QVector<ParameterCoord> &in_samples, const QVector<float> &in_offsets, 
		const QVector<Vec2f> &in_normals, QVector<Eigen::Vector3f> &out_points, QVector<Eigen::Vector3f> &out_normals, bool isApprox);
	static void reconstructGeometrySheet( Structure::Sheet * base_sheet, const QVector<ParameterCoord> &in_samples, const QVector<float> &in_offsets, 
		const QVector<Vec2f> &in_normals, QVector<Eigen::Vector3f> &out_points, QVector<Eigen::Vector3f> &out_normals, bool isApprox);

	// Blend skeleton bases
	static void blendCurveBases(Structure::Curve * curve1, Structure::Curve * curve2, float alpha);
	static void blendSheetBases(Structure::Sheet * sheet1, Structure::Sheet * sheet2, float alpha);

	// Helper functions
	static RMF consistentFrame( Structure::Curve * curve, Array1D_Vector4d & coords );

	// IO
	static void saveSynthesisData(Structure::Node *node, QString prefix, SynthData & input);
	static void loadSynthesisData(Structure::Node *node, QString prefix, SynthData & output);
	static void writeXYZ( QString filename, std::vector<Eigen::Vector3f> points, std::vector<Eigen::Vector3f> normals );
};

Q_DECLARE_METATYPE(Eigen::Vector3f)
Q_DECLARE_METATYPE(QVector<float>)
Q_DECLARE_METATYPE(QVector<Vec2f>)
Q_DECLARE_METATYPE(QVector<Eigen::Vector3f>)
Q_DECLARE_METATYPE(QVector<ParameterCoord>)
