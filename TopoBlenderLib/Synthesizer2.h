#pragma once

#include "StructureGraph.h"
using namespace Structure;

#include "Octree.h"

struct SynthSample{
	Vec4d c;
	double r, signedAngle, signedDistance;
	SynthSample(const Vec4d & coord, double angle, double distance){
		this->c = coord;
		this->r = 0;
		this->signedAngle = angle;
		this->signedDistance = distance;
	}
	SynthSample(){ c = Vec4d(0); r = 0; signedAngle = signedDistance = 0; }
	bool operator < (const SynthSample& str) const{
		return (c[0] < str.c[0]);
	}
};

struct Synthesizer2{
	static QVector<SynthSample> generateFeatureSamplesCurve( Structure::Curve * curve );
	static QVector<SynthSample> generateUniformSamplesCurve( Structure::Curve * curve, double resolution = -1);
	static QVector<SynthSample> generateSamplesCurve( Structure::Curve * curve, const std::vector<Vec3d> & points, double resolution = -1 );

	static QVector<SynthSample> generateSamplesSheet( Structure::Sheet * sheet );
	static QVector<SynthSample> generateUniformSamplesSheet( Structure::Sheet * sheet, double resolution = -1);

	static void outputSamplesCurve( QVector<SynthSample> & samples, Structure::Curve * curve );
	static void synthesizeCurve( QVector<SynthSample> samples, Structure::Curve * curve, QString id = "" );

	static Vec3d intersectionPoint( Ray ray, Octree * useTree, int * faceIndex = 0 );
};
