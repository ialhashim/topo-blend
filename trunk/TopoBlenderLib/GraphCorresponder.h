#pragma once;

#include "SurfaceMeshTypes.h"
#include "StructureGraph.h"

#include <vector>

typedef std::vector< std::vector<float> > MATRIX;

class GraphCorresponder : public QObject
{
	Q_OBJECT

public:
    GraphCorresponder(Structure::Graph *source, Structure::Graph *target);

	// Helper functions
	QString sgName();
	QString tgName();

	// Matrix operations
	template <class Type> 
	std::vector<std::vector<Type> > transpose(const std::vector<std::vector<Type> > data);
	template <class Type>
	void initializeMatrix(std::vector< std::vector<Type> > & M, Type value);
	void normalizeMatrix(MATRIX & M);
	bool minElementInMatrix(MATRIX &M, int &row, int &column, float &minValue);

	// Point Landmarks
	QVector< POINT_LANDMARK > pointLandmarks;
	void addPointLandmark();

	QVector< POINT_ID > sPointLandmarks, tPointLandmarks;
	void prepareOneToOnePointLandmarks();

	QVector< QVector<double> > sLandmarkFeatures, tLandmarkFeatures;
	QVector< QVector<double> > computeLandmarkFeatures(Structure::Graph *g, QVector<POINT_ID> &pointLandmarks);
	double distanceBetweenLandmarkFeatures(QVector<double> sFeature, QVector<double> tFeature);

	// Part Landmarks
	QVector<PART_LANDMARK> landmarks;
	std::vector<bool> sIsLandmark, tIsLandmark;
	void addLandmarks(QVector<QString> sParts, QVector<QString> tParts);
	void removeLandmarks(int pos, int n);
	void saveLandmarks(QString filename);
	void loadLandmarks(QString filename);

	// Spatial Hausdorff distance
	float supInfDistance(std::vector<Vector3> &A, std::vector<Vector3> &B);
	float HausdorffDistance(std::vector<Vector3> &A, std::vector<Vector3> &B);

	// Distance matrices
	void computeValidationMatrix();
	void computeHausdorffDistanceMatrix(MATRIX & M);
	void computeSizeDiffMatrix(MATRIX & M);
	void computeOrientationDiffMatrix(MATRIX & M);
	void computeLandmarkFeatureMatrix(MATRIX & M);
	void prepareAllMatrices();

	// Final distance matrix by combining all matrices
	MATRIX hM, sM, oM, fM;
	double hW, sW, oW, fW;
	void computeFinalDistanceMatrix();

	// Part to Part
	float scoreThreshold;
	void computePartToPartCorrespondences();
	void saveCorrespondences(QString filename);
	void LoadCorrespondences(QString filename);

	// Point to Point 
	void correspondAllNodes();
	void correspondTwoNodes(Structure::Node *sNode, Structure::Node *tNode);
	void correspondTwoCurves(Structure::Curve *sCurve, Structure::Curve *tCurve);
	void correspondTwoSheets(Structure::Sheet *sSheet, Structure::Sheet *tSheet);

	// Find non-corresponding nodes
	std::vector<QString> nonCorresSource(); // to kill
	std::vector<QString> nonCorresTarget(); // to grow

	// Results
	std::vector<bool> sIsCorresponded, tIsCorresponded;
	std::vector<PART_LANDMARK> correspondences;
	std::vector<std::vector<float> > corrScores;

public slots:
	void visualizePart2PartDistance(int sourceID);
	void computeCorrespondences();

public:
	Structure::Graph *sg, *tg;
	std::vector< std::vector<float> > disM;
	std::vector< std::vector<bool> > validM;
};