#pragma once;

#include "SurfaceMeshModel.h"
#include "StructureGraph.h"

#include <vector>
#include <QVector>
#include <QMap>

typedef std::vector< std::vector<float> > MATRIX;

class GraphCorresponder : public QObject
{
	Q_OBJECT

public:
    GraphCorresponder(Structure::Graph *source, Structure::Graph *target);

	// Source and target
	Structure::Graph *sg, *tg;
	QString sgName();
	QString tgName();

	// Matrix operations
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
	void removeLandmark(int id);
	void saveLandmarks(QString filename);
	void loadLandmarks(QString filename);

	void clear();
	void clearFor( QString sID, bool isSource );
	
	void doHopelessCorrespondence();

	// Distance matrices
	std::vector< std::vector<bool> > validM;
	MATRIX spatialM, sizeM, orientationM, structuralM;
	void computeValidationMatrix();
	void computeHausdorffDistanceMatrix(MATRIX & M);
	void computeLandmarkFeatureMatrix(MATRIX & M);
	void computeSizeDiffMatrix(MATRIX & M);
	void computeOrientationDiffMatrix(MATRIX & M);
	void prepareAllMatrices();

	// Final distance matrix by combining all matrices
	MATRIX disM;
	double spactialW, sizeW, orientationW, structuralW;
	void computeFinalDistanceMatrix();

	// Part to Part
	float scoreThreshold;
	void computePartToPartCorrespondences();

	// I/O
	void addCorrespondences( QVector<QString> sParts, QVector<QString> tParts );
	void addCorrespondences( QVector<QString> sParts, QVector<QString> tParts, float presetScore = -1 );
	void saveCorrespondences( QString filename, bool isWithScores = true );
	void loadCorrespondences( QString filename, bool isReversed = false );

	// Point to Point 
	void correspondAllNodes();
	void correspondTwoNodes(Structure::Node *sNode, Structure::Node *tNode);
	static void correspondTwoCurves( Structure::Curve *sCurve, Structure::Curve *tCurve, Structure::Graph * tgt );
	static void correspondTwoSheets( Structure::Sheet *sSheet, Structure::Sheet *tSheet, Structure::Graph * tgt );
	
	// Find non-corresponding nodes
	std::vector<QString> nonCorresSource(); // to kill
	std::vector<QString> nonCorresTarget(); // to grow
	void setNonCorresSource(QString sID);
	void setNonCorresTarget(QString sID);

	// Results
	bool isReady;
	std::vector<bool> sIsCorresponded, tIsCorresponded;
	QMap< PART_LANDMARK, std::vector<float> > corrScores;
	QVector< PART_LANDMARK > correspondences;

	void insertCorrespondence( PART_LANDMARK vector2vector );

	QSet<int> nonCorresS;
	QSet<int> nonCorresT;

public slots:
	void visualizePart2PartDistance(int sourceID);
	void computeCorrespondences();
signals:
	void cleared();

};
