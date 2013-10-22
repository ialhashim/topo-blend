#pragma once

#include "StructureGlobal.h"
#include "StructureLink.h"

#define DECODE_ZERO_THRESHOLD 1e-7

namespace Structure{

struct Node
{
	// Constructors
	virtual Node * clone() = 0;
	virtual ~Node(){}

	// Properties
	QString id;
	PropertyMap property;
    virtual QString type() = 0;
    virtual Eigen::AlignedBox3d bbox(double scaling = 1.0) = 0;

	bool hasProperty(QString propertyName) { return property.contains(propertyName); }

	virtual std::vector<int> controlCount() = 0;
	virtual std::vector<Vector3> controlPoints() = 0;
	virtual std::vector<Scalar> controlWeights() = 0;
	virtual Vector3 & controlPoint(int idx) = 0;
	virtual void setControlPoints(const std::vector<Vector3> & newPositions) = 0;
	virtual int numCtrlPnts() = 0;

	// Modifiers
	virtual void moveBy( const Vector3d & delta ) = 0;
	virtual void scale( Scalar scaleFactor ) = 0;
	virtual void rotate( double angle, Vector3 axis ) = 0;
	virtual void refineControlPoints(int nU, int nV = 0) = 0;
	virtual void equalizeControlPoints( Structure::Node * other ) = 0;
	virtual void deformTo( const Vector4d & handle, const Vector3 & to, bool isRigid ) = 0;
	virtual void deformTwoHandles( Vector4d& handleA, Vector3 newPosA, Vector4d& handleB, Vector3 newPosB ) = 0;

	// Coordinates
    virtual void get( const Vector4d& coordinates, Vector3 & pos, std::vector<Vector3> & frame ) = 0;
	virtual Vector3 position( const Vector4d& coordinates ) = 0;
	virtual Vector4d approxCoordinates( const Vector3 & pos ) = 0;
	virtual Vector3 approxProjection( const Vector3 & point ) = 0;
	virtual Vector3 center() = 0;

	// Special coordinates [todo: check type of node]
	inline Vector4d minCoord(){ return Vector4d(0.0); }
	inline Vector4d maxCoord(){ return Vector4d(1.0); }

	virtual Array2D_Vector3 discretized(Scalar resolution) = 0;
	virtual Array2D_Vector4d discretizedPoints(Scalar resolution) = 0;
	Array2D_Vector3 getPoints( const Array2D_Vector4d & coords ){
		Array2D_Vector3 pnts(coords.size(), std::vector<Vector3>(coords.front().size(), Vector3(0,0,0)));
		for(int i = 0; i < (int)coords.size(); i++)
			for(int j = 0; j < (int)coords.front().size(); j++)
				pnts[i][j] = position( coords[i][j] );
		return pnts;
	}

	// Geometric properties
	virtual Scalar area() = 0;
	virtual Array1D_Vector3 geometricDiff( Structure::Node * other ){
		Array1D_Vector3 cp1 = controlPoints(), cp2 = other->controlPoints(), result = cp1;
		for(int i = 0; i < (int)cp1.size(); i++){
			if(i < (int)cp2.size())	result[i] = (cp1[i] - cp2[i]);
		}
		return result;
	}
	virtual Scalar distortion( Structure::Node * other ){
		if(!other) return 0;
		Array1D_Vector3 cp1 = controlPoints(), cp2 = other->controlPoints();
		if(cp1.size() != cp2.size()) return 0;
		Vector3 mean1(0,0,0), mean2(0,0,0);
		foreach(Vector3 p, cp1) mean1 += p;
		foreach(Vector3 p, cp2) mean2 += p;
		mean1 /= cp1.size();
		mean2 /= cp2.size();
		double maxDistort = 0;
		for(int i = 0; i < (int)cp1.size(); i++) maxDistort = qMax(maxDistort, (cp1[i] - cp2[i]).norm());
		return maxDistort;
	}

	// Selections
	QMap<int, QColor> selections;
	void addSelectionWithColor(int pID, QColor color = Qt::green){
		if (selections.contains(pID)) selections.remove(pID);
		else selections[pID] = color;
	}

    // Visualization
    virtual void draw(bool isShowCtrlPts = false) = 0;
    QMap< QString, QVariant > vis_property;
	virtual void drawWithNames(int nID, int pointIDRange) = 0;

	std::vector<Vector3> debugPoints,debugPoints2,debugPoints3;
};

}
