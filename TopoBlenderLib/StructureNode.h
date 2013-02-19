#pragma once

#include "StructureGlobal.h"
#include "StructureLink.h"

namespace Structure{

struct Node
{
	// Constructors
	virtual Node * clone() = 0;

	// Properties
	QString id;
	QMap< QString, QVariant > property;
    virtual QString type() = 0;
    virtual QBox3D bbox(double scaling = 1.0) = 0;

	bool hasProperty(QString propertyName) { return property.contains(propertyName); }

	virtual std::vector<int> controlCount() = 0;
	virtual std::vector<Vector3> controlPoints() = 0;
	virtual std::vector<Scalar> controlWeights() = 0;
	virtual Vector3 & controlPoint(int idx) = 0;
	virtual void setControlPoints(const std::vector<Vector3> & newPositions) = 0;
	virtual int numCtrlPnts() = 0;

	// Modifiers
	virtual void moveBy( const Vec3d & delta ) = 0;
	virtual void scale( Scalar scaleFactor ) = 0;
	virtual void rotate( double angle, Vector3 axis ) = 0;
	virtual void equalizeControlPoints( Structure::Node * other ) = 0;
	virtual void deformTo( const Vec4d & handle, const Vector3 & to ) = 0;

	// Coordinates
    virtual void get( const Vec4d& coordinates, Vector3 & pos, std::vector<Vector3> & frame ) = 0;
	virtual Vector3 position( const Vec4d& coordinates ) = 0;
	virtual Vec4d approxCoordinates( const Vector3 & pos ) = 0;
	virtual Vector3 approxProjection( const Vector3 & point ) = 0;
	virtual Vector3 center() = 0;

	// Special coordinates [todo: check type of node]
	inline Vec4d minCoord(){ return Vec4d(0.0); }
	inline Vec4d maxCoord(){ return Vec4d(1.0); }

	virtual std::vector< std::vector<Vector3> > discretized(Scalar resolution) = 0;
	virtual std::vector< std::vector<Vec4d> > discretizedPoints(Scalar resolution) = 0;
	std::vector< std::vector<Vector3> > getPoints( const std::vector< std::vector<Vec4d> > & coords ){
		std::vector< std::vector<Vector3> > pnts(coords.size(), std::vector<Vector3>(coords.front().size(), Vector3(0)));
		for(int i = 0; i < (int)coords.size(); i++)
			for(int j = 0; j < (int)coords.front().size(); j++)
				pnts[i][j] = position( coords[i][j] );
		return pnts;
	}

	// Geometric properties
	virtual Scalar area() = 0;

    // Visualization
    virtual void draw() = 0;
    QMap< QString, QVariant > vis_property;
	virtual void drawWithNames(int nID, int pointIDRange) = 0;

	// Selections
	QMap<int, QColor> selections;
	void addSelectionWithColor(int pID, QColor color = Qt::green){
		if (selections.contains(pID)) selections.remove(pID);
		else selections[pID] = color;
	}
};

}
