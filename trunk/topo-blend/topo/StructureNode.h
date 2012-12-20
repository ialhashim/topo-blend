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
    virtual QString type() = 0;
    virtual QBox3D bbox(double scaling = 1.0) = 0;
    QMap< QString, QVariant > property;

	bool hasProperty(QString propertyName) { return property.contains(propertyName); }

	virtual std::vector<int> controlCount() = 0;
	virtual std::vector<Vector3> controlPoints() = 0;
	virtual std::vector<Scalar> controlWeights() = 0;
	virtual Vector3 & controlPoint(int idx) = 0;

	// Coordinates
	virtual void get( const Vec4d& coordinates, Vector3 & pos, std::vector<Vector3> & frame = noFrame() ) = 0;
	virtual Vector3 position( const Vec4d& coordinates ) = 0;
	virtual Vec4d approxCoordinates( const Vector3 & pos ) = 0;
	virtual Vector3 approxProjection( const Vector3 & point ) = 0;
	virtual Vector3 center() = 0;

	// Special coordinates [todo: check type of node]
	inline Vec4d minCoord(){ return Vec4d(0.0); }
	inline Vec4d maxCoord(){ return Vec4d(1.0); }

	virtual std::vector< std::vector<Vector3> > discretized(Scalar resolution) = 0;
	virtual std::vector< std::vector<Vector3> > discretizedPoints(Scalar resolution) = 0;

	// Geometric properties
	virtual Scalar area() = 0;

    // Visualization
    virtual void draw() = 0;
    QMap< QString, QVariant > vis_property;
};

}
