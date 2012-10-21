#pragma once

#include "StructureGlobal.h"
#include "StructureLink.h"

namespace Structure{

struct Node
{
	// Properties
	QString id;
    virtual QString type() = 0;
    virtual QBox3D bbox(double scaling = 1.0) = 0;
    QMap< QString, QVariant > property;

	virtual std::vector<int> controlCount() = 0;
	virtual std::vector<Vector3> controlPoints() = 0;
	virtual std::vector<Scalar> controlWeights() = 0;

	// Coordinates
	virtual void get( const Vector3& coordinates, Vector3 & pos, std::vector<Vector3> & frame = noFrame() ) = 0;
	virtual Vec2d approxProjection( const Vector3 & pos ) = 0;

	virtual std::vector< std::vector<Vector3> > discretized(Scalar resolution) = 0;

    // Visualization
    virtual void draw() = 0;
    QMap< QString, QVariant > vis_property;
};

}
