#pragma once

#include <QGraphicsObject>
#include "TopoBlender.h"
#include "RMF.h"

Q_DECLARE_METATYPE(QVector<QString>);

typedef std::map< int, Array1D_Real > CurveEncoding;
typedef std::map< int, Array1D_Real > SheetEncoding;

// red, orange, yellow, green, blue
// pink, purpule, portage, lacoste, corn
static QColor TaskColors[] = { QColor(255,97,121),  QColor(255,219,88), 
	QColor(107,255,135), QColor(255,165,107) , QColor(104,126,255),
	QColor(242,5,135), QColor(113,53,242), QColor(138,109,242), QColor(3,166,60), QColor(242,203,5)};

static QString TaskNames[] = { "SHRINK", "MERGE", "MORPH", "SPLIT", "GROW" };

class Task : public QGraphicsObject
{
public:
	enum TaskType{ SHRINK, MERGE, MORPH, SPLIT, GROW };

	Task( Structure::Graph * activeGraph, Structure::Graph * targetGraph, TaskType taskType, int ID );
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent * event);
		
	// Helper functions
	NodeCoord futureOtherNodeCoord( Structure::Link *link );
	NodeCoord futureLinkCoord( Structure::Link *link );
	void copyTargetEdge( Structure::Link *tlink );

	// Prepare stage
	void prepare();

	// Main functions
	void prepareShrinkCurve();
	void prepareGrowCurve();
	void prepareShrinkCurveConstraint();
	void prepareGrowCurveConstraint();
	void prepareMorphCurve();

	void prepareShrinkSheet();
	void prepareGrowSheet();
	void prepareMorphSheet();

	// Sub routines
	void prepareShrinkCurveOneEdge(Structure::Link* l);

	// Execution stage
	void execute( double t );

	void foldCurve( double t );

	void executeGrowShrinkCurve( double t );
	void executeGrowShrinkSheet( double t );

	void executeMorphCurve( double t );
	void executeMorphSheet( double t );

	void geometryMorph( double t );

	// Helper functions
	QVector< GraphDistance::PathPointPair > weldPath( QVector< GraphDistance::PathPointPair > oldPath );
	bool isActive(double t);
	Array1D_Vector3 positionalPath( QVector< GraphDistance::PathPointPair > & from_path, int smoothingIters = 0 );
	QVector< GraphDistance::PathPointPair > smoothStart( Structure::Node * n, Vec4d startOnNode, QVector< GraphDistance::PathPointPair > oldPath );
	QVector< GraphDistance::PathPointPair > smoothEnd( Structure::Node * n, Vec4d startOnNode, QVector< GraphDistance::PathPointPair > oldPath );
	
	Structure::Node * prepareEnd( Structure::Node * n, Structure::Link * slink );
	QPair<Structure::Node*,Structure::Node*> prepareEnd2( Structure::Node * n, Structure::Link * linkA, Structure::Link * linkB );

	CurveEncoding encodeCurve(Structure::Curve * curve, Vector3 start, Vector3 end, Vector3 X, Vector3 Y, Vector3 Z);
	Array1D_Vector3 decodeCurve(CurveEncoding cpCoords, Vector3 start, Vector3 end, Vector3 X, Vector3 Y, Vector3 Z);

	SheetEncoding encodeSheet( Structure::Sheet * sheet, Vector3 origin, Vector3 X, Vector3 Y, Vector3 Z );
	Array1D_Vector3 decodeSheet( SheetEncoding cpCoords, Vector3 origin, Vector3 X, Vector3 Y, Vector3 Z );

	RMF::Frame sheetFrame(Structure::Sheet * sheet);
	Array1D_Vector3 sheetDeltas(Structure::Sheet * sheet);

	Structure::Node * node();
	Structure::Node * targetNode();
	Structure::Curve * targetCurve();
	Structure::Sheet * targetSheet();

	Structure::Link * getCoorespondingEdge( Structure::Link * link, Structure::Graph * otherGraph );
    QVector<Structure::Link*> filteredEdges(Structure::Node * n, QVector<Structure::Link*> all_edges);
	QVector<Structure::Link*> filterDissimilar( Structure::Node * n, QVector<Structure::Link*> all_edges );

	// Task properties
	TaskType type;

	Structure::Graph *active, *target;
	QMap<QString, QVariant> property;

	QString nodeID;

	// Time related
	int start;
	int length;
	int currentTime;
	bool isReady;
	int taskID;

	void setLength(int newLength);
	void reset();
	bool stillWorking();
	bool isDone;
	void setStart(int newStart);
	int endTime();
	double localT( int globalTime );

	// Visual properties
	int width;
	int height;
	QColor mycolor;

	// Resizing variables
	bool isResizing;
	int resizeDir;
	QPointF clickPos, myOldPos;
	int myOldWidth;

	// DEBUG:
	std::vector<Vec3d> debugPoints, debugPoints2;
	void drawDebug();

protected:
	virtual QVariant itemChange ( GraphicsItemChange change, const QVariant & value );

};
