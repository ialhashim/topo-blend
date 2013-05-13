#pragma once
#include <QGraphicsObject>
#include "TopoBlender.h"
#include "RMF.h"

class Task : public QGraphicsObject
{
public:
	enum TaskType{ SHRINK, MERGE, MORPH, SPLIT, GROW };

	Task( Structure::Graph * activeGraph, Structure::Graph * targetGraph, TaskType taskType, int ID );
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	// Set node
	void setNode( QString nodeID );

	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent * event);
		
	// Helper functions
	NodeCoord futureOtherNodeCoord( Structure::Link *link );
	NodeCoord futureLinkCoord( Structure::Link *link );
	void copyTargetEdge( Structure::Link *tlink );

	// Prepare stage
	void prepare();
	void prepareMorphEdges();

	// Execution stage
	void execute( double t );
	void executeMorphEdges( double t );

    // Geometry morphing stage
	void geometryMorph( double t );

	// Helper functions
	bool isActive(double t);
	bool isCrossing();
	bool isCutting();
	bool isCuttingReal();

	bool ungrownNode(QString nid);

	QVector< GraphDistance::PathPointPair > weldPath( QVector< GraphDistance::PathPointPair > oldPath );
	QVector< GraphDistance::PathPointPair > smoothStart( Structure::Node * n, Vec4d startOnNode, QVector< GraphDistance::PathPointPair > oldPath );
	QVector< GraphDistance::PathPointPair > smoothEnd( Structure::Node * n, Vec4d startOnNode, QVector< GraphDistance::PathPointPair > oldPath );
	bool isPathOnSingleNode( QVector< GraphDistance::PathPointPair > path );

	Structure::Node * addAuxNode(Vec3d position, Structure::Graph * g);
	Structure::Node * prepareEnd( Structure::Node * n, Structure::Link * slink );
	QPair<Structure::Node*,Structure::Node*> prepareEnd2( Structure::Node * n, Structure::Link * linkA, Structure::Link * linkB );

	RMF::Frame curveFrame( Structure::Curve * curve, bool isFlip = false );
	RMF::Frame sheetFrame( Structure::Sheet * sheet );
	Array1D_Vector3 sheetDeltas(Structure::Sheet * sheet);

	std::vector<RMF::Frame> smoothRotateFrame( RMF::Frame sframe, Eigen::Quaterniond &result, int steps = 100 );

	// Quick access
	Structure::Node * node();
    Structure::Node * targetNode();

	// Edge helpers
	Structure::Link * getCoorespondingEdge( Structure::Link * link, Structure::Graph * otherGraph );
	QVector<Structure::Link*> filterEdges( Structure::Node * n, QVector<Structure::Link*> all_edges );
	Structure::Link * preferredEnd(Structure::Node * n, QVector<Structure::Link*> edges, Structure::Graph * g);

    // Preperation and execution based on type
    virtual void prepareCurve(){}
    virtual void executeCurve(double t){ Q_UNUSED(t) }

    virtual void prepareSheet(){}
    virtual void executeSheet(double t){ Q_UNUSED(t) }

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

// Useful global definitions
typedef std::vector< std::pair<double,double> > VectorPairDouble;

// Global meta types
Q_DECLARE_METATYPE( Vector3 )
Q_DECLARE_METATYPE( Vec4d )
Q_DECLARE_METATYPE( VectorPairDouble )
Q_DECLARE_METATYPE( GraphDistance::PathPointPair )
Q_DECLARE_METATYPE( RMF )
Q_DECLARE_METATYPE( RMF::Frame )
Q_DECLARE_METATYPE( std::vector<RMF::Frame> )
Q_DECLARE_METATYPE( CurveEncoding )
Q_DECLARE_METATYPE( QVector<QString> )
Q_DECLARE_METATYPE( QVector<Structure::Link*> )
Q_DECLARE_METATYPE( Eigen::Quaterniond )

static inline double rad_to_deg(const double& _angle){ return 180.0*(_angle/M_PI); }

// red, orange, yellow, green, blue
// pink, purpule, portage, lacoste, corn
static QColor TaskColors[] = { QColor(255,97,121),  QColor(255,219,88),
    QColor(107,255,135), QColor(255,165,107) , QColor(104,126,255),
    QColor(242,5,135), QColor(113,53,242), QColor(138,109,242), QColor(3,166,60), QColor(242,203,5)};

static QString TaskNames[] = { "SHRINK", "MERGE", "MORPH", "SPLIT", "GROW" };
