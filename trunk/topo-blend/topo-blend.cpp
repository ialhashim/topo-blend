#include <QElapsedTimer>
#include "topo-blend.h"
#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "interfaces/ModePluginDockWidget.h"
#include "../CustomDrawObjects.h"

topoblend::topoblend(){
    widget = NULL;
}

void topoblend::create()
{
    if(!widget)
    {
        ModePluginDockWidget * dockwidget = new ModePluginDockWidget(mainWindow());
        widget = new topo_blend_widget(this);
        dockwidget->setWidget(widget);
        dockwidget->setWindowTitle(widget->windowTitle());
        mainWindow()->addDockWidget(Qt::RightDockWidgetArea,dockwidget);
    }
}

void topoblend::decorate()
{
    foreach(const Structure::Graph g, graphs)
        g.draw();

    glColor3d(1,1,1);
    drawArea()->drawText(40,40, "TopoBlend mode.");
}

void topoblend::test1()
{
	QElapsedTimer assembleTimer; assembleTimer.start(); 

    Structure::Graph chair1, chair2;

    NURBSRectangle backSheet = NURBSRectangle::createSheet(2,1, Vector3(0,-0.5,2));
	NURBSRectangle seatSheet = NURBSRectangle::createSheet(2,2, Vector3(0,1,0), Vector3(1,0,0), Vector3(0,1,0));
	
	NURBSCurve backLeft = NURBSCurve::createCurve(Vector3(-1,0,0), Vector3(-1,-0.5,1.5));
	NURBSCurve backRight = NURBSCurve::createCurve(Vector3(1,0,0), Vector3(1,-0.5,1.5));
	NURBSCurve backLeft2 = NURBSCurve::createCurve(Vector3(-0.75,0,0), Vector3(-0.75,-0.5,1.5));
	NURBSCurve backRight2 = NURBSCurve::createCurve(Vector3(0.75,0,0), Vector3(0.75,-0.5,1.5));

	NURBSCurve frontLegLeft = NURBSCurve::createCurve(Vector3(-1,1.75,0), Vector3(-1,1.9,-2));
	NURBSCurve frontLegRight = NURBSCurve::createCurve(Vector3(1,1.75,0), Vector3(1,1.9,-2));
	NURBSCurve backLegLeft = NURBSCurve::createCurve(Vector3(-1,0.25,0), Vector3(-1,0,-2));
	NURBSCurve backLegRight = NURBSCurve::createCurve(Vector3(1,0.25,0), Vector3(1,0,-2));

	// Chair 1
    chair1.addNode( new Structure::Sheet(backSheet, "BackSheet") );
	chair1.addNode( new Structure::Curve(backLeft, "BackLeft") );
	chair1.addNode( new Structure::Curve(backRight, "BackRight") );
	chair1.addNode( new Structure::Sheet(seatSheet, "SeatSheet") );

	chair1.addNode( new Structure::Curve(frontLegLeft, "FrontLegLeft") );
	chair1.addNode( new Structure::Curve(frontLegRight, "FrontLegRight") );
	chair1.addNode( new Structure::Curve(backLegLeft, "BackLegLeft") );
	chair1.addNode( new Structure::Curve(backLegRight, "BackLegRight") );

	// Chair 2
	chair2.addNode( new Structure::Sheet(backSheet, "BackSheet") );
	chair2.addNode( new Structure::Curve(backLeft, "BackLeft") );
	chair2.addNode( new Structure::Curve(backLeft2, "BackLeft2") );
	chair2.addNode( new Structure::Curve(backRight2, "BackRight2") );
	chair2.addNode( new Structure::Curve(backRight, "BackRight") );
	chair2.addNode( new Structure::Sheet(seatSheet, "SeatSheet") );

	chair2.addNode( new Structure::Curve(frontLegLeft, "FrontLegLeft") );
	chair2.addNode( new Structure::Curve(frontLegRight, "FrontLegRight") );
	chair2.addNode( new Structure::Curve(backLegLeft, "BackLegLeft") );
	chair2.addNode( new Structure::Curve(backLegRight, "BackLegRight") );

	// Add edges
	chair1.addEdge( chair1.getNode("BackSheet"), chair1.getNode("BackLeft") );
	chair1.addEdge( chair1.getNode("BackSheet"), chair1.getNode("BackRight") );

	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackLeft") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackRight") );

	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("FrontLegLeft") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("FrontLegRight") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackLegLeft") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackLegRight") );

    // Set scene bounds
    Vector3 a = chair1.bbox().minimum();
    Vector3 b = chair1.bbox().maximum();
    drawArea()->setSceneBoundingBox(qglviewer::Vec(a.x(), a.y(), a.z()), qglviewer::Vec(b.x(), b.y(), b.z()));

	// Save to file
	//chair1.saveToFile("chair1.xml");
	//chair1.loadFromFile("chair1.xml");

	graphs.push_back( chair1 );
	graphs.push_back( chair2 );

	qDebug() << assembleTimer.elapsed() << " ms";

    drawArea()->updateGL();
}

void topoblend::test2()
{

}

Q_EXPORT_PLUGIN(topoblend)
