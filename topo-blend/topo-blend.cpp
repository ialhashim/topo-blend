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
    std::vector<Vec3d> ctrlPoint1, ctrlPoint2;


    // Curve 1
    std::vector<Vec3d> cps;
    int steps = 10;
    double theta = M_PI * 5 / steps;
    double r = M_PI;

    for(int i = 0; i <= steps; i++){
        double x = (double(i) / steps) * r;
        double y = sin(i * theta) * r * 0.25;

        ctrlPoint1.push_back(Vec3d(x - r * 0.5, y, cos(i*theta)));
    }

    // Curve 2
    ctrlPoint2.push_back(Vec3d(-1,1,0));
    ctrlPoint2.push_back(Vec3d(0,1,0));
    ctrlPoint2.push_back(Vec3d(2,1,1));
    ctrlPoint2.push_back(Vec3d(3,1,2));
    ctrlPoint2.push_back(Vec3d(4,1,3));

    std::vector<Scalar> ctrlWeight1(ctrlPoint1.size(), 1.0);
    std::vector<Scalar> ctrlWeight2(ctrlPoint2.size(), 1.0);

    int degree = 2;
    bool loop = false, open = true;

    NURBSCurve3d * c1 = new NURBSCurve3d(ctrlPoint1, ctrlWeight1, degree, loop, open);
    NURBSCurve3d * c2 = new NURBSCurve3d(ctrlPoint2, ctrlWeight2, degree, loop, open);

    Structure::Graph graph;
    graph.addEdge( new Structure::Curve(c1, "curve 1"), new Structure::Curve(c2, "curve 2") );
    graphs.push_back( graph );
}

void topoblend::test2()
{

}

Q_EXPORT_PLUGIN(topoblend)
