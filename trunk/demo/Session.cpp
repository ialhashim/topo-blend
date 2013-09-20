#include "Session.h"
#include "ui_Controls.h"

Session::Session(   Scene *scene, ShapesGallery *gallery, Controls * control,
                    Matcher * matcher, Blender * blender, QObject *parent) : QObject(parent),
                    s(scene), g(gallery), c(control), m(matcher), b(blender)
{
    this->connect( control, SIGNAL(hideAll()), SLOT(hideAll()) );
    this->connect( control, SIGNAL(showSelect()), SLOT(showSelect()));
    this->connect( control, SIGNAL(showMatch()), SLOT(showMatch()));
    this->connect( control, SIGNAL(showCreate()), SLOT(showCreate()));

	/// per-page connections
	// Matcher:
	control->connect(matcher, SIGNAL(correspondenceFromFile()), SLOT(forceManualMatch()));
	m->connect(control->ui->autoButton, SIGNAL(clicked()), SLOT(autoMode()));
	m->connect(control->ui->manualButton, SIGNAL(clicked()), SLOT(manualMode()));

	// Blender:
	control->connect(blender, SIGNAL(blendStarted()), SLOT(disableTabs()));
	control->connect(blender, SIGNAL(blendDone()), SLOT(enableTabs()));
	blender->connect(control->ui->exportButton, SIGNAL(clicked()), SLOT(exportSelected()));

	// Gallery:
	gallery->connect(blender, SIGNAL(exportShape(QString,PropertyMap)), SLOT(appendShape(QString,PropertyMap)));
}

void Session::shapeChanged(int i, QGraphicsItem * shapeItem)
{
    // Clean up if existing
    if(s->inputGraphs[i])
    {
        s->removeItem(s->inputGraphs[i]);
        delete s->inputGraphs[i];
    }
    s->inputGraphs[i] = NULL;

    ShapeItem * item = (ShapeItem *) shapeItem;

    QString graphFile = item->property["graph"].toString();

    s->inputGraphs[i] = new GraphItem(new Structure::Graph(graphFile), s->graphRect(i), s->camera);
	s->inputGraphs[i]->name = item->property["name"].toString();
    s->addItem(s->inputGraphs[i]);
    Structure::Graph * graph = s->inputGraphs[i]->g;

	// Connection
	m->connect(s->inputGraphs[i], SIGNAL(hit(GraphItem::HitResult)), SLOT(graphHit(GraphItem::HitResult)));
	m->connect(s, SIGNAL(rightClick()), SLOT(clearMatch()));
	m->connect(s, SIGNAL(doubleClick()), SLOT(setMatch()));

    // Visualization options
    graph->property["showNodes"] = false;
    foreach(Structure::Node * n, graph->nodes)
    {
        n->vis_property["meshColor"].setValue( QColor(180,180,180) );
        n->vis_property["meshSolid"].setValue( true );
    }

    emit( update() );
}

void Session::showSelect(){
	g->show();
}

void Session::showMatch(){
	m->show();
}

void Session::showCreate(){
	b->show();
}

void Session::hideAll(){
    if(g->isVisible()) g->hide();
    if(m->isVisible()) m->hide();
    if(b->isVisible()) b->hide();
}
