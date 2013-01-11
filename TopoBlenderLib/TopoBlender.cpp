#include <set>
#include <QFileSystemModel>
#include <QDockWidget>
#include <QApplication>
#include <QMainWindow>
#include <QtConcurrentRun>

#include "TopoBlender.h"
using namespace Structure;

#include "ExportDynamicGraph.h"

#include "Task.h"
#include "Scheduler.h"
#include "SchedulerWidget.h"

// Temporary solution for output
#include "surface_mesh/IO.h"

typedef std::pair<QString, QString> PairQString;

TopoBlender::TopoBlender( Structure::Graph * graph1, Structure::Graph * graph2, GraphCorresponder * useCorresponder, Scheduler * useScheduler, QObject *parent ) : QObject(parent)
{
    sg = graph1;
    tg = graph2;
	scheduler = useScheduler;

	/// STEP 1) Compute correspondences and align mis-aligned nodes
	this->gcoor = useCorresponder;
	this->gcoor->computeCorrespondences();

	/// STEP 2) Create the magic active graph and generate tasks
	active = new Structure::Graph(*sg);

	// Set pointers to graphs
	scheduler->sourceGraph = new Structure::Graph(*sg);
	scheduler->activeGraph = active;
	scheduler->targetGraph = new Structure::Graph(*tg);

	// Shrink extra source nodes
	foreach(QString nodeID, gcoor->nonCorresSource())
	{
		// Generate task
		Task * task = new Task( active, tg, Task::SHRINK, scheduler->tasks.size() );
		task->property["nodeID"] = nodeID;
		scheduler->tasks.push_back( task );

		// Check cut node case
		if( sg->isCutNode(nodeID) )
		{
			task->property["isCutNode"];

			foreach(Structure::Link * edge, active->getEdges(nodeID))
				edge->property["isCut"].setValue(true);
		}
	}

	// Morph corresponded nodes
	QVector< PairQString > core_pairs;
	foreach (PART_LANDMARK vec2vec, gcoor->correspondences)
	{
		QVector<QString> sNodes = vec2vec.first;
		QVector<QString> tNodes = vec2vec.second;

		int sN = sNodes.size();
		int tN = tNodes.size();

		Task * task = NULL;
		Structure::Node * snode = active->getNode(*sNodes.begin());
		Structure::Node * tnode = tg->getNode(*tNodes.begin());

		// One to One
		snode->property["correspond"] = tnode->id;
		tnode->property["correspond"] = snode->id;
		task = new Task( active, tg, Task::MORPH, scheduler->tasks.size() );
		task->property["nodeID"] = snode->id;
		scheduler->tasks.push_back(task);
		core_pairs.push_back(std::make_pair(snode->id, tnode->id));

		if (tN > 1)
		{
			tNodes.erase(tNodes.begin());

			// One to remaining of many : splitting
			foreach(QString tnodeID, tNodes)
			{
				Structure::Node * clonedNode = snode->clone();
				clonedNode->id += "_cloned";
				clonedNode->property["correspond"] = tnodeID;
				tg->getNode(tnodeID)->property["correspond"] = clonedNode->id;

				// Generate task
				task = new Task( active, tg, Task::SPLIT, scheduler->tasks.size() );
				task->property["nodeID"] = clonedNode->id;
				scheduler->tasks.push_back(task);

				// Graph edit
				active->addNode( clonedNode );
			}
		}
		
		if (sN > 1)
		{
			sNodes.erase(sNodes.begin());

			// Remaining of many to One: merging
			foreach(QString snodeID, sNodes)
			{
				Structure::Node *mergedNode = tnode->clone();
				mergedNode->id += "_merged";
				active->getNode(snodeID)->property["correspond"] = mergedNode->id;
				mergedNode->property["correspond"] = snodeID;

				// Generate task
				task = new Task( active, tg, Task::MERGE, scheduler->tasks.size() );
				task->property["nodeID"] = snodeID;
				scheduler->tasks.push_back(task);

				// Graph edit - nodes
				tg->addNode(mergedNode);

				// Graph edit - edges
				foreach( Structure::Link * link, tg->getEdges(tnode->id) )
				{
					LinkCoords c1 = link->getCoord(tnode->id);
					LinkCoords c2 = link->getCoordOther(tnode->id);
					
					Structure::Link * newEdge = tg->addEdge( mergedNode, link->otherNode(tnode->id), c1, c2 );

					newEdge->property["newEdge"].setValue(true);
				}
			}
		}
	}

	// Replace edges for core nodes
	int nb = core_pairs.size();
	for(int i = 0; i < nb; i++)
	{
		QString s1 = core_pairs[i].first;
		QString t1 = core_pairs[i].second;

		for (int j = i + 1; j < nb; j++)
		{
			QString s2 = core_pairs[j].first;
			QString t2 = core_pairs[j].second;

			Structure::Link * sLink = active->getEdge(s1,s2);
			Structure::Link * tLink = tg->getEdge(t1,t2);

			// Exists on both
			if( (sLink && tLink) || (!sLink && !tLink) ) continue;

			// Exists only active [delete it]
			if( sLink && !tLink )
			{
				active->removeEdge( s1, s2 );
			}

			// Exists only target [add it]
			if( !sLink && tLink )
			{
				Structure::Node * n1 = active->getNode( tLink->n1->property["correspond"].toString() );
				Structure::Node * n2 = active->getNode( tLink->n2->property["correspond"].toString() );

				LinkCoords c1 = tLink->coord[0];
				LinkCoords c2 = tLink->coord[1];

				active->addEdge( n1, n2, c1, c2, active->linkName(n1,n2) );
			}
		}
	}

	// Grow missing target nodes
	foreach(QString nodeID, gcoor->nonCorresTarget())
	{
		// Clone and correspond
		Structure::Node * missingNode = tg->getNode(nodeID)->clone();
		missingNode->id += "_TG";
		missingNode->property["correspond"] = nodeID;
		tg->getNode(nodeID)->property["correspond"] = missingNode->id;

		// Generate task
		Task * task = new Task( active, tg, Task::GROW, scheduler->tasks.size() );

		task->property["nodeID"] = missingNode->id;
		scheduler->tasks.push_back(task);

		// Graph edit
		active->addNode( missingNode );

		// Check cut node case
		if( tg->isCutNode(nodeID) )
		{
			task->property["isCutNode"];

			foreach(Structure::Link * edge, active->getEdges(nodeID))
				edge->property["isCut"].setValue(true);
		}
	}

	// Add missing edges from target graph
	foreach (Structure::Link * e, tg->edges)
	{
		if(e->property.contains("newEdge")) continue;

		QString tn1 = e->n1->id;
		QString tn2 = e->n2->id;

		QString sn1 = e->n1->property["correspond"].toString();
		QString sn2 = e->n2->property["correspond"].toString();

		Structure::Node * n1 = active->getNode(sn1);
		Structure::Node * n2 = active->getNode(sn2);

		if(active->getEdge(n1->id, n2->id) == NULL)
		{
			active->addEdge( n1, n2, e->coord[0], e->coord[1], active->linkName(n1, n2) );
		}
	}

	// Modify edges coordinates for morphing
	foreach(Structure::Link * sLink, active->edges)
	{
		QString t_n1 = sLink->n1->property["correspond"].toString();
		QString t_n2 = sLink->n2->property["correspond"].toString();

		Structure::Link * tLink = tg->getEdge(t_n1, t_n2);
		if(!tLink) continue;
		
		sLink->property["finalCoord_n1"].setValue( qMakePair(sLink->n1->id, tLink->getCoord(t_n1)) );
		sLink->property["finalCoord_n2"].setValue( qMakePair(sLink->n2->id, tLink->getCoord(t_n2)) );
	}

	qApp->setOverrideCursor(Qt::WaitCursor);

	/// STEP 3) Order and schedule the tasks
	scheduler->schedule();

	qApp->restoreOverrideCursor();

	// Show the scheduler window:
	SchedulerWidget * sw = new SchedulerWidget( scheduler );
	QDockWidget *dock = new QDockWidget("Scheduler");
	dock->setWidget(sw);
	QMainWindow * win = (QMainWindow *) qApp->activeWindow();
	win->addDockWidget(Qt::BottomDockWidgetArea, dock);

	this->connect( scheduler, SIGNAL(startBlend()), SLOT(executeBlend()) );
}

void TopoBlender::executeBlend()
{
	/// STEP 4) Execute the tasks
	QtConcurrent::run( scheduler, &Scheduler::executeAll ); //scheduler->executeAll();
}

void TopoBlender::drawDebug()
{

}
