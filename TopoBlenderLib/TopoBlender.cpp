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

	/// STEP 2) Generate super graphs
	generateSuperGraphs();

	/// STEP 3) Generate tasks 
	active = super_sg;
	generateTasks();

	/// STEP 4) Order and schedule the tasks
	scheduler->activeGraph = active;
	scheduler->sourceGraph = super_sg;
	scheduler->targetGraph = super_tg;
	qApp->setOverrideCursor(Qt::WaitCursor);
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

// Super graphs

bool TopoBlender::isExtraNode( Structure::Node *node )
{
	return node->property["correspond"].toString().contains("null");
}

bool TopoBlender::isExtraEdge( Structure::Link *link )
{
	return link->property["correspond"].toString().contains("null");
}

void TopoBlender::tagEdge( Structure::Link *link, QString tag )
{
	link->property[tag] = 0;
}

bool TopoBlender::taggedEdge( Structure::Link *link, QString tag )
{
	return link->property.contains(tag);
}

QVector<QString> TopoBlender::cloneGraphNode( Structure::Graph *g, QString nodeID, int N )
{
	Structure::Node * node = g->getNode(nodeID);

	// Clone the nodes
	QVector<QString> cloned_node_IDs;
	QVector<Structure::Node *> cloned_nodes;
	for (int i = 0; i < N; i++)
	{
		Structure::Node * n = node->clone();
		n->id = nodeID + "_" + QString::number(i);

		g->addNode(n);

		cloned_node_IDs.push_back(n->id);
		cloned_nodes.push_back(n);
	}

	// Link the cloned nodes
	int currCloned = 0;
	foreach(Structure::Link *link, g->getEdges(nodeID))
	{
		Structure::Node * other = link->otherNode(nodeID);
		QString otherID = other->id;

		if (isExtraNode(other))
		{
			if (currCloned >= N)
			{
				continue;
				qDebug() << "Warning: clone nodes error.";
			}

			// Link other to one of the cloned node
			QString clonedID = cloned_nodes[currCloned++]->id;
			g->addEdge(otherID, clonedID);
		}
		else
		{
			// Link other to all cloned nodes
			foreach (Structure::Node* cnode, cloned_nodes)
				g->addEdge(other->id, cnode->id);
		}

		// Remove this link
		g->removeEdge(node, other);
	}

	// remove the original node
	g->removeNode(nodeID);

	return cloned_node_IDs;
}

void TopoBlender::correspondSuperNodes()
{
	// Add virtual corresponding nodes for missing nodes
	foreach(QString snodeID, gcoor->nonCorresSource())
	{
		Structure::Node *snode = super_sg->getNode(snodeID);
		Structure::Node *ctnode = snode->clone();
		super_tg->addNode(ctnode);

		ctnode->id = snodeID + "_null";
		superNodeCorr[snodeID] = ctnode->id;
	}

	foreach(QString tnodeID, gcoor->nonCorresTarget())
	{
		Structure::Node *tnode = super_tg->getNode(tnodeID);
		Structure::Node *csnode = tnode->clone();
		super_sg->addNode(csnode);
		csnode->property["isReady"] = false;

		csnode->id = tnodeID + "_null";
		superNodeCorr[csnode->id] = tnodeID;
	}

	// Build node correspondence for corresponded nodes
	QVector< PairQString > core_pairs;
	foreach (PART_LANDMARK vec2vec, gcoor->correspondences)
	{
		QVector<QString> sNodes = vec2vec.first;
		QVector<QString> tNodes = vec2vec.second;

		int sN = sNodes.size();
		int tN = tNodes.size();
		Structure::Node * snode = super_sg->getNode(sNodes.front());
		Structure::Node * tnode = super_tg->getNode(tNodes.front());

		// 1-to-1
		if (sN == 1 && tN == 1)	superNodeCorr[snode->id] = tnode->id;

		// N-to-1
		if (sN > 1)	{
			QVector<QString> ctnodeIDs = cloneGraphNode(super_tg, tnode->id, sN);
			for (int i = 0; i < sN; i++) superNodeCorr[sNodes[i]] = ctnodeIDs[i];
		}

		// 1-to-N
		if (tN > 1)	{
			QVector<QString> csnodeIDs = cloneGraphNode(super_sg, snode->id, tN);
			for (int i = 0; i < tN; i++) superNodeCorr[csnodeIDs[i]] = tNodes[i];
		}
	}

	// Store correspondences in the graphs
	foreach(QString snode, superNodeCorr.keys())
	{
		QString tnode = superNodeCorr[snode];

		Structure::Node * sn = super_sg->getNode(snode);
		Structure::Node * tn = super_tg->getNode(tnode);

		sn->property["correspond"] = tnode;
		tn->property["correspond"] = snode;
	}
}

void TopoBlender::correspondSuperEdges()
{
	// Links for extra nodes are mapped to null edges
	// Null edges actually doesn't exist for now
	// Extra source nodes: do nothing
	// Extra target nodes: add edges for the null source node after growing is done
	foreach(QString snodeID, superNodeCorr.keys())
	{
		Structure::Node *snode = super_sg->getNode(snodeID);
		if ( isExtraNode(snode) )
		{
			foreach (Structure::Link *sl, super_sg->getEdges(snodeID))
			{
				superEdgeCorr[sl->id] = sl->id + "_null";
				tagEdge(sl, "corresponded");
			}

			// snode and tnode cannot be null at the same time
			continue;
		}

		QString tnodeID = superNodeCorr[snodeID];
		Structure::Node *tnode = super_tg->getNode(tnodeID);
		if (isExtraNode(tnode))
		{
			foreach (Structure::Link *tl, super_tg->getEdges(tnodeID))
			{
				superEdgeCorr[tl->id + "_null"] = tl->id;
				tagEdge(tl, "corresponded");
			}
		}
	}


	// Both ends are corresponded -> for sure
	QMap<QString, QString>::Iterator begin = superNodeCorr.begin(), end = superNodeCorr.end();
	QMap<QString, QString>::Iterator itr1, itr2;
	for (itr1 = begin; itr1 != end; itr1 ++) {
		for (itr2 = itr1+1; itr2 != end; itr2++)
		{
			QString sn1 = itr1.key(), sn2 = itr2.key();
			QString tn1 = itr1.value(), tn2 = itr2.value();

			Structure::Link *slink = super_sg->getEdge(sn1, sn2);
			Structure::Link *tlink = super_tg->getEdge(tn1, tn2);

			if ( slink && tlink)	
			{
				superEdgeCorr[slink->id] = tlink->id;

				// Mark
				slink->property["corresponded"] = true;
				tlink->property["corresponded"] = true;
			}
		}
	}

	// Two corresponded nodes have the same number of non-corresponded edges
	// Correspond them one by one
	for (itr1 = begin; itr1 != end; itr1 ++) 
	{
		QString snode = itr1.key();
		QString tnode = itr1.value();

		// Non-corresponded links
		QVector<Structure::Link *> slinks, tlinks;
		foreach(Structure::Link * l, super_sg->getEdges(snode))
			if (! taggedEdge(l, "corresponded")) slinks.push_back(l);
		foreach(Structure::Link * l, super_tg->getEdges(tnode))
			if (! taggedEdge(l, "corresponded")) tlinks.push_back(l);


		// Correspond them only if numbers of links are equal
		if (slinks.size() == tlinks.size())
		{
			for (int i = 0; i < slinks.size(); i++)
			{
				superEdgeCorr[slinks[i]->id] = tlinks[i]->id;

				// Mark
				tagEdge(slinks[i], "corresponded");
				tagEdge(tlinks[i], "corresponded");
			}
		}
	}

	// Remove remaining non-correspond edges
	// These edges seem useless (???)
	foreach (Structure::Link *sl, super_sg->edges)
	{
		if (!taggedEdge(sl, "corresponded"))
			super_sg->removeEdge(sl->n1, sl->n2);
	}

	foreach (Structure::Link *tl, super_tg->edges)
	{
		if (!taggedEdge(tl, "corresponded"))
			super_tg->removeEdge(tl->n1, tl->n2);
	}

	// Store correspondences in the graphs
	foreach (QString slink, superEdgeCorr.keys())
	{
		QString tlink = superEdgeCorr[slink];

		if (!slink.contains("null"))
			super_sg->getEdge(slink)->property["correspond"] = tlink;

		if (!tlink.contains("null"))
			super_tg->getEdge(tlink)->property["correspond"] = slink;
	}
}

void TopoBlender::generateSuperGraphs()
{
	// Equalize resolution for corresponded nodes
	equalizeResolutions();

	// Two super graphs have one-to-one correspondence between nodes and edges
	// Two corresponded edge don't have to link two corresponded nodes
	super_sg = new Structure::Graph(*sg);
	super_tg = new Structure::Graph(*tg);

	// Correspond nodes in super graphs
	correspondSuperNodes();

	// Correspond edges in super graphs
	correspondSuperEdges();
}

void TopoBlender::generateTasks()
{
	foreach(QString snode, superNodeCorr.keys())
	{
		QString tnode = superNodeCorr[snode];

		Task * task;
		
		if (snode.contains("null"))  // Grow
			task = new Task( active, super_tg, Task::GROW, scheduler->tasks.size() );
		else if (tnode.contains("null")) // Shrink
			task = new Task( active, super_tg, Task::SHRINK, scheduler->tasks.size() );
		else
			task = new Task( active, super_tg, Task::MORPH, scheduler->tasks.size() );

		task->property["nodeID"] = snode;
		scheduler->tasks.push_back( task );
	}
}


void TopoBlender::equalizeResolutions()
{
	qDebug() << "Equalizing resolutions...";

	foreach (PART_LANDMARK vec2vec, gcoor->correspondences)
	{
		QVector<QString> sNodes = vec2vec.first;
		QVector<QString> tNodes = vec2vec.second;

		// Pick up the best one 
		Structure::Node * bestNode = NULL;
		int bestResolution = 0;

		foreach( QString sid, sNodes)
		{
			Structure::Node * snode = sg->getNode(sid);
			if (snode->numCtrlPnts() > bestResolution)
			{
				bestResolution = snode->numCtrlPnts();
				bestNode = snode;
			}
		}

		foreach( QString tid, tNodes)
		{
			Structure::Node * tnode = tg->getNode(tid);
			if (tnode->numCtrlPnts() > bestResolution)
			{
				bestResolution = tnode->numCtrlPnts();
				bestNode = tnode;
			}
		}


		// Equalize resolution to the best one
		foreach( QString sid, sNodes)
		{
			Structure::Node * snode = sg->getNode(sid);
			if (snode != bestNode) snode->equalizeControlPoints(bestNode);
		}

		foreach( QString tid, tNodes)
		{
			Structure::Node * tnode = tg->getNode(tid);
			if (tnode != bestNode) tnode->equalizeControlPoints(bestNode);
		}
	}

	qDebug() << "Equalizing resolution is done.";
}



