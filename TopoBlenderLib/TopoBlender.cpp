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

void TopoBlender::oldSetup()
{
	/// STEP 2) Create the magic active graph and generate tasks
	active = new Structure::Graph(*sg);

	// Assign active edges as so:
	foreach(Structure::Link * edge, active->edges)
		edge->property["active"].setValue(true);

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
		core_pairs.push_back( std::make_pair(snode->id, tnode->id) );

		if (tN > 1)
		{
			tNodes.erase(tNodes.begin());

			// One to remaining of many : splitting
			Structure::Node * clonedNode = NULL;
			foreach(QString tnodeID, tNodes)
			{
				clonedNode = snode->clone();
				clonedNode->id += "_cloned";
				clonedNode->property["correspond"] = tnodeID;
				clonedNode->property["isCloned"].setValue(true);
				clonedNode->property["origin"].setValue(snode->id);
				tg->getNode(tnodeID)->property["correspond"] = clonedNode->id;

				// Generate task
				task = new Task( active, tg, Task::SPLIT, scheduler->tasks.size() );
				task->property["nodeID"] = clonedNode->id;
				task->property["splitFrom"] = snode->id;
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
				task->property["mergeTo"] = snode->id;
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

				foreach(Structure::Link * sLink, active->edges)
				{
					QString t_n1 = sLink->n1->property["correspond"].toString();
					QString t_n2 = sLink->n2->property["correspond"].toString();

					Structure::Link * tLink = tg->getEdge(t_n1, t_n2);
					if(!tLink) continue;

					sLink->property["finalCoord_n1"].setValue( qMakePair(sLink->n1->id, tLink->getCoord(t_n1)) );
					sLink->property["finalCoord_n2"].setValue( qMakePair(sLink->n2->id, tLink->getCoord(t_n2)) );
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
				if(active->property.contains("mergeTo"))
				{
					sLink->property["active"].setValue(false);
					continue;
				}

				active->removeEdge( s1, s2 );
			}

			// Exists only target [add it]
			if( !sLink && tLink )
			{
				Structure::Node * n1 = active->getNode( tLink->n1->property["correspond"].toString() );
				Structure::Node * n2 = active->getNode( tLink->n2->property["correspond"].toString() );

				LinkCoords c1 = tLink->coord[0];
				LinkCoords c2 = tLink->coord[1];

				Structure::Link * newEdge = active->addEdge( n1, n2, c1, c2, active->linkName(n1,n2) );

				newEdge->property["active"].setValue(false);
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
			Structure::Link * newEdge = active->addEdge( n1, n2, e->coord[0], e->coord[1], active->linkName(n1, n2) );

			newEdge->property["active"].setValue(false);

			// Cloned nodes take coordinates from source
			Structure::Node * clonedNode = newEdge->getNodeHasProperty("isCloned", true);

			if(clonedNode)
			{
				QString originNodeID = clonedNode->property["origin"].toString();
				QString baseNodeID = newEdge->otherNode(clonedNode->id)->id;

				Structure::Link * orginLink = active->getEdge(originNodeID, baseNodeID);

				newEdge->setCoord(clonedNode->id, orginLink->getCoord(originNodeID));
				newEdge->setCoord(baseNodeID, orginLink->getCoord(baseNodeID));
			}
		}
	}

	//qDebug() << "\n\n===== <Target> :";
	//tg->printLinksInfo();
	//qDebug() << "\n\n===== Active";
	//active->printLinksInfo();

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

		super_sg->getNode(snode)->property["correspond"] = tnode;
		super_tg->getNode(tnode)->property["correspond"] = snode;
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


