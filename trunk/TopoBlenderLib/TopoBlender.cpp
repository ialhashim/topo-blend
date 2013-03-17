#include <set>
#include <QStack>
#include <QFileSystemModel>
#include <QDockWidget>
#include <QApplication>
#include <QMainWindow>
#include <QtConcurrentRun>

#include "TopoBlender.h"
using namespace Structure;

#include "ExportDynamicGraph.h"
using namespace DynamicGraphs;

#include "Task.h"
#include "Scheduler.h"
#include "SchedulerWidget.h"

// Temporary solution for output
#include "surface_mesh/IO.h"

typedef std::pair<QString, QString> PairQString;

Q_DECLARE_METATYPE( QSet<Structure::Node*> )

TopoBlender::TopoBlender( Structure::Graph * graph1, Structure::Graph * graph2, 
	GraphCorresponder * useCorresponder, Scheduler * useScheduler, QObject *parent ) : QObject(parent)
{
    sg = graph1;
    tg = graph2;
	scheduler = useScheduler;

	// Check for existing landmark file
	QFileInfo landMarkFile(QString("%1_%2.txt").arg(graph1->name()).arg(graph2->name()));
	if(landMarkFile.exists()) 
		useCorresponder->loadLandmarks(landMarkFile.fileName());

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

	/// Visualize super graphs
	DynamicGraph sdg(scheduler->activeGraph);
	DynamicGraph tdg(scheduler->targetGraph);

	toGraphviz(sdg, "_sourceGraph", true, QString("V = %1, E = %2").arg(sdg.nodes.size()).arg(sdg.edges.size()), "super source graph");
	toGraphviz(tdg, "_targetGraph", true, QString("V = %1, E = %2").arg(tdg.nodes.size()).arg(tdg.edges.size()), "super target graph");
	QImage img1("_sourceGraph.png"), img2("_targetGraph.png");
	QImage bothImg(img1.width() + img2.width(), qMax(img1.height(), img2.height()), QImage::Format_RGB32);
	bothImg.fill(Qt::white);
	QPainter paint;
	paint.begin(&bothImg);
	paint.drawImage(0,0,img1);
	paint.drawImage(img1.width(),0,img2);
	paint.end();
	bothImg.save("_zGraphs.png");
	// END VIZ

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
	link->property[tag] = false;
}

bool TopoBlender::taggedEdge( Structure::Link *link, QString tag )
{
	return link->property.contains(tag);
}

bool TopoBlender::isCorrespondedEdge( Structure::Link *link )
{
	return link->property.contains("corresponded") && link->property["corresponded"].toBool();
}

bool TopoBlender::isShareCorrespondedNode( Structure::Link * slink, Structure::Link * tlink )
{
	QString tn1 = slink->n1->property["correspond"].toString();
	QString tn2 = slink->n2->property["correspond"].toString();

	bool has_n1 = tlink->hasNode( tn1 );
	bool has_n2 = tlink->hasNode( tn2 );

	return has_n1 || has_n2;
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
			Structure::Node * cnode = g->getNode(clonedID);

			LinkCoords c1 = link->getCoord(other->id);
			LinkCoords c2 = link->getCoordOther(other->id);
			g->addEdge(other, cnode, c1, c2, g->linkName(other,cnode));
		}
		else
		{
			// Link other to all cloned nodes
			foreach (Structure::Node* cnode, cloned_nodes)
			{
				LinkCoords c1 = link->getCoord(other->id);
				LinkCoords c2 = link->getCoordOther(other->id);
				g->addEdge(other, cnode, c1, c2, g->linkName(other,cnode));
			}
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

	// To shrink:
	foreach(QString snodeID, gcoor->nonCorresSource())
	{
		Structure::Node *snode = super_sg->getNode(snodeID);
		Structure::Node *ctnode = snode->clone();
		super_tg->addNode(ctnode);

		ctnode->id = snodeID + "_null";
		superNodeCorr[snodeID] = ctnode->id;

		if(sg->isCutNode(snodeID))
			ctnode->property["isPossibleCut"] = true;
	}

	// To grow:
	foreach(QString tnodeID, gcoor->nonCorresTarget())
	{
		Structure::Node *tnode = super_tg->getNode(tnodeID);
		Structure::Node *csnode = tnode->clone();
		super_sg->addNode(csnode);

		csnode->id = tnodeID + "_null";
		superNodeCorr[csnode->id] = tnodeID;

		csnode->property["isReady"] = false;

		if(tg->isCutNode(tnodeID)) 
			csnode->property["isPossibleCut"] = true;
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

	// Keep tack of missing / extra nodes
	QVector<QString> missingNodes, extraNodes;

	// Store correspondences in the graphs
	foreach(QString snode, superNodeCorr.keys())
	{
		QString tnode = superNodeCorr[snode];

		Structure::Node * sn = super_sg->getNode(snode);
		Structure::Node * tn = super_tg->getNode(tnode);

		// UNDEFINED: When does this happen?
		{
			if(!sn) sn = addMissingNode(super_sg, super_tg, tn);
			if(!tn) tn = addMissingNode(super_tg, super_sg, sn);
		}

		if(tnode.contains("null")) extraNodes.push_back( snode );
		if(snode.contains("null")) missingNodes.push_back( tnode );

		sn->property["correspond"] = tnode;
		tn->property["correspond"] = snode;
	}
}

void TopoBlender::correspondTwoEdges( Structure::Link *slink, Structure::Link *tlink )
{
	if(slink->property.contains("correspond") || tlink->property.contains("correspond")) return;
	slink->property["correspond"] = tlink->id;
	tlink->property["correspond"] = slink->id;
}

// This function could be written with [graphA] and [graphB] but we should 
// keep it this way to ensure correspondence happen together and once
void TopoBlender::correspondSuperEdges()
{
	bool CASE_1 = true;		// Trivial edges, both nodes exist on both graphs
	bool CASE_2 = true;		// Edges between two [extra] or two [missing] nodes
	bool CASE_3 = true;		// Edges connecting null groups to existing nodes
	bool CASE_4 = true;		// Edges with changed ends
	bool CASE_5 = true;		// UNDEFINED: When does this happen?

	/// CASE 1: correspond trivial edges, i.e. both nodes exist on both graphs
	if( CASE_1 )
	{
		// Source graph pass
		foreach(Structure::Link * slink, super_sg->edges)
		{
			Structure::Node *sn1 = slink->n1, *sn2 = slink->n2;
			Structure::Node *tn1 = super_tg->getNode(sn1->property["correspond"].toString()),
							*tn2 = super_tg->getNode(sn2->property["correspond"].toString());
			Structure::Link * tlink = super_tg->getEdge(tn1->id, tn2->id);

			if(!tlink) continue;

			correspondTwoEdges(slink,tlink);
		}

		// Target graph pass
		foreach(Structure::Link * tlink, super_tg->edges)
		{
			Structure::Node *tn1 = tlink->n1, *tn2 = tlink->n2;
			Structure::Node *sn1 = super_sg->getNode(tn1->property["correspond"].toString()),
							*sn2 = super_sg->getNode(tn2->property["correspond"].toString());
			Structure::Link * slink = super_sg->getEdge(sn1->id, sn2->id);

			if(!slink) continue;

			correspondTwoEdges(slink,tlink);
		}
	}

	/// CASE 2: correspond edges between two [extra] or two [missing] nodes
	if( CASE_2 )
	{
		// Source graph pass
		foreach(Structure::Link * slink, super_sg->edges)
		{
			Structure::Node *sn1 = slink->n1, *sn2 = slink->n2;
			Structure::Node *tn1 = super_tg->getNode(sn1->property["correspond"].toString()),
							*tn2 = super_tg->getNode(sn2->property["correspond"].toString());
		
			// We are only looking for missing edges
			if(!(tn1->id.contains("null") && tn2->id.contains("null"))) continue;
		
			Structure::Link * tlink = addMissingLink(super_tg, slink);
			correspondTwoEdges(slink,tlink);
		}

		// Target pass
		foreach(Structure::Link * tlink, super_tg->edges)
		{
			Structure::Node *tn1 = tlink->n1, *tn2 = tlink->n2;
			Structure::Node *sn1 = super_sg->getNode(tn1->property["correspond"].toString()),
							*sn2 = super_sg->getNode(tn2->property["correspond"].toString());

			// We are only looking for missing edges
			if(!(sn1->id.contains("null") && sn2->id.contains("null"))) continue;

			Structure::Link * slink = addMissingLink(super_sg, tlink);
			correspondTwoEdges(slink,tlink);
		}
	}

	/// CASE 3: Connect null groups to existing nodes
	if( CASE_3 )
	{
		// Source graph pass
		foreach( SetNodes set, nullNodeSets( super_sg, super_tg ) )
			connectNullSet( set, super_sg, super_tg );

		// Target pass
		foreach( SetNodes set, nullNodeSets( super_tg, super_sg ) )
			connectNullSet( set, super_tg, super_sg );
	}

	/// CASE 4: edges with changed ends
	if( CASE_4 )
	{
		foreach(Structure::Node * snode, super_sg->nodes){
			Structure::Node * tnode = super_tg->getNode( snode->property["correspond"].toString() );
			
			QVector<Structure::Link*> sedges = edgesNotContain( super_sg->getEdges(snode->id), "correspond" );
			QVector<Structure::Link*> tedges = edgesNotContain( super_tg->getEdges(tnode->id), "correspond" );

			if(!sedges.size() || (sedges.size() != tedges.size())) continue;

			/// Match edges:
			// For now, arbitrarily = by index
			for(int i = 0; i < (int)sedges.size(); i++)
			{
				Structure::Link * slink = sedges[i];
				Structure::Link * tlink = tedges[i];

				correspondTwoEdges(slink,tlink);
			}
		}
	}

	// UNDEFINED: When does this happen?
	if( CASE_5 )
	{
		correspondMissingEdges(super_sg, super_tg);
		correspondMissingEdges(super_tg, super_sg);

		//removeMissingEdges(super_sg);
		//removeMissingEdges(super_tg);
	}

	/// Post processing:
	checkIntermediateCuts(tg, super_sg, super_tg);
	checkIntermediateCuts(sg, super_tg, super_sg);
}

void TopoBlender::checkIntermediateCuts(Structure::Graph * original, Structure::Graph * super_s, Structure::Graph * super_t)
{
	foreach(Node * n, original->nodes)
	{
		Node * tnode = super_t->getNode( n->id );
		if(!tnode) continue;

		QString snodeID = tnode->property["correspond"].toString();
		Node * snode = super_s->getNode( snodeID );
		if(!snode->property.contains("isPossibleCut")) continue;

		// Split original target graph and find if node [n] is part of branch
		QVector< QVector<Node*> > parts = original->split( tnode->id );
		foreach(QVector<Node*> part, parts){
			int null_count = 0;
			foreach(Node * p, part){
				Node * tn = super_t->getNode(p->id);
				if(!tn) continue;
				QString sid = tn->property["correspond"].toString();
				if(sid.contains("null")) null_count++;
			}
			if(null_count == part.size()){
				snode->property.remove("isPossibleCut");
				foreach(Node * p, part) p->property.remove("isPossibleCut");
				break;
			}
		}

		// Its an actual cut
		if(snode->property.contains("isPossibleCut"))
		{
			snode->property["isCutGroup"] = true;
			tnode->property["isCutGroup"] = true;
		}
	}
}

void TopoBlender::correspondMissingEdges( Structure::Graph * sgraph, Structure::Graph * tgraph )
{
	foreach(Structure::Node * snode, sgraph->nodes)
	{
		QVector<Structure::Link*> sedges = edgesNotContain( sgraph->getEdges(snode->id), "correspond" );
		if(!sedges.size()) continue;

		foreach(Structure::Link * slink, sedges)
		{
			Structure::Link * tlink = addMissingLink(tgraph, slink);

			correspondTwoEdges(slink,tlink);
		}
	}
}

void TopoBlender::removeMissingEdges( Structure::Graph * sgraph )
{
	foreach(Structure::Node * snode, sgraph->nodes){
		QVector<Structure::Link*> sedges = edgesNotContain( sgraph->getEdges(snode->id), "correspond" );
		if(!sedges.size()) continue;

		foreach(Structure::Link * slink, sedges)
			sgraph->removeEdge(slink->n1,slink->n2);
	}
}

QVector< SetNodes > TopoBlender::nullNodeSets( Structure::Graph * sgraph, Structure::Graph * tgraph )
{
	QVector< SetNodes > result;

	QSet<Structure::Node*> visited;

	while( visited.size() < sgraph->nodes.size() )
	{
		QStack<Structure::Node*> nodesToVisit;
		SetNodes curSet;

		// Find a null node to start from
		foreach(Structure::Node * node, sgraph->nodes)
		{
			if(visited.contains(node)) continue;

			visited.insert( node );

			if( node->id.contains("null") )
			{
				nodesToVisit.push( node );
				curSet.set.insert( node );
				break;
			}
		}

		while( !nodesToVisit.empty() )
		{
			Structure::Node * cur = nodesToVisit.pop();
			visited.insert(cur);
			curSet.set.insert(cur);

			foreach( Structure::Node * adj, sgraph->adjNodes(cur) ){
				if( !visited.contains(adj) && !curSet.set.contains(adj) ){
					if(adj->id.contains("null"))
						nodesToVisit.push( adj );
				}
			}
		}

		// Find outer nodes
		QSet<Structure::Node*> outerNodes;
		foreach(Node * sn, curSet.set){
			Node * tn = tgraph->getNode( sn->property["correspond"].toString() );
			foreach( Structure::Link * tlink, tgraph->getEdges(tn->id) ){
				Node * other = tlink->otherNode(tn->id);
				if( !curSet.set.contains(other) )
					outerNodes.insert( sgraph->getNode( other->property["correspond"].toString() ) );
			}
		}

		// Check if current set share exactly outer nodes with previous sets
		// If so, add current set to previous set
		for(int i = 0; i < (int)result.size(); i++){
			QSet<Structure::Node*> curOuter = result[i].property["outerNodes"].value< QSet<Structure::Node*> >();
			if(curOuter != outerNodes) continue;

			// Add nodes to previous set
			result[i].set += curSet.set;
			curSet.set.clear();
			break;
		}

		if( curSet.set.size() ) 
		{
			curSet.property["outerNodes"].setValue( outerNodes );
			result.push_back( curSet );
		}
	}

	// Mark set elements
	for(int i = 0; i < (int)result.size(); i++){
		foreach(Structure::Node * n, result[i].set){
			n->property["nullSet"] = i;
		}
	}

	return result;
}

void TopoBlender::connectNullSet( SetNodes nullSet, Structure::Graph * source, Structure::Graph * target )
{
	/// 0) Find outer nodes 
	QSet< Structure::Node* > s_outter_nodes = nullSet.property["outerNodes"].value< QSet< Structure::Node* > >();
	foreach(Structure::Node * snode, nullSet.set){
		Structure::Node * tnode = target->getNode( snode->property["correspond"].toString() );
		foreach(Structure::Link * tlink, nonCorrespondEdges(tnode, target))
		{
			Structure::Node *s1 = source->getNode( correspondingNode(tlink,0) );
			Structure::Node *s2 = source->getNode( correspondingNode(tlink,1) );
		}
	}

	/// 1) Find external edges = not yet corresponded
	QSet< Structure::Link* > t_ext_edges, s_ext_edges;

	foreach(Structure::Node * snode, nullSet.set){
		Structure::Node * tnode = target->getNode( snode->property["correspond"].toString() );
		foreach(Structure::Link * tlink, nonCorrespondEdges(tnode, target))
		{
			// Links to correspond in target graph
			t_ext_edges.insert( tlink );
			
			// Links to correspond in source graph
			Structure::Node *s1 = source->getNode( correspondingNode(tlink,0) );
			Structure::Node *s2 = source->getNode( correspondingNode(tlink,1) );
			QVector<Structure::Link*> non_corr_edges = nonCorrespondEdges(s1, source) + nonCorrespondEdges(s2, source);

			foreach(Structure::Link* slink, non_corr_edges) 
			{
				if( s_outter_nodes.contains(slink->n1) && s_outter_nodes.contains(slink->n2) )
					s_ext_edges.insert( slink );
			}
		}
	}
	if( !t_ext_edges.size() ) return; // shouldn't happen?

	// Count number of edges toward each outer node on target
	QMap<Structure::Node*,int> s_outter_counts;
	foreach( Structure::Link * l, t_ext_edges )
	{
		Structure::Node * n1 = source->getNode(l->n1->property["correspond"].toString());
		Structure::Node * n2 = source->getNode(l->n2->property["correspond"].toString());
		if(s_outter_nodes.contains(n1)) s_outter_counts[n1]++;
		if(s_outter_nodes.contains(n2)) s_outter_counts[n2]++;
	}

	bool isCutGroup = false;

	/// 2) Classify external edges either as [1-paring] or otherwise
	//	a. 1-paring find existing and match
	//	b. otherwise add edge
	foreach( Structure::Link * tlink, t_ext_edges )
	{
		Structure::Link * slink = NULL;

		// Find candidate edges to match with tlink
		QVector<Structure::Link *> s_candidates;

		foreach( Structure::Link * otherLink, s_ext_edges )
		{
			if(s_outter_counts[otherLink->n1] > 1) { isCutGroup = true; continue;}
			if(s_outter_counts[otherLink->n2] > 1) { isCutGroup = true; continue;}

			if( isShareCorrespondedNode( otherLink, tlink ) )
				s_candidates.push_back( otherLink );
		}

		if( s_candidates.size() == 1 )
		{
			slink = s_candidates.front();
			isCutGroup = true;

			slink->property["changingEnd"] = true;
			tlink->property["changingEnd"] = true;
		}
		else
		{
			slink = addMissingLink( source, tlink );
		}

		correspondTwoEdges(slink,tlink);
	}

	/// 3) Clean up "redundant" edges
	foreach( Structure::Link * l, s_ext_edges )
	{
		if(!l->property.contains("correspond")){
			source->removeEdge(l->n1->id, l->n2->id);
		}
	}

	foreach(Structure::Node * snode, nullSet.set)
	{
		Structure::Node * tnode = target->getNode( snode->property["correspond"].toString() );

		if( isCutGroup )
		{
			snode->property["isCutGroup"] = true;
			tnode->property["isCutGroup"] = true;
		}
	}
}

QVector<Structure::Link*> TopoBlender::nonCorrespondEdges( Structure::Node * node, Structure::Graph * graph )
{
	return edgesNotContain(graph->getEdges( node->id ), "correspond");
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

	// Visualization - color similar items
	foreach(Structure::Node * n, super_sg->nodes)
	{
		QString corrId = n->property["correspond"].toString();
		Structure::Node * other = super_tg->getNode(corrId);
		if(other) n->vis_property["color"].setValue( other->vis_property["color"] );
	}

	// Correspond edges in super graphs
	correspondSuperEdges();
}

void TopoBlender::generateTasks()
{
	foreach(QString snodeID, superNodeCorr.keys())
	{
		QString tnodeID = superNodeCorr[snodeID];

		Task * task;
		
		if (snodeID.contains("null"))  // Grow
			task = new Task( active, super_tg, Task::GROW, scheduler->tasks.size() );
		else if (tnodeID.contains("null")) // Shrink
			task = new Task( active, super_tg, Task::SHRINK, scheduler->tasks.size() );
		else
			task = new Task( active, super_tg, Task::MORPH, scheduler->tasks.size() );

		task->setNode( snodeID );
		scheduler->tasks.push_back( task );
	}
}


void TopoBlender::equalizeResolutions()
{
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
}

Structure::Node * TopoBlender::addMissingNode( Structure::Graph *toGraph, Structure::Graph * fromGraph, Structure::Node * fromNode )
{
	Structure::Node * newNode = fromNode->clone();

	newNode->id = fromNode->id + "_null";
	newNode->property["correspond"] = fromNode->id;

	return toGraph->addNode( newNode );
}

Structure::Link * TopoBlender::addMissingLink( Structure::Graph *g, Structure::Link * link )
{
	Structure::Node * bn1 = link->n1;
	Structure::Node * bn2 = link->n2;
	Structure::Node * an1 = g->getNode(bn1->property["correspond"].toString());
	Structure::Node * an2 = g->getNode(bn2->property["correspond"].toString());
	LinkCoords c1 = link->getCoord(bn1->id);
	LinkCoords c2 = link->getCoordOther(bn1->id);

	Structure::Link * existEdge = g->getEdge(an1->id,an2->id);

	if(!existEdge)
		return g->addEdge(an1, an2, c1, c2, g->linkName(an1, an2));
	else
		return existEdge;
}

QVector<Structure::Link*> TopoBlender::edgesContain( QVector<Structure::Link*> edges, QString property_name )
{
	QVector<Structure::Link*> result;
	foreach(Structure::Link * edge, edges) 
		if(edge->property.contains(property_name)) 
			result.push_back(edge);
	return result;
}

QVector<Structure::Link*> TopoBlender::edgesNotContain( QVector<Structure::Link*> edges, QString property_name )
{
	QVector<Structure::Link*> result;
	foreach(Structure::Link * edge, edges) 
		if(!edge->property.contains(property_name)) 
			result.push_back(edge);
	return result;
}

QString TopoBlender::correspondingNode( Structure::Link *link, int i )
{
	if(i == 0)	return link->n1->property["correspond"].toString();
	else		return link->n2->property["correspond"].toString();
}
