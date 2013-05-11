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

#include "TaskCurve.h"
#include "TaskSheet.h"

#include "Scheduler.h"
#include "SchedulerWidget.h"

// Temporary solution for output
#include "surface_mesh/IO.h"

typedef std::pair<QString, QString> PairQString;

Q_DECLARE_METATYPE( Structure::Sheet* )
Q_DECLARE_METATYPE( QSet<Structure::Node*> )

TopoBlender::TopoBlender( Structure::Graph * graph1, Structure::Graph * graph2, 
	GraphCorresponder * useCorresponder, Scheduler * useScheduler, QObject *parent ) : QObject(parent)
{
    sg = graph1;
    tg = graph2;
	scheduler = useScheduler;

	// Check for existing landmark file
	QFileInfo coordFile(QString("%1_%2.txt").arg(graph1->name()).arg(graph2->name()));
	if(coordFile.exists()) 
	{
		useCorresponder->loadCorrespondences(coordFile.fileName());
	}

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
	if(false)
	{
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
	}
	// END VIZ

	// Show the scheduler window:
	scheduler->widget = new SchedulerWidget( scheduler );
	QDockWidget *dock = new QDockWidget("Scheduler");
	dock->setWidget( scheduler->widget );
	QMainWindow * win = (QMainWindow *) qApp->activeWindow();
	win->addDockWidget(Qt::BottomDockWidgetArea, dock);

	this->connect( scheduler, SIGNAL(startBlend()), SLOT(executeBlend()) );
}

void TopoBlender::executeBlend()
{
    /// STEP 4) Execute the tasks
    QtConcurrent::run( scheduler, &Scheduler::executeAll ); // scheduler->executeAll();
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
		ctnode->id = snodeID + "_null";

		super_tg->addNode(ctnode);
		superNodeCorr[snodeID] = ctnode->id;
	}

	// To grow:
	foreach(QString tnodeID, gcoor->nonCorresTarget())
	{
		Structure::Node *tnode = super_tg->getNode(tnodeID);
		Structure::Node *csnode = tnode->clone();
		csnode->id = tnodeID + "_null";
		super_sg->addNode(csnode);

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
			super_tg->addGroup(ctnodeIDs);
		}

		// 1-to-N
		if (tN > 1)	{
			QVector<QString> csnodeIDs = cloneGraphNode(super_sg, snode->id, tN);
			for (int i = 0; i < tN; i++) superNodeCorr[csnodeIDs[i]] = tNodes[i];
			super_tg->addGroup(csnodeIDs);

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

void TopoBlender::correspondTwoEdges( Structure::Link *slink, Structure::Link *tlink, bool isFlip, Structure::Graph* source )
{
	if(slink->property.contains("correspond") || tlink->property.contains("correspond")) return;

	// Force flip order of nodes of link on super_sg
	if (isFlip)
	{
		QString n1 = slink->n1->id;
		QString n2 = slink->n2->id;
		Array1D_Vec4d c1 = slink->coord.front();
		Array1D_Vec4d c2 = slink->coord.back();

		source->removeEdge(n1,n2);
		slink = source->addEdge(source->getNode(n2), source->getNode(n1), c2, c1, source->linkName(n2,n1) );
	}

	slink->property["correspond"] = tlink->id;
	tlink->property["correspond"] = slink->id;
}

// This function could be written with [graphA] and [graphB] but we should 
// keep it this way to ensure correspondence happen together, once and at same time
void TopoBlender::correspondSuperEdges()
{
	bool CASE_1 = true;
	bool CASE_2 = true;
	bool CASE_3 = true;
	bool CASE_4 = true;
	bool CASE_5 = true;
	bool CASE_6 = true;

	/// Correspond trivial edges, i.e. both nodes exist on both graphs
	if( CASE_1 )
	{
		correspondTrivialEdges ( super_sg, super_tg );
		correspondTrivialEdges ( super_tg, super_sg );
	}

	/// Correspond edges between two [extra] or two [missing] nodes
	if( CASE_2 )
	{
		correspondSimilarType( super_sg, super_tg );
		correspondSimilarType( super_tg, super_sg );
	}
	
	/// Creating one edge for floating null nodes
	if (CASE_3)
	{
		connectNullNodes( super_sg, super_tg );
		connectNullNodes( super_tg, super_sg );
	}

	/// Correspond edges with changed ends
	if( CASE_4 )
	{
		correspondChangedEnds( super_sg, super_tg );
		correspondChangedEnds( super_tg, super_sg );
	}

	/// Do remaining edges
	if( CASE_5 )
	{
		remainingUncorrespondedEdges( super_sg, super_tg );
		remainingUncorrespondedEdges( super_tg, super_sg );
	}

	/// Remove excess edges (edges of core nodes still uncorresponded)
	if( CASE_6 )

	{
		removeRedundantEdges( super_sg );
		removeRedundantEdges( super_tg );
	}
	
	// Visualization: assign global unique ids
	int uid = 0;
	foreach(Structure::Link * slink, super_sg->edges){
		if(!slink->property.contains("correspond")) continue;
		Link* tlink = super_tg->getEdge(slink->property["correspond"].toString());
		if(!tlink) continue;
		slink->property["uid"] = tlink->property["uid"] = uid++;
	}
}

void TopoBlender::removeRedundantEdges( Structure::Graph * source )
{
	foreach(Structure::Node * snode, source->nodes)
	{
		if (snode->id.contains("null")) continue;

		foreach(Link* link, edgesNotContain( source->getEdges(snode->id), "correspond" ))
			source->removeEdge(link->n1, link->n2);
	}
}

void TopoBlender::remainingUncorrespondedEdges( Structure::Graph * source, Structure::Graph * target )
{
	foreach(Structure::Node * snode, source->nodes)
	{
		if (!snode->id.contains("null")) continue;
		Structure::Node * tnode = target->getNode( snode->property["correspond"].toString() );

		// Get uncorresponded of target
		QVector<Structure::Link*> tedges = edgesNotContain( target->getEdges(tnode->id), "correspond" );
		if(tedges.isEmpty()) continue;

		foreach(Link * tlink, tedges)
		{
			Link * slink = addMissingLink( source, tlink );

			correspondTwoEdges(slink, tlink, false, source);
		}
	}
}

void TopoBlender::correspondTrivialEdges( Structure::Graph * source, Structure::Graph * target )
{
	foreach(Structure::Link * slink, source->edges)
	{
		Structure::Node *sn1 = slink->n1, *sn2 = slink->n2;
		Structure::Node *tn1 = target->getNode( sn1->property["correspond"].toString() ),
						*tn2 = target->getNode( sn2->property["correspond"].toString() );

		Structure::Link * tlink = target->getEdge(tn1->id, tn2->id);

		if(!tlink) continue;

		bool isFlip = ( tlink->n1->id != tn1->id );
		correspondTwoEdges(slink, tlink, isFlip, source);
	}
}

void TopoBlender::correspondSimilarType( Structure::Graph * source, Structure::Graph * target )
{
	foreach(Structure::Link * slink, source->edges)
	{
		if (slink->property.contains("correspond")) continue;
		 
		Structure::Node *sn1 = slink->n1, *sn2 = slink->n2;
		Structure::Node *tn1 = target->getNode(sn1->property["correspond"].toString()),
			*tn2 = target->getNode(sn2->property["correspond"].toString());

		// We are only looking for missing edges
		if(!(tn1->id.contains("null") && tn2->id.contains("null"))) continue;

		Structure::Link * tlink = addMissingLink(target, slink);
		correspondTwoEdges(slink, tlink, false, source);
	}
}

void TopoBlender::connectNullNodes( Structure::Graph * source, Structure::Graph * target )
{
	foreach(Structure::Node * snode, source->nodes)
	{
		if (!snode->id.contains("null")) continue;

		Structure::Node * tnode = target->getNode( snode->property["correspond"].toString() );

		// Get edges of target node
		QVector<Structure::Link*> tedges = edgesNotContain( target->getEdges(tnode->id), "correspond" );
		if(tedges.isEmpty()) continue;

		if (tedges.size() == 1)
		{
			Link* tlink = tedges.front();
			Link* slink = addMissingLink(source, tlink);
			correspondTwoEdges(slink, tlink, false, source);
		}

		else if (tedges.size() == 2)
		{
			foreach( Link* tlink, tedges)
			{
				Link* slink = addMissingLink(source, tlink);
				correspondTwoEdges(slink, tlink, false, source);
			}
		}

		
		else
		{
			// Collect names, in source, of target's neighboring nodes
			QStringList nodesKeep;
			foreach(Link * tlink, tedges){
				Node* tOther = tlink->otherNode(tnode->id);
				Node* sOther = source->getNode(tOther->property["correspond"].toString());
				nodesKeep << sOther->id;
			}

			// Keep only subgraph with names from above
			Structure::Graph copy(*source);
			foreach(Node*n, source->nodes) if(!nodesKeep.contains(n->id)) copy.removeNode(n->id);
		
			// Find node with most valence, otherwise pick first
			int bestValence = 0;
			QString bestID = copy.nodes.front()->id;
			foreach(Node * n, copy.nodes){
				int valence = copy.valence(n);
				if (valence > bestValence){
					bestID = n->id;
					bestValence = valence;
				}
			}

			// Find corresponding link on target
			Node* bestTNode = target->getNode( source->getNode(bestID)->property["correspond"].toString() );
			Link* bestTLink = target->getEdge( tnode->id, bestTNode->id ); 

			// Add the link to source
			Link* slink = addMissingLink(source, bestTLink);
			correspondTwoEdges(slink, bestTLink, false, source);
		}
	}
}

void TopoBlender::correspondChangedEnds( Structure::Graph * source, Structure::Graph * target )
{
	bool isRunning = true;
	do{
		isRunning = false;

		foreach(Structure::Node * snode, source->nodes)
		{
			Structure::Node * tnode = target->getNode( snode->property["correspond"].toString() );

			QVector<Structure::Link*> sedges = edgesNotContain( source->getEdges(snode->id), "correspond" );
			QVector<Structure::Link*> tedges = edgesNotContain( target->getEdges(tnode->id), "correspond" );


			if( sedges.size() == tedges.size() && !sedges.isEmpty())
			{
				for( int i = 0; i < (int)sedges.size(); i++)
				{
					Link* slink = sedges[i], *tlink = tedges[i];

					bool sFromMe = ( slink->n1->id == snode->id );
					bool tFromMe = ( tlink->n1->id == tnode->id );

					bool isFlip = (sFromMe != tFromMe);

					correspondTwoEdges(slink, tlink, isFlip, source);
				}

				isRunning = true;
			}
		}
	} while (isRunning);
}

void TopoBlender::removeUncorrespondedEdges( Structure::Graph * graph )
{
	QVector<Link*> linksToRemove;

	foreach(Link * link, graph->edges) 
		if( !link->property.contains("correspond") ) linksToRemove << link;

	foreach(Link* link, linksToRemove) 
		graph->removeEdge(link->n1, link->n2);
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

QVector<Structure::Link*> TopoBlender::nonCorrespondEdges( Structure::Node * node, Structure::Graph * graph )
{
	return edgesNotContain(graph->getEdges( node->id ), "correspond");
}

void TopoBlender::generateSuperGraphs()
{
	// Two super graphs have one-to-one correspondence between nodes and edges
	// Two corresponded edge don't have to link two corresponded nodes
	super_sg = new Structure::Graph(*sg);
	super_tg = new Structure::Graph(*tg);

	/// NODES:
	// Correspond nodes in super graphs
	correspondSuperNodes();

	// Equalize resolution for corresponded nodes
	equalizeSuperNodeResolutions();

	// Equalize type for corresponded nodes
	equalizeSuperNodeTypes();

	// Visualization - color similar items
	foreach(Structure::Node * n, super_sg->nodes)
	{
		QString corrId = n->property["correspond"].toString();
		Structure::Node * other = super_tg->getNode(corrId);
		if(other) n->vis_property["color"].setValue( other->vis_property["color"] );
	}

	/// EDGES:
	// Correspond edges in super graphs
	correspondSuperEdges();
	
	postprocessSuperEdges();

	// Zero the geometry for null nodes
	foreach(Structure::Node * snode, super_sg->nodes)
	{
		if (!snode->id.contains("null")) continue;
		snode->setControlPoints( Array1D_Vector3(snode->numCtrlPnts(), Vector3(0)) );
		snode->property["zeroGeometry"] = true;
	}
}

void TopoBlender::postprocessSuperEdges()
{
	// Initial edge radius for all nodes = using actual delta
	foreach(Link * l, super_sg->edges) l->property["delta"].setValue( l->delta() );
	foreach(Link * l, super_tg->edges) l->property["delta"].setValue( l->delta() );

	// Set delta for edges of null nodes, because now we can not determine the 
	// Null-to-Null: use the corresponded real delta
	// Null-to-Real: 
	// >> One real: use the corresponded real delta
	// >> More real: put null at the centoid of relative joint to all real neighbours
	//               and the delta is computed thus from there
	foreach (Node* n, super_sg->nodes){
		if (n->id.contains("null")){
			foreach(Link* sl, super_sg->getEdges( n->id )) {
				Link* tl = super_tg->getEdge(sl->property["correspond"].toString());
				sl->property["delta"].setValue( tl->delta() );
			}
		}
	}
	foreach (Node* n, super_tg->nodes){
		if (n->id.contains("null")){
			foreach(Link* tl, super_tg->getEdges(n->id)) {
				Link* sl = super_sg->getEdge(tl->property["correspond"].toString());
				tl->property["delta"].setValue( sl->delta() );
			}
		}
	}

	// Set blended delta for first time relinking
	foreach(Link * l, super_sg->edges) l->property["blendedDelta"].setValue( l->property["delta"].value<Vec3d>() );
}

void TopoBlender::generateTasks()
{
	foreach(QString snodeID, superNodeCorr.keys())
	{
		QString tnodeID = superNodeCorr[snodeID];

		Task * task;
		
        if(active->getNode(snodeID)->type() == Structure::CURVE)
        {
            if (snodeID.contains("null"))  // Grow
                task = new TaskCurve( active, super_tg, Task::GROW, scheduler->tasks.size() );
            else if (tnodeID.contains("null")) // Shrink
                task = new TaskCurve( active, super_tg, Task::SHRINK, scheduler->tasks.size() );
			else
				task = new TaskCurve( active, super_tg, Task::MORPH, scheduler->tasks.size() );
        }

        if(active->getNode(snodeID)->type() == Structure::SHEET)
        {
            if (snodeID.contains("null"))  // Grow
                task = new TaskSheet( active, super_tg, Task::GROW, scheduler->tasks.size() );
            else if (tnodeID.contains("null")) // Shrink
                task = new TaskSheet( active, super_tg, Task::SHRINK, scheduler->tasks.size() );
            else
                task = new TaskSheet( active, super_tg, Task::MORPH, scheduler->tasks.size() );
        }

		task->setNode( snodeID );
		scheduler->tasks.push_back( task );
	}
}

void TopoBlender::equalizeSuperNodeResolutions()
{
	foreach(QString snodeID, superNodeCorr.keys())
	{
		QString tnodeID = superNodeCorr[snodeID];

		// skip non-corresponded nodes
		if (snodeID.contains("null") || tnodeID.contains("null"))
			continue;

		Structure::Node* snode = super_sg->getNode(snodeID);
		Structure::Node* tnode = super_tg->getNode(tnodeID);

		if (snode->type() == tnode->type())
		{
			if (snode->numCtrlPnts() < tnode->numCtrlPnts())
				snode->equalizeControlPoints(tnode);
			else
				tnode->equalizeControlPoints(snode);
		}
		// One sheet and one curve
		// Sheet has squared resolution of curve
		else
		{
			int sN, tN, betterN;

			if (snode->type() == Structure::SHEET)
			{
				sN = qMax(((Structure::Sheet*) snode)->numUCtrlPnts(), ((Structure::Sheet*) snode)->numVCtrlPnts());
				tN = tnode->numCtrlPnts();
				betterN = qMax(sN, tN);

				snode->refineControlPoints(betterN, betterN);
				tnode->refineControlPoints(betterN);

			}
			else 
			{
				sN = snode->numCtrlPnts();
				tN = qMax(((Structure::Sheet*) tnode)->numUCtrlPnts(), ((Structure::Sheet*) tnode)->numVCtrlPnts());
				betterN = qMax(sN, tN);

				snode->refineControlPoints(betterN);
				tnode->refineControlPoints(betterN, betterN);
			}
		}
	}
}

void TopoBlender::equalizeSuperNodeTypes()
{
	// initial tags and collect the node pairs that need to be converted
	QMap<QString, QString> diffPairs;
	foreach(QString snodeID, superNodeCorr.keys())
	{
		QString tnodeID = superNodeCorr[snodeID];

		Structure::Node* snode = super_sg->getNode(snodeID);
		Structure::Node* tnode = super_tg->getNode(tnodeID);

		bool equalType = true;
		if (snode->type() != tnode->type())
		{
			equalType = false;
			diffPairs[snodeID] = tnodeID;
		}

		snode->property["type_equalized"] = equalType;
		tnode->property["type_equalized"] = equalType;
	}

	// convert sheet to curve if corresponded to curve
	// iteratively determine the orientation and location of the new curves
	while(!diffPairs.isEmpty())
	{
		foreach(QString snodeID, diffPairs.keys())
		{
			QString tnodeID = diffPairs[snodeID];

            Structure::Node* snode = super_sg->getNode(snodeID);

			// try to convert
			bool converted;
			if (snode->type() == Structure::SHEET)
				converted = convertSheetToCurve(snodeID, tnodeID, super_sg, super_tg);
			else
				converted = convertSheetToCurve(tnodeID, snodeID, super_tg, super_sg);

			// check if success
			if (converted)  diffPairs.remove(snodeID);
		}

	}

	// remove tags
	foreach(Structure::Node* n, super_sg->nodes) n->property.remove("type_equalized");
	foreach(Structure::Node* n, super_tg->nodes) n->property.remove("type_equalized");

}

// The sheet and curve should serve the same functionality in each shape
// which implies the curve(s) lay within the BB of the sheet
// some of the curves maintain the same connections to others
// we can relink the new curves (converted sheet) in the same way as the curve
bool TopoBlender::convertSheetToCurve( QString nodeID1, QString nodeID2, Structure::Graph* superG1, Structure::Graph* superG2 )
{
	Structure::Node* node1 = superG1->getNode(nodeID1);
	Structure::Node* node2 = superG2->getNode(nodeID2);

	Structure::Sheet *sheet1 = (Structure::Sheet*)node1;
	Structure::Curve *curve2 = (Structure::Curve*)node2;

	// Find a for-sure link(links with corresponded ends) to determine the new curve skeleton
	bool converted = false;
	foreach(Structure::Link* link2, superG2->getEdges(nodeID2))
	{
		Structure::Node* other2 = link2->otherNode(nodeID2);

		// choose type-equalized neighbours
		if (other2->property["type_equalized"].toBool())
		{
			QString otherID1 = other2->property["correspond"].toString();

			if (!converted)
			{
				Structure::Node* other1 = superG1->getNode(otherID1);
				Vec4d otherCoord = link2->getCoordOther(nodeID2).front();
				Vec3d linkOtherPos1 = other1->position(otherCoord);

				Vec3d direction2 = curve2->direction();
				NURBS::NURBSCurved new_curve = sheet1->convertToNURBSCurve(linkOtherPos1, direction2);

				// create new curve node with the same id
				Structure::Curve *curve1 = new Structure::Curve(new_curve, nodeID1);

				// copy properties
				curve1->property = node1->property;
				curve1->property["original_sheet"].setValue(sheet1);
			
				// replace the sheet with curve
				superG1->removeNode(nodeID1);
				superG1->addNode(curve1);

				// mark the tag
				curve1->property["type_equalized"] = true;
				node2->property["type_equalized"] = true;

				converted = true;
			}

			// relink only the type-equalized neighbours
			// Other neighbors will be relinked in future
			superG1->addEdge(otherID1, nodeID1);
		}
	}

	return converted;
}


Structure::Node * TopoBlender::addMissingNode( Structure::Graph *toGraph, Structure::Graph * fromGraph, Structure::Node * fromNode )
{
    Q_UNUSED(fromGraph);

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
