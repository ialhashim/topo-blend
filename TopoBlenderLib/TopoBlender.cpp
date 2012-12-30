#include <set>
#include <QFileSystemModel>
#include <QDockWidget>
#include <QApplication>
#include <QMainWindow>

#include "TopoBlender.h"
using namespace Structure;

#include "ExportDynamicGraph.h"

#include "Scheduler.h"
#include "SchedulerWidget.h"

// Temporary solution for output
#include "surface_mesh/IO.h"


TopoBlender::TopoBlender( Structure::Graph * sourceGraph, Structure::Graph * targetGraph, Scheduler * useScheduler, QObject *parent ) : QObject(parent)
{
    sg = sourceGraph;
    tg = targetGraph;
	scheduler = useScheduler;

	active = new Structure::Graph(*sg);
	
	gcoor = new GraphCorresponder(sg, tg);
	gcoor->computeCorrespondences();

	// Generate shrinking tasks
	foreach(QString nodeID, gcoor->nonCorresSource())
	{
		Task * task = new Task( active, Task::SHRINK );
		task->property["nodeID"] = "Shrink:" + nodeID;
		scheduler->tasks.push_back( task );
	}

	// Generate growing tasks
	foreach(QString nodeID, gcoor->nonCorresTarget())
	{
		Task * task = new Task( active, Task::GROW );
		task->property["nodeID"] = "Grow:" +nodeID;
		scheduler->tasks.push_back( task );
	}


	// Generate morphing tasks
	// 1) Find non-corresponding edges, re-link
	// 2) Generate task
	foreach (SET_PAIR set2set, gcoor->correspondences)
	{
		int sN = set2set.first.size();
		int tN = set2set.second.size();

		Task * task = NULL;
		if (sN == 1 && tN > 1)
		{
			// One to many : splitting
			task = new Task( active, Task::SPLIT );
			task->property["nodeID"] = "One-to-many";
		}
		
		else if (sN > 1 && tN == 1)
		{
			// Many to one : merging
			task = new Task( active, Task::MERGE );
			task->property["nodeID"] = "Many-to-one";
		}

		else if (sN == 1 && tN == 1)
		{
			// One to one : fix links
			// Extract the "core", which nodes and edges are fully corresponded
			task = new Task( active, Task::MORPH );
			task->property["nodeID"] = "Morph!";
		}

		task->property["corr"].setValue(set2set);
		scheduler->tasks.push_back( task );
	}
	 
	scheduler->schedule();

	// Show the scheduler window:
	SchedulerWidget * sw = new SchedulerWidget( scheduler );
	QDockWidget *dock = new QDockWidget("Scheduler");
	dock->setWidget(sw);
	QMainWindow * win = (QMainWindow *) qApp->activeWindow();
	win->addDockWidget(Qt::BottomDockWidgetArea, dock);
}

/*
Graph * TopoBlender::blend()
{
	// Initial steps
	cleanup();
	originalGraphDistance = new GraphDistance(sg);

	/// 1) Best partial correspondence
	bestPartialCorrespondence();

	visualizeActiveGraph(QString("step%1").arg(stepCounter), "Initial graph");

	// Keep track of one-sided links
	QMap< int, int > needLink;
	std::map< int, std::vector<Link> > deadLinks;
	std::vector< int > movingLinks;

	Flags flags;

	// STAGE 1) Check ACTIVE nodes link situation
	while((flags = active.flags("state")).count(DONE) != target.nodes.size())
	{
		QString log;

		// Break if we ran out of 'ACTIVE' nodes
		if(flags.indexOf(ACTIVE,0) < 0) break;

		// Find an active node
		int active_idx = active.nodeIndex("state", ACTIVE);
		if(active_idx < 0) break;

		// Get corresponding target node
		SimpleNode & n_active = active.nodes[active_idx];
		QString activeNodeID = n_active.str("original");
		QString targetNodeID = n_active.str("correspond");
		
		// Get coordinates of all links of both
		QMap< Link*, std::vector<Vec4d> > coord_active, coord_target;
		QList< ScalarLinksPair > corresp = badCorrespondence(activeNodeID, targetNodeID, coord_active, coord_target);

		// Decide either remove links (when target has less links) or add links
		int linkDiff = coord_active.size() - coord_target.size();
		
		// Link abundance:
		if(linkDiff > 0)
		{
			// Find excess links on active node
			for(int i = 0; i < linkDiff; i++)
			{
				// We take away from bottom of sorted list
				Link * link = corresp.takeLast().second.first;
				QString otherNodeID = link->otherNode( activeNodeID )->id;
				int other_idx = active.nodeIndex("original", otherNodeID);

				// Remove the edges
				active.removeEdge(n_active.idx, other_idx);

				// Mark as 'DISCONNECTED' if it is isolated
				if(active.valence(other_idx) == 0)
					active.nodes[other_idx].set("state", DISCONNECTED);

				deadLinks[n_active.idx].push_back(*link);
			}

			// To remaining neighbors, propagate 'ACTIVE' state
			foreach(ScalarLinksPair sp, corresp)
			{
				Link * link = sp.second.first;
				QString otherNodeID = link->otherNode( activeNodeID )->id;
				int other_idx = active.nodeIndex("original", otherNodeID);
				int otherState = active.nodes[other_idx].val("state");

				// Mark as 'ACTIVE'
				if(otherState != DONE && otherState != DISCONNECTED)
					active.nodes[other_idx].set("state", ACTIVE);

				// Set corresponding target node
				Link * linkTarget = sp.second.second;
				QString otherNodeTarget = linkTarget->otherNode(targetNodeID)->id;
				active.nodes[other_idx].set("correspond", otherNodeTarget);
			}

			// Log activity
			log += QString("Removed links from node");
		}

		// link deficiency:
		if(linkDiff < 0)
		{
			// Add special links that need to be filled with [something]
			//	[something] nearby nodes ? 
			needLink[n_active.idx] = abs(linkDiff);

			// propagate 'ACTIVE' state to neighbors
			foreach(ScalarLinksPair sp, corresp)
			{
				Link * link = sp.second.first;
				QString otherNodeID = link->otherNode( activeNodeID )->id;
				int other_idx = active.nodeIndex("original", otherNodeID);
				int otherState = active.nodes[other_idx].val("state");

				// Mark as 'ACTIVE'
				if(otherState != DONE && otherState != DISCONNECTED)
				{
					active.nodes[other_idx].set("state", ACTIVE);

					// Set corresponding target node
					Link * linkTarget = sp.second.second;
					QString otherNodeTarget = linkTarget->otherNode(targetNodeID)->id;
					active.nodes[other_idx].set("correspond", otherNodeTarget);
				}
			}

			log += QString("Need to add links to node [%1]").arg(activeNodeID);
		}

		// Same links, need to check "quality" of these links
		if(linkDiff == 0)
		{
			foreach(ScalarLinksPair sp, corresp)
			{
				Link * link = sp.second.first;
				QString otherNodeID = link->otherNode( activeNodeID )->id;
				int other_idx = active.nodeIndex("original", otherNodeID);
				int otherState = active.nodes[other_idx].val("state");

				// Mark as 'ACTIVE'
				if(otherState != DONE && otherState != DISCONNECTED)
				{
					active.nodes[other_idx].set("state", ACTIVE);

					// Set corresponding target node
					Link * linkTarget = sp.second.second;
					QString otherNodeTarget = linkTarget->otherNode(targetNodeID)->id;
					active.nodes[other_idx].set("correspond", otherNodeTarget);
				}
			}

			log += QString("Same links count");
		}

		// This node is now 'DONE'
		n_active.set("state", DONE);

		visualizeActiveGraph(QString("step%1").arg(stepCounter++), log);
	}

	// STAGE 2) Reconnect for missing links
	foreach(int index, needLink.keys())
	{
		SimpleNode & n_active = active.nodes[index];
		SimpleNode & n_target = *target.getNode(n_active.str("correspond"));

		// Get link differences
		QMap<int, SimpleEdge> adjActive = active.getEdges(n_active.idx);
		QMap<int, SimpleEdge> adjTarget = target.getEdges(n_target.idx);
		foreach(int ei, adjActive.keys()){
			SimpleNode & ni = active.nodes[adjActive[ei].otherNode(n_active.idx)];
			foreach(int ej, adjTarget.keys()){
				SimpleNode & nj = target.nodes[adjTarget[ej].otherNode(n_target.idx)];
				if(ni.property.contains("correspond") && ni.str("correspond") == nj.str("original"))
					adjTarget.remove(ej);
			}
		}

		// Try to connect with dissconnected nodes
		foreach(int edge, adjTarget.keys())
		{
			int n_targetOtherIdx = adjTarget[edge].otherNode(n_target.idx);
			SimpleNode & n_targetOther = target.nodes[n_targetOtherIdx];

			if(target.getEdges(n_targetOtherIdx).size() == 1) 
				continue;

			// Get coordinates on the corresponding target node
			Structure::Link * link = target.getOriginalLink(n_target.str("original"), n_targetOther.str("original"));
			Vec4d coordinate = link->getCoord(n_target.str("original")).front();

			// Get position on active
			Vector3 linkPosition = active.mGraph->getNode(n_active.str("original"))->position(coordinate);

			// Find all distances from link point
			originalGraphDistance->computeDistances(linkPosition, 0.1);

			// Store computed distances
			QMap< double, std::pair<int,Vec4d> > dists;

			// First: dissconnected parts
			std::vector<int> dissconnected = active.nodesWith("state", DISCONNECTED);
			if( dissconnected.size() )
			{
				foreach(int d, dissconnected)
				{
					Structure::Node * n = active.mGraph->getNode(active.nodes[d].str("original"));
				
					// Two extremities (based on a coordinate system [0,1])
					double d1 = originalGraphDistance->distanceTo( n->position(n->minCoord()) );
					double d2 = originalGraphDistance->distanceTo( n->position(n->maxCoord()) );

					dists[d1] = std::make_pair(d, n->minCoord());
					dists[d2] = std::make_pair(d, n->maxCoord());
				}

				// Pick closest dissconnected node
				std::pair<int,Vec4d> closestRecord = dists.values().first();
				int closestIdx = closestRecord.first;
				Vec4d closestCoordinate = closestRecord.second;

				SimpleNode & closest = active.nodes[closestIdx];

				// Set corresponding target node & set as active
				closest.set("correspond", n_targetOther.str("original"));
				closest.set("state", ACTIVE);

				// Add a new edge
				int edgeID = active.addEdge(index, closest.idx);
				movingLinks.push_back( edgeID );

				// Handle new link coordinates
				active.specialCoords[edgeID][index] = std::vector<Vec4d>(1,coordinate);
				active.specialCoords[edgeID][closest.idx] = std::vector<Vec4d>(1,closestCoordinate);
				active.movable[edgeID] = closestIdx;

				needLink[index]--;

				// Log event
				QString log = QString("connected node [%1] with [%2]").arg(active.nodes[index].str(
					"original")).arg(active.nodes[closest.idx].str("original"));
				visualizeActiveGraph(QString("step%1").arg(stepCounter++), log);
			}
			else
			{
				// Second: clone parts case

				// Find node similar to n_targetOther in active
				foreach(SimpleEdge edge, active.getEdges(index))
				{
					int other_idx = edge.otherNode(index);
					SimpleNode & n_other = active.nodes[other_idx];

					if(active.nodeType(other_idx) != target.nodeType(n_targetOther.idx)) continue;

					Structure::Node * n = active.mGraph->getNode(active.nodes[other_idx].str("original"));

					// Two extremities (based on a coordinate system [0,1])
					double d1 = originalGraphDistance->distanceTo( n->position(n->minCoord()) );
					double d2 = originalGraphDistance->distanceTo( n->position(n->maxCoord()) );

					dists[d1] = std::make_pair(other_idx, n->minCoord());
					dists[d2] = std::make_pair(other_idx, n->maxCoord());
				}
				if(dists.size() < 1) continue;

				// Found best candidate for a clone
				std::pair<int,Vec4d> bestRecord = dists.values().first();
				int bestIdx = bestRecord.first;
				Vec4d bestCoordinate = bestRecord.second;

				SimpleNode & candidate = active.nodes[bestIdx];

				// Clone if no corresponding already exists
				if(active.nodesWith("correspond",n_targetOther.str("original")).size()) continue;

				int new_idx = active.cloneNode( candidate.idx );
				SimpleNode & cloned = active.nodes[new_idx];

				// Set corresponding target node & set as active
				cloned.set("correspond", n_targetOther.str("original"));
				cloned.set("state", ACTIVE);

				// Add a new edge
				int edgeID = active.addEdge(index, cloned.idx);
				movingLinks.push_back( edgeID );

				// Handle new link coordinates
				active.specialCoords[edgeID][index] = std::vector<Vec4d>(1,coordinate);
				active.specialCoords[edgeID][cloned.idx] = std::vector<Vec4d>(1,bestCoordinate);
				active.movable[edgeID] = new_idx;

				needLink[index]--;

				visualizeActiveGraph(QString("step%1").arg(stepCounter++), 
					QString("Cloned node [%1]").arg(cloned.str("original")));
			}
		}
	}

	// STAGE 3) Fix missing links for newly connected nodes
	while(active.flags("state").count(ACTIVE))
	{
		// Find an active node
		int active_idx = active.nodeIndex("state", ACTIVE);
		if(active_idx < 0) break;
		SimpleNode & n_active = active.nodes[active_idx];
		QString activeNodeID = n_active.str("original");

		// Only worry about parts we know correspond
		if(!n_active.has("correspond")) break;
		QString targetNodeID = n_active.str("correspond");
		SimpleNode & n_target = target.nodes[target.nodeIndex("original",targetNodeID)];

		// Apply links from corresponding target node
		QMap<int, SimpleEdge> edges = target.getEdges(n_target.idx);
		foreach(int ei, edges.keys())
		{
			QString targetOtherID = target.nodes[edges[ei].otherNode(n_target.idx)].str("original");
			
			int other_idx = active.nodeIndex("correspond",targetOtherID);

			// Check:
			if(other_idx < 0 || active.hasEdge(active_idx, other_idx)) continue;

			// Add a new edge
			int edgeID = active.addEdge(active_idx, other_idx);
			movingLinks.push_back( edgeID );

			// Handle new link coordinates
			Link * targetLink = target.mGraph->getEdge(targetNodeID, targetOtherID);
			Vec4d otherCoordinate = targetLink->getCoord(targetOtherID).front();
			Vec4d activeCoordinate = inverseCoord(active.firstSpecialCoord(active_idx).front()); // expected other end from before
			
			active.specialCoords[edgeID][active_idx] = std::vector<Vec4d>(1,activeCoordinate);
			active.specialCoords[edgeID][other_idx] = std::vector<Vec4d>(1,otherCoordinate);
			active.movable[edgeID] = active_idx;
			
			needLink[other_idx]--;

			// Log event
			QString log = QString("connected node [%1] with [%2]").arg(active.nodes[active_idx].str(
				"original")).arg(active.nodes[other_idx].str("original"));
			visualizeActiveGraph(QString("step%1").arg(stepCounter++), log);
		}

		// Set corresponding target node
		n_active.set("state", DONE);
	}

	// Remove resolved links so far
	foreach(int index, needLink.keys()){
		if(needLink[index] == 0) 
			needLink.remove(index);
	}

	// STAGE 4) Grow new nodes if needed
	foreach(int index, needLink.keys())
	{
		if(needLink[index] == 0) continue;

		SimpleNode & n_active = active.nodes[index];
		SimpleNode & n_target = *target.getNode(n_active.str("correspond"));

		QString targetNodeID = n_target.str("original");

		// Get link differences
		QMap<int, SimpleEdge> adjActive = active.getEdges(n_active.idx);
		QMap<int, SimpleEdge> adjTarget = target.getEdges(n_target.idx);
		foreach(int ei, adjActive.keys()){
			SimpleNode & ni = active.nodes[adjActive[ei].otherNode(n_active.idx)];
			foreach(int ej, adjTarget.keys()){
				SimpleNode & nj = target.nodes[adjTarget[ej].otherNode(n_target.idx)];
				if(ni.property.contains("correspond") && ni.str("correspond") == nj.str("original"))
					adjTarget.remove(ej);
			}
		}
		
		// For each missing link
		foreach(int ei, adjTarget.keys())
		{
			SimpleEdge & edge = adjTarget[ei];
			SimpleNode & missing = target.nodes[edge.otherNode(n_target.idx)];
			Properties missingProperty = missing.property;
			QString missingID = missing.str("original");
			QSet<int> missingNeighbors = target.adjNodes(missing.idx);
			
			if(missingNeighbors.size() == 1)
			{
				// Default parameter
				double shrinkFactorCurve = 1e-5;

				// CASE 1) Missing node branches out with no other connections
				Link & l = *target.mGraph->getEdge(missingID, targetNodeID);

				// Add node to dynamic graph
				SimpleNode & newNode = active.nodes[active.addNode(missingProperty)];

				// Set correspondence and state
				newNode.set("correspond",missingID);
				newNode.set("state", DONE);
				int active_idx = newNode.idx;

				int other_idx = active.nodeIndex("correspond", targetNodeID);
				int edgeID = active.addEdge(active_idx, other_idx);

				// Curve
				if(target.nodeType(missing.idx) == Structure::CURVE)
				{
					Vec4d coordOnTarget = l.getCoord(targetNodeID).front();
					Vector3 linkPosition = active.mGraph->getNode(n_active.str("original"))->position(coordOnTarget);
					Curve * targetCurve = ((Curve*)target.mGraph->getNode(missingID));

					int cpIDX = targetCurve->controlPointIndexFromCoord(l.getCoordOther(targetNodeID).front());

					// Copy geometry from target curve
					NURBSCurve curveCopy = targetCurve->curve;

					// Place curve
					curveCopy.translate( -targetCurve->position(l.getCoordOther(targetNodeID).front()) );
					curveCopy.translate( linkPosition );

					// Resize to baby size
					curveCopy.scaleInPlace( shrinkFactorCurve, cpIDX );

					// Add node to Structure Graph
					active.mGraph->addNode(new Structure::Curve(curveCopy, missingID));

					// Growing instructions
					active.growingCurves[edgeID] = std::make_pair(active_idx, std::make_pair(cpIDX, 1.0 / shrinkFactorCurve));
				}

				// Sheet
				if(target.nodeType(missing.idx) == Structure::SHEET)
				{
					Sheet * targetSheet = ((Sheet*)target.mGraph->getNode(missingID));
					Sheet * sheet = (Sheet *)active.mGraph->addNode(new Structure::Sheet(targetSheet->surface, missingID));

					Array2D_Vector3 deltas = sheet->foldTo( l.getCoord(missingID), true );

					// Growing instructions
					active.growingSheets[edgeID] = std::make_pair(active_idx, deltas);
				}

				// Copy coordinates from target's link
				active.specialCoords[edgeID][active_idx] = l.getCoord(missingID);
				active.specialCoords[edgeID][other_idx] = l.getCoordOther(missingID);

				// Mark as done
				needLink[index]--;

				// Log event
				QString log = QString("Added new node [%1] that will grow").arg(newNode.str("original"));
				visualizeActiveGraph(QString("step%1").arg(stepCounter++), log);
			}
			else
			{
				// CASE 2) Missing is connected with 'DONE' nodes only

				// Check neighbors status
				bool isNeighborsDone = true;
				foreach(int j, missingNeighbors)
				{
					QString nodeid = target.nodes[j].str("original");
					if(!active.nodesWith("correspond",nodeid).size()){
						isNeighborsDone = false;
						break;
					}
				}
				if(!isNeighborsDone) continue;

				QList<Link> furthermost = target.mGraph->furthermostEdges(missing.str("original"));

				Link l1 = furthermost.front();
				Link l2 = furthermost.back();
				
				Vector3 p1 = l1.position(missingID);
				Vector3 p2 = l2.position(missingID);

				// Get middle point
				std::vector<Vector3> path;

				originalGraphDistance->computeDistances(p1, 0.1);
				originalGraphDistance->distanceTo(p2, path);

				Vector3 midPoint = path[path.size() / 2];

				// Create a single point node
				if(target.nodeType(missing.idx) == Structure::CURVE)
				{
					active.mGraph->addNode(new Structure::Curve(NURBSCurve::createCurve(midPoint,midPoint), missingID));
				}
				if(target.nodeType(missing.idx) == Structure::SHEET)
				{
					// [TODO: correct this]
					active.mGraph->addNode(new Structure::Sheet(NURBSRectangle::createSheet(0,0,midPoint), missingID));
				}

				// Add node to dynamic graph
				SimpleNode & newNode = active.nodes[active.addNode(missingProperty)];

				// Log event
				visualizeActiveGraph(QString("step%1").arg(stepCounter++), 
					QString("Added new node [%1]").arg(newNode.str("original")));

				// Set correspondence and state
				newNode.set("correspond",missingID);
				newNode.set("state", DONE);
				int active_idx = newNode.idx;

				// Add edges
				foreach(Link l, furthermost)
				{
					int other_idx = active.nodeIndex("correspond", l.otherNode(missingID)->id);
					int edgeID = active.addEdge(active_idx, other_idx);
					
					active.specialCoords[edgeID][active_idx] = l.getCoord(missingID);
					active.specialCoords[edgeID][other_idx] = l.getCoordOther(missingID);
					active.movable[edgeID] = active_idx;

					needLink[other_idx]--;

					// Visualize event
					QString log = QString("connected node [%1] with [%2]").arg(active.nodes[active_idx].str(
						"original")).arg(active.nodes[other_idx].str("original"));
					visualizeActiveGraph(QString("step%1").arg(stepCounter++), log);
				}
			}
		}
	}

	// Show final graph & create GIF animation
	visualizeActiveGraph(QString("step%1").arg(stepCounter++), "Final Graph");
	//system(qPrintable( QString("convert -resize 800x800	-delay %1 -loop 0 *.png steps.gif").arg( 200 ) ));

    return active.toStructureGraph();
}
*/

void TopoBlender::materializeInBetween( Graph * graph, double t, Graph * sourceGraph )
{
	// Create morph animation:
	int NUM_STEPS = params["NUM_STEPS"].toInt();
	int NUM_SMOOTH_ITR = 3;

	std::vector< std::pair< Link, QVariant > > specialLinks;

	// Create paths on structural geometry
	foreach(Link link, graph->edges)
	{
		// Only consider special links with movable nodes
		if(!link.link_property.contains("special"))
			continue;

		if(link.link_property.contains("movable"))
		{
			QString movableNode = link.link_property["movable"].toString();

			Vector3 destination = link.position(movableNode);
			Vector3 source = link.position( link.otherNode(movableNode)->id );

			std::vector<Vector3> path;

			GraphDistance gd(sourceGraph);
			gd.computeDistances(source, 0.05);
			gd.distanceTo(destination, path);

			QVariant pathVar; pathVar.setValue(path);
			specialLinks.push_back( std::make_pair(link, pathVar) );

			// DEBUG:
			debugLines.push_back(PairVector3(source, destination));
			debugPoints.clear();
			foreach(Vector3 p, path) debugPoints.push_back(p);
		}

		if(link.link_property.contains("growing"))
		{
			QString growingNode = link.link_property["growing"].toString();

			Node * n = link.getNode(growingNode);
			QVariant growInfo;

			if(n->type() == Structure::CURVE)
			{
				Curve * curve = (Curve *) n;
				growInfo.setValue(curve->curve.mCtrlPoint);
			}

			if(n->type() == Structure::SHEET)
            {
				growInfo = link.link_property["deltas"];
			}

			specialLinks.push_back( std::make_pair( link, growInfo) );
		}
	}
	
	// Apply motion
	for(int s = 0; s <= NUM_STEPS; s++)
	{
		double stepTime = double(s) / NUM_STEPS;

		for(int i = 0; i < (int) specialLinks.size(); i++)
		{
			Link link = specialLinks[i].first;

			if(link.link_property.contains("movable"))
			{
				std::vector<Vector3> path = specialLinks[i].second.value< Array1D_Vector3 >();
				QString movableNode = link.link_property["movable"].toString();
				Node * n = link.getNode(movableNode);

				if(n->type() == Structure::CURVE)
				{
					Curve * curve = (Curve *) n;

					// Pick up closest control point on curve
					int cpIdx = curve->controlPointIndexFromCoord( link.getCoord(movableNode).front() );	

					// Move the control point to location on path
					int path_idx = stepTime * (path.size() - 1);
					curve->controlPoint(cpIdx) = path[path_idx];

					// Smooth other control points
					std::set<int> a; a.insert(-1); // anchor first and last

					curve->laplacianSmoothControls(NUM_SMOOTH_ITR, a);
				}
			}

			if(link.link_property.contains("growing"))
			{
				QString growingNode = link.link_property["growing"].toString();
				Node * n = link.getNode(growingNode);

				if(n->type() == Structure::CURVE)
				{
					Curve * curve = (Curve *) n;

					Scalar growFactor = link.link_property["growFactor"].toDouble();
					int growAnchor = link.link_property["growAnchor"].toInt();

					std::vector<Vector3> ctrlPoints = specialLinks[i].second.value< Array1D_Vector3 >();
					Vector3 anchor = ctrlPoints[growAnchor];

					for(int j = 0; j < (int)ctrlPoints.size(); j++)
						ctrlPoints[j] = ((ctrlPoints[j] - anchor) * (growFactor * stepTime)) + anchor;
					
					curve->curve.mCtrlPoint = ctrlPoints;
				}

				if(n->type() == Structure::SHEET)
				{
					Sheet * sheet = (Sheet *) n;

					Array2D_Vector3 cpts = link.link_property["cpts"].value<Array2D_Vector3>();
					Array2D_Vector3 deltas = link.link_property["deltas"].value<Array2D_Vector3>();

					for(int u = 0; u < sheet->surface.mNumUCtrlPoints; u++)
					{
						for(int v = 0; v < sheet->surface.mNumVCtrlPoints; v++)
						{
							sheet->surface.mCtrlPoint[u][v] = cpts[u][v] + (deltas[u][v] * stepTime);
						}
					}
				}
			}
		}

		// Synthesize the geometry
		SurfaceMeshModel meshStep;
		graph->materialize(&meshStep, params["materialize"].toDouble());
		//DynamicVoxel::MeanCurvatureFlow(&meshStep, 0.05);
		DynamicVoxel::LaplacianSmoothing(&meshStep);

		// Output this step to mesh file
		QString seq_num; seq_num.sprintf("%02d", s);
		QString fileName = QString("blend_sequence_%1.off").arg(seq_num);
		meshStep.triangulate();
		write_off(meshStep, qPrintable(fileName));
	}
}

void TopoBlender::drawDebug()
{
	glDisable(GL_LIGHTING);

	glColor3d(1,0,0);
	glPointSize(25);
	glBegin(GL_POINTS);
	foreach(Vector3 p, debugPoints) glVector3(p);
	glEnd();

	glColor3d(1,0,0);
	glLineWidth(12);
	glBegin(GL_LINES);
	foreach(PairVector3 line, debugLines){
		glLine(line.first, line.second);
	}
	glEnd();

	glEnable(GL_LIGHTING);
}

void TopoBlender::visualizeActiveGraph( QString caption, QString subcaption )
{
	toGraphML(active, caption);
	toGraphviz(active, caption, true, caption, subcaption);
}

void TopoBlender::cleanup()
{
	// Reset step counter
	stepCounter = 0;

	// Delete resulting graph files etc.
	QDir directory(QDir::currentPath());
	QStringList filesList = directory.entryList(QDir::Files);
	foreach(QString f, filesList){
		if(f.endsWith(".png") || f.endsWith(".gv") || f.endsWith(".graphml")) 
			directory.remove(f);
	}
}

void TopoBlender::testScheduler()
{
	
}

Structure::Graph * TopoBlender::blend()
{
	return NULL;
}
