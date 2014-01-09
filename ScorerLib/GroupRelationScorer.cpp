#include "GroupRelationScorer.h"
#include <algorithm>
#include <numeric>

double GroupRelationScorer::evaluate(QVector<QVector<GroupRelation> > &groupss, QVector<PART_LANDMARK> &corres)
{
	double resultMax(0.0); int num(0), maxId(0);
	for ( int j = 0; j < groupss.size(); ++j)
	{
		QVector<GroupRelation>& groups = groupss[j];

		for ( int i = 0; i < groups.size(); ++i)
		{
			GroupRelation& gr = groups[i];			
			GroupRelation cgr = findCorrespondenceGroup(this->graph_,gr,corres, 0==j);
			cgr.deviation = cgr.deviation - gr.deviation;
			cgr.deviation = cgr.deviation * cgr.diameter / this->normalizeCoef_;
			cgr.deviation = std::max(0.0, cgr.deviation);

			++num;
			if ( resultMax < cgr.deviation)
			{
				resultMax = cgr.deviation;
				maxId = num;
			}					
					
			if (logLevel_>0) // && cgr.deviation > 0
			{
				logStream_ << num << "\n";
				if ( j == 0 )
					logStream_ << "groups in source shape \n";
				else
					logStream_ << "groups in target shape \n";

				logStream_ << gr << "correspond to: \n";
				logStream_ << cgr << "\n";
			}
		}
	}

	if (logLevel_>0 )
	{
		logStream_ << "group: " << maxId << " with max deviation: " << resultMax << ", which leads to score: " << 1/(1+resultMax) << "\n";		
	}
	return 1/(1+resultMax);
}

GroupRelation GroupRelationScorer::findCorrespondenceGroup(Structure::Graph *graph, GroupRelation &gr,QVector<PART_LANDMARK>& corres,bool bSource)
{
    GroupRelation cgr;
    for ( int i = 0; i < (int) gr.ids.size(); ++i)
    {
        std::vector<Structure::Node*> nodes = findNodesInB(gr.ids[i], graph, corres, bSource);
        for ( int j = 0; j < (int) nodes.size(); ++j)
        {
            Structure::Node* n = nodes[j];
            if ( NULL == n) continue;
			cgr.ids.push_back(n->id);
        }
    }
    if ( int(cgr.ids.size()) < 2 )
    {
		cgr.ids.clear();
        return cgr;
    }
    cgr.diameter = computeGroupDiameter(cgr);
    computeGroupDeviationByCpts(cgr, gr);
    return cgr;
}

void GroupRelationScorer::computeGroupAxis(GroupRelation& cgr, QString pairtype)
{	
	int n = cgr.ids.size();
	cgr.direction = Eigen::Vector3d(0.0,0.0,0.0);
	for (int j=0; j<n; ++j)
	{
		Structure::Node* n1 = graph_->getNode(cgr.ids[j]);
		Structure::Node* n2 = graph_->getNode(cgr.ids[(j+1)%n]);

		Vector_3 v1, v2;
		node2direction(n1,v1); node2direction(n2,v2);
        Vector_3 dir = cross_product(v1, v2);
		dir.normalize();
		if ( dir.dot(cgr.direction) < 0)
			cgr.direction = cgr.direction - dir;
		else
			cgr.direction = cgr.direction + dir;
	}
	cgr.direction.normalize();
	return;
}

void GroupRelationScorer::computeGroupDeviationByCpts(GroupRelation& cgr, GroupRelation& gr)
{
    cgr.type = gr.type;
    if ( cgr.type == AXIS_SYMMETRY)
    {
		computeGroupCenter(cgr);	
		computeGroupAxis(cgr, gr.note);
        cgr.deviation = computeAxisSymmetryGroupDeviationSortParts(cgr,pointLevel_);
    }
    else if ( cgr.type == REF_SYMMETRY)
    {
        cgr.deviation = computeRefSymmetryGroupDeviation(cgr,pointLevel_);
    }
}

//double GroupRelationScorer::errorOfCoplanarGroupByCpts(std::vector<Structure::Node*> &nodes, Eigen::Vector3d& point, Eigen::Vector3d& normal)
//{
//    double err(0.0);
//    Point_3 p(point[0],point[1],point[2]);
//    Vector_3 n(normal[0],normal[1],normal[2]);
//    Plane_3 plane(p, n);
//
//    for ( int i = 0; i < (int) nodes.size(); ++i)
//    {
//        Eigen::Vector3d pt1 = nodes[i]->controlPoint(0);
//        int ncp = nodes[i]->numCtrlPnts();
//        Eigen::Vector3d pt2 = nodes[i]->controlPoint( ncp-1 );
//        Point_3 p1(pt1.x(), pt1.y(), pt1.z());
//        Point_3 p2(pt2.x(), pt2.y(), pt2.z());
//
//        double dist1 = squared_distance(plane, p1);
//        double dist2 = squared_distance(plane, p2);
//        err += (dist1+dist2);
//    }
//    return err;
//}