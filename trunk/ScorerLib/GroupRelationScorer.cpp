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
			if (cgr.deviation < gr.deviation)
				cgr.deviation = 0;
			else
			{
				cgr.deviation = cgr.deviation - gr.deviation;
				cgr.deviation = cgr.deviation * cgr.diameter / this->normalizeCoef_;
			}

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
    std::vector<double> nodeDiameter;
    double diameter, maxDiameter(0.0);
    int maxIdx(0);

    std::vector<Structure::Node*> tmpNodes;
    for ( int i = 0; i < (int) gr.ids.size(); ++i)
    {
        std::vector<Structure::Node*> nodes = findNodesInB(gr.ids[i], graph, corres, bSource);
        for ( int j = 0; j < (int) nodes.size(); ++j)
        {
            Structure::Node* n = nodes[j];
            if ( NULL == n) continue;

            tmpNodes.push_back(n);
			diameter = n->bbox().diagonal().norm();
            nodeDiameter.push_back(diameter);
            if ( diameter > maxDiameter)
            {
                maxDiameter = diameter;
                maxIdx = tmpNodes.size()-1;
            }
        }
    }

    //////////////	use max node as the first node, the following will be transformed into it, & compare with it.
    if ( int(tmpNodes.size()) < 2 )//if ( int(tmpNodes.size()) < gr.ids.size() )
    {
        return cgr;
    }

    for ( int i = maxIdx; i < (int) tmpNodes.size(); ++i)
    {
        Structure::Node* n = tmpNodes[i];
        cgr.ids.push_back(n->id);
    }
    for ( int i = 0; i < (int) maxIdx; ++i)
    {
        Structure::Node* n = tmpNodes[i];
        cgr.ids.push_back(n->id);
    }
    cgr.diameter = computeGroupDiameter(cgr);

    //////////////////////
    computeGroupDeviationByCpts(cgr, gr);
    return cgr;
}

void GroupRelationScorer::computeGroupAxis(GroupRelation& cgr, QString pairtype)
{
	if ( TRANS == pairtype)
	{
		QVector<QString>::iterator it = cgr.ids.begin();
		Structure::Node* n = graph_->getNode(*it);    
		Vector_3 direction(0,0,0);
		node2direction(n, direction);
		++it;
		Vector_3 dc;
		int k(1);
		for (; it != cgr.ids.end(); ++it,++k )
		{
			Structure::Node* nn = graph_->getNode(*it);        

			node2direction(nn, dc);
			if ( dot(direction, dc) > 0)
				direction = direction + dc;
			else
				direction = direction - dc;

			direction = direction/sqrt(direction.squaredNorm());
		}
    
		cgr.direction = direction;
		return;
	}

	// for RES
	double dotVal1(0.0);
    Structure::Node* n1 = graph_->getNode(cgr.ids[0]);
    Structure::Node* n2 = graph_->getNode(cgr.ids[1]);
    Vector_3 v1, v2;
    node2direction(n1,v1); node2direction(n2,v2);
    dotVal1 = std::abs(dot(v1,v2));
    if ( dotVal1 > thAxisDeviationRadio_)
    {
        if ( dot(v1,v2) < 0)
            cgr.direction = v1 -v2;
        else
            cgr.direction = v1+v2;
    }
    else
    {
        cgr.direction = cross_product(v1, v2);
    }
    cgr.direction = cgr.direction/sqrt(cgr.direction.squaredNorm());

    for (int j=2; j<cgr.ids.size(); ++j)
    {     
		Structure::Node* n = graph_->getNode(cgr.ids[j]);
		Vector_3 dir;
        node2direction(n,dir);
        dir = dir/sqrt(dir.squaredNorm());

        double tmp = std::abs( dot(cgr.direction, dir) );
        if ( tmp > thAxisDeviationRadio_)
        {
            if ( dot(cgr.direction, dir) < 0)
                cgr.direction = cgr.direction - dir;
            else
                cgr.direction = cgr.direction + dir;
            cgr.direction = cgr.direction/sqrt(cgr.direction.squaredNorm());

        }
	}
	return;
}

void GroupRelationScorer::computeGroupDeviationByCpts(GroupRelation& cgr, GroupRelation& gr)
{
    cgr.type = gr.type;
    if ( cgr.type == AXIS_SYMMETRY)
    {
		computeGroupCenter(cgr);	
		computeGroupAxis(cgr, gr.note);
        cgr.deviation = computeAxisSymmetryGroupDeviation(cgr,pointLevel_);
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