#include "GroupRelationScorer.h"
#include <algorithm>
#include <numeric>

double GroupRelationScorer::evaluate(QVector<QVector<GroupRelation> > &groupss, QVector<PART_LANDMARK> &corres)
{
	double resultMax(0.0); int num(0);
	double dist = graph_->bbox().diagonal().norm();

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
				cgr.deviation = cgr.deviation * cgr.diameter / dist;
			}

			++num;
			if ( resultMax < cgr.deviation)
			{
				resultMax = cgr.deviation;
			}					
					
			if (logLevel_>0 && cgr.deviation > 0) //
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
		logStream_ << "max score: " << 1-resultMax << "\n";		
	}
	return 1-resultMax;
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
            nodeDiameter.push_back(diameter);
            if ( diameter > maxDiameter)
            {
                maxDiameter = diameter;
                maxIdx = tmpNodes.size()-1;
            }
        }
    }

    //////////////	use max node as the first node, the following will be transformed into it, & compare with it.
    if ( tmpNodes.size()<2 )
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

void GroupRelationScorer::computeGroupDeviationByCpts(GroupRelation& cgr, GroupRelation& gr)
{
    cgr.type = gr.type;
    if ( cgr.type == AXIS_SYMMETRY)
    {
		// jjcao todo if from trans, it is easy
		//computeTransGroupInfo(cgr, ids);
		// jjcao todo if from ref, how to compute direction?
        cgr.deviation = computeAxisSymmetryGroupDeviation(cgr,pointLevel_);
    }
    else if ( cgr.type == REF_SYMMETRY)
    {
        cgr.deviation = computeRefSymmetryGroupDeviation(cgr,pointLevel_);
    }
    else if ( cgr.type == COPLANAR)
    {
        std::vector<Structure::Node*> cgrNodes;
        Structure::Node* n = graph_->getNode( cgr.ids[0]);
        cgrNodes.push_back(n);
        Eigen::Vector3d d0 = curve2vectorNormalized(n);
        Line_3 l0 = curve2line(n);
        std::vector<Eigen::Vector3d> pts, normals;

        for ( int j = 1; j < (int) cgr.ids.size(); ++j)
        {
            n = graph_->getNode( cgr.ids[j]);
            Eigen::Vector3d d1 = curve2vectorNormalized(n);
            double error = errorOfParallel(d0, d1);
            if (error < thParal_)
            {
                Line_3 l1 = curve2line(n);
                error = squared_distance(l0, l1);
                if ( error < 0.001) // they are the same line
                    continue;
            }
            createPlane( cgrNodes[0], n, cgr.point, cgr.normal);
            pts.push_back(cgr.point);
            normals.push_back(cgr.normal);

            cgrNodes.push_back( n );
        }
        if ( pts.empty() )
            cgr.deviation = 0;
        else
        {
            cgr.point = Eigen::Vector3d::Zero();
            cgr.normal = Eigen::Vector3d::Ones();
            for ( int i = 1; i < (int) pts.size(); ++i)
            {
                Eigen::Vector3d& nl = normals[i];
                cgr.point = cgr.point + pts[i];
                if ( cgr.normal.dot(nl) > 0)
                    cgr.normal = cgr.normal + nl;
                else
                    cgr.normal = cgr.normal - nl;
            }
            cgr.point = cgr.point/pts.size();
            //cgr.normal=gr.normal/pts.size();
            cgr.normal.normalize();
            cgr.deviation = errorOfCoplanarGroupByCpts(cgrNodes, cgr.point, cgr.normal);
        }
        //if ( cgr.ids[0] == "BarLeft")
        //{
        //	logStream_ << "\n\n\n";
        //
        //	Vector_3 normal = cgr.plane.orthogonal_vector();
        //	logStream_ << "normal of plane: " << normal.x() << ", " << normal.y() << ", " << normal.z() << "\n";
        //}
    }

}

double GroupRelationScorer::errorOfCoplanarGroupByCpts(std::vector<Structure::Node*> &nodes, Eigen::Vector3d& point, Eigen::Vector3d& normal)
{
    double err(0.0);
    Point_3 p(point[0],point[1],point[2]);
    Vector_3 n(normal[0],normal[1],normal[2]);
    Plane_3 plane(p, n);

    for ( int i = 0; i < (int) nodes.size(); ++i)
    {
        Eigen::Vector3d pt1 = nodes[i]->controlPoint(0);
        int ncp = nodes[i]->numCtrlPnts();
        Eigen::Vector3d pt2 = nodes[i]->controlPoint( ncp-1 );
        Point_3 p1(pt1.x(), pt1.y(), pt1.z());
        Point_3 p2(pt2.x(), pt2.y(), pt2.z());

        double dist1 = squared_distance(plane, p1);
        double dist2 = squared_distance(plane, p2);
        err += (dist1+dist2);
    }
    return err;
}