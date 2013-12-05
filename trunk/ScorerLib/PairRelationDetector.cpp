#include "PairRelationDetector.h"
#pragma warning(disable:4100 4101)

void PairRelationDetector::pushToOtherPairs(QVector<PairRelation>& prs, QString type)
{
	for (QVector<PairRelation>::iterator it = prs.begin(); it != prs.end(); ++it)
	{
		if ( it->tag)
		{
			it->type = type;
			otherPairs_.push_back(*it);
		}
	}
}
double PairRelationDetector::ConnectedPairModifier::operator() (Structure::Node *n1, Eigen::MatrixXd& m1, Structure::Node *n2, Eigen::MatrixXd& m2)
{
	double min_dist(-1.0), mean_dist, max_dist;
	if ( n1->id == n2->id)
		return min_dist;
	
	distanceBetween(m1, m2, min_dist, mean_dist, max_dist);
					
	min_dist = min_dist/graphDiameter_;
	if ( min_dist > maxAllowDeviation_)
	{
		maxAllowDeviation_ = min_dist;
	}

	return min_dist;
}
double PairRelationDetector::TransPairModifier::operator() (Structure::Node *n1, Eigen::MatrixXd& m1, Structure::Node *n2, Eigen::MatrixXd& m2)
{
	double deviation(-1.0), min_dist, mean_dist, max_dist;
	if ( n1->id == n2->id)
		return deviation;
	
	
	Eigen::Vector3d transVec = n1->center() - n2->center();
	Eigen::Matrix4d transMat = create_translation3d(transVec);
    Eigen::MatrixXd newverts2 = transform_point3d(m2, transMat);
    
	distanceBetween(m1, newverts2, min_dist, mean_dist, max_dist);

	deviation = 2 * mean_dist / (n1->bbox().diagonal().norm() + n2->bbox().diagonal().norm());
	if ( deviation > maxAllowDeviation_)
	{
		maxAllowDeviation_ = deviation;
	}

	return deviation;
}

double PairRelationDetector::RefPairModifier::operator() (Structure::Node *n1, Eigen::MatrixXd& m1, Structure::Node *n2, Eigen::MatrixXd& m2)
{
	double deviation(-1.0), min_dist, mean_dist, max_dist;
	if ( n1->id == n2->id)
		return deviation;
	
	
	Eigen::Vector3d refCenter = (n1->center() + n2->center())*0.5;
	Eigen::Vector3d refNormal = n1->center() - n2->center();
	refNormal.normalize();

    Eigen::MatrixXd newverts2;
    reflect_points3d(m2, refCenter, refNormal, newverts2);

	distanceBetween(m1, newverts2, min_dist, mean_dist, max_dist);
	deviation = 2 * mean_dist / (n1->bbox().diagonal().norm() + n2->bbox().diagonal().norm());

	
	if ( deviation > maxAllowDeviation_)
	{
		maxAllowDeviation_ = deviation;
	}

	return deviation;
}

double PairRelationDetector::ParallelPairModifier::operator() (Structure::Node *n1, Eigen::MatrixXd& m1, Structure::Node *n2, Eigen::MatrixXd& m2)
{
	double deviation(-1.0);
	if ( n1->id == n2->id)
		return deviation;
	
	
    Vector_3 d1, d2;
    bool iscurve1 = node2direction(n1, d1);
    bool iscurve2 = node2direction(n2, d2);

    if ( iscurve1 != iscurve2 )
		return deviation;

	deviation = errorOfParallel(d1, d2);
	
	if ( deviation > maxAllowDeviation_)
	{
		maxAllowDeviation_ = deviation;
	}

	return deviation;
}

double PairRelationDetector::OrthogonalPairModifier::operator() (Structure::Node *n1, Eigen::MatrixXd& m1, Structure::Node *n2, Eigen::MatrixXd& m2)
{
	double deviation(-1.0);
	if ( n1->id == n2->id)
		return deviation;
	
	
    Vector_3 d1, d2;
    bool iscurve1 = node2direction(n1, d1);
    bool iscurve2 = node2direction(n2, d2);

    if ( iscurve1 == iscurve2 )
	{
		deviation = errorOfOrthogonal(d1, d2);
	}
	else
	{
		deviation = errorOfParallel(d1, d2);
	}
	
	if ( deviation > maxAllowDeviation_)
	{
		maxAllowDeviation_ = deviation;
	}

	return deviation;
}
double PairRelationDetector::CoplanarPairModifier::operator() (Structure::Node *n1, Eigen::MatrixXd& m1, Structure::Node *n2, Eigen::MatrixXd& m2)
{
	double deviation(-1.0);
	if ( n1->id == n2->id)
		return deviation;
	
	
    Vector_3 d1, d2;
    bool iscurve1 = node2direction(n1, d1);
    bool iscurve2 = node2direction(n2, d2);

    if ( iscurve1 == iscurve2 )
	{
		if ( iscurve1 && iscurve2 )
		{
			Line_3 l1 = curve2line(n1), l2 = curve2line(n2);
            deviation = squared_distance(l1, l2);
		}
	}
	else
	{
		Vector3 point, normal;
        if ( iscurve1 )
        {
            sheet2plane(dynamic_cast<Structure::Sheet *>(n2), point, normal);
            Segment_3 cs = curve2segment(n1);
            deviation = errorOfLineInPlane( cs, point, normal);
        }
        else
        {
            sheet2plane(dynamic_cast<Structure::Sheet *>(n1), point, normal);
            Segment_3 cs = curve2segment(n2);
            deviation = errorOfLineInPlane( cs, point, normal);
        }
	}
	
	if ( deviation > maxAllowDeviation_)
	{
		maxAllowDeviation_ = deviation;
	}

	return deviation;
}
//////////////////////////////////////////////////////
PairRelationDetector::PairRelationDetector(Structure::Graph* g, int ith, double normalizeCoef, int logLevel)
					 :RelationDetector(g, "PairRelationDetector-", ith, normalizeCoef, 1, logLevel)
{
	bSource_ = ith == 0;

	for ( int i = 0; i < (int) graph_->nodes.size(); ++i)
	{
		Structure::Node * n = graph_->nodes[i];

		std::vector<Eigen::Vector3d> nodeCptsV;
		extractCpts( n, nodeCptsV, pointLevel_);

		Eigen::MatrixXd nodeCptsM;
		vectorPts2MatrixPts(nodeCptsV, nodeCptsM);
		nodesCpts_.push_back(nodeCptsM);

		nodesCenter_.push_back( Eigen::Vector3d(nodeCptsM.col(0).mean(), nodeCptsM.col(1).mean(), nodeCptsM.col(2).mean()) );
		nodesDiameter_.push_back(n->bbox().diagonal().norm());
	}
}

void PairRelationDetector::detectConnectedPairs(Structure::Graph* g, QVector<PART_LANDMARK> &corres)
{
	QVector<PairRelation> pairs;	

	int tmp1 = graph_->edges.size();
	for ( int i = 0; i < tmp1; ++i)
	{
		Structure::Link* link = graph_->edges[i];
		SurfaceMesh::Vector3 p1 = link->position(link->n1->id);
		SurfaceMesh::Vector3 p2 = link->position(link->n2->id);
		
		PairRelation prb(link->n1,link->n2);
		prb.deviation = (p1-p2).norm()/normalizeCoef_;

		pairs.push_back(prb);
	}

	//int tmp1 = nodesPts_.size()-1;
	//for ( int i = 0; i < tmp1; ++i)
	//{
	//	Eigen::MatrixXd ptsi = nodesPts_[i];
	//	for ( int j = i+1; j < tmp1+1; ++j)
	//	{
	//		Eigen::MatrixXd ptsj = nodesPts_[j];
	//		double tmpd = distanceBetween(ptsi, ptsj)/normalizeCoef_;
	//		if (tmpd < thIntersectDist_)
	//		{
	//			PairRelation prb;
	//			prb.n1 = graph_->nodes[i];
	//			prb.n2 = graph_->nodes[j];
	//			prb.miniDist = tmpd;
	//			pairs_.push_back(prb);
	//		}
	//	}
	//}

	modifyPairsDegree(pairs, ConnectedPairModifier(normalizeCoef_, pointLevel_), g, corres, "Connected pairs");

	connectedPairs_.clear();
	for (QVector<PairRelation>::iterator it = pairs.begin(); it != pairs.end(); ++it)
	{
		if (it->tag)
		{
			it->type = CONNECTED;
			connectedPairs_.push_back(*it);		
		}
	}
}
void PairRelationDetector::detectOtherPairs(Structure::Graph* g, QVector<PART_LANDMARK> &corres)
{
	transPairs_.clear();    refPairs_.clear();
	parallelPairs_.clear();	orthogonalPairs_.clear();
	coplanarPairs_.clear(); otherPairs_.clear();

    int nNodes = graph_->nodes.size();
    for (int i=0; i<nNodes; ++i)
    {
		Structure::Node *n1 = graph_->nodes[i];
        double diam1 = n1->bbox().diagonal().norm();        
        for (int j=i+1; j<nNodes; ++j)
        {
			Structure::Node *n2 = graph_->nodes[j];
            double diam2 = n2->bbox().diagonal().norm();

            bool hasTrans(false),hasRef(false);
            // compare radius // whether 2 part could be symmetry coarsely
			if ( n1->type() == n2->type() )
			{
				if ( (diam1/diam2) < thRadiusRadio_ && (diam2/diam1) < thRadiusRadio_ )
				{
					hasTrans = has_trans_relation(i,j);//
					if ( !hasTrans)
						hasRef = has_ref_relation(i, j);
				}
			}
            if ( !hasTrans && !hasRef )
            {
                isParalOrthoCoplanar(i,j);
            }
        }
    }

	//
	modifyPairsDegree(transPairs_, TransPairModifier(pointLevel_), g, corres, "Trans pairs");
	modifyPairsDegree(refPairs_, RefPairModifier(pointLevel_), g, corres, "Reflection pairs");
	modifyPairsDegree(parallelPairs_, ParallelPairModifier(pointLevel_), g, corres,"Parallel pairs");
	modifyPairsDegree(orthogonalPairs_, OrthogonalPairModifier(pointLevel_), g, corres, "Orthogonal pairs");
	modifyPairsDegree(coplanarPairs_, CoplanarPairModifier(pointLevel_), g, corres, "Coplanar pairs");

	otherPairs_.clear();
	pushToOtherPairs(transPairs_, TRANS);
	pushToOtherPairs(refPairs_, REF);
	pushToOtherPairs(parallelPairs_, PARALLEL);
	pushToOtherPairs(orthogonalPairs_, ORTHOGONAL);
	pushToOtherPairs(coplanarPairs_, COPLANAR);
}

void PairRelationDetector::detect(Structure::Graph* g, QVector<PART_LANDMARK> &corres)
{
	detectConnectedPairs(g, corres);
	detectOtherPairs(g,corres);
}

bool PairRelationDetector::has_trans_relation(int id1, int id2)
{
    Eigen::Vector3d transVec = nodesCenter_[id1] - nodesCenter_[id2];
	Eigen::Matrix4d transMat = create_translation3d(transVec);
    Eigen::MatrixXd newverts2 = transform_point3d(nodesCpts_[id2], transMat);
    
    double min_dist, mean_dist, max_dist;
	distanceBetween(nodesCpts_[id1], newverts2, min_dist, mean_dist, max_dist);

    double error = 2 * mean_dist / (nodesDiameter_[id1] + nodesDiameter_[id2]);
	double nerror = fixDeviationByPartName(graph_->nodes[id1]->id, graph_->nodes[id2]->id, error);

    if (nerror < thTransRadio_)
    {
        PairRelation pr(graph_->nodes[id1], graph_->nodes[id2]);
        pr.trans_vec = transVec;
        pr.diameter = computePairDiameter(pr);
        pr.deviation = error;
        transPairs_.push_back(pr);
        return true;
    }
    else
        return false;
}

bool PairRelationDetector::has_ref_relation(int id1, int id2)
{
	Eigen::Vector3d refCenter = (nodesCenter_[id1] + nodesCenter_[id2])*0.5;
	Eigen::Vector3d refNormal = nodesCenter_[id1] - nodesCenter_[id2];
    refNormal.normalize();

    Eigen::MatrixXd newverts2;
    reflect_points3d(nodesCpts_[id2], refCenter, refNormal, newverts2);

	double min_dist, mean_dist, max_dist;
	distanceBetween(nodesCpts_[id1], newverts2, min_dist, mean_dist, max_dist);
	double error = 2 * mean_dist / (nodesDiameter_[id1] + nodesDiameter_[id2]);
	double nerror = fixDeviationByPartName(graph_->nodes[id1]->id, graph_->nodes[id2]->id, error);

    /////////////
	if( logLevel_ >1 )
	{
		logStream_ <<"<(" << graph_->nodes[id1]->id <<", " << graph_->nodes[id2]->id <<"), ref)> center: " << refCenter[0] << ", " << refCenter[1] <<", "<< refCenter[2] << "\n"
					<<"normal: " << refNormal[0] <<", " << refNormal[1] << ", " << refNormal[2] << "\n";
		logStream_ << "deviation : " << error << "\n";
		logStream_ << "\n";
	}
    if ( nerror < thRefRadio_)
    {       
		PairRelation pr(graph_->nodes[id1], graph_->nodes[id2]);
        pr.diameter = computePairDiameter(pr);
        pr.deviation = error;
        refPairs_.push_back(pr);
        return true;
    }
    else
    {
        return false;
    }
}

void PairRelationDetector::isParalOrthoCoplanar(int id1, int id2)
{
    Structure::Node * n1 = graph_->nodes[id1];
    Structure::Node * n2 = graph_->nodes[id2];

    Vector_3 d1, d2;
    bool iscurve1 = node2direction(n1, d1);
    bool iscurve2 = node2direction(n2, d2);
    if ( iscurve1 == iscurve2 )
    {
        double error = errorOfParallel(d1, d2);
		double nerror = fixDeviationByPartName(n1->id, n2->id, error);
        if (nerror < thParal_)
        {
            PairRelation pr(n1, n2);
            pr.deviation = error;
            pr.diameter = computePairDiameter(pr);
            parallelPairs_.push_back(pr);
        }
        else
        {
            error = errorOfOrthogonal(d1, d2);
			nerror = fixDeviationByPartName(n1->id, n2->id, error);

            if ( nerror < thOthog_)
            {
                PairRelation pr(n1, n2);
                pr.deviation = error;
                pr.diameter = computePairDiameter(pr);
				orthogonalPairs_.push_back( pr );
            }
            if (iscurve1 && iscurve2 )
            {
                Line_3 l1 = curve2line(n1), l2 = curve2line(n2);
                error = squared_distance(l1, l2);
				nerror = fixDeviationByPartName(n1->id, n2->id, error);

                if ( nerror < thCopla_)
                {
                    PairRelation pr(n1, n2);
                    createPlane(n1,n2, pr.point, pr.normal);
                    pr.deviation = error;
                    pr.diameter = computePairDiameter(pr);
					coplanarPairs_.push_back( pr );
                }
            }
        }
    }
    else// curve & sheet
    {
        double error = errorOfParallel(d1, d2);
		double nerror = fixDeviationByPartName(n1->id, n2->id, error);

        if (nerror < thOthog_)
        {
            PairRelation pr(n1, n2);
            pr.deviation = error;
            pr.diameter = computePairDiameter(pr);
			orthogonalPairs_.push_back(pr);
        }
        else
        {
            Vector3 point, normal;
            if ( iscurve1 )
            {
                sheet2plane(dynamic_cast<Structure::Sheet *>(n2), point, normal);
                Segment_3 cs = curve2segment(n1);
                error = errorOfLineInPlane( cs, point, normal);
            }
            else
            {
                sheet2plane(dynamic_cast<Structure::Sheet *>(n1), point, normal);
                Segment_3 cs = curve2segment(n2);
                error = errorOfLineInPlane( cs, point, normal);
            }
			nerror = fixDeviationByPartName(n1->id, n2->id, error);

            if ( nerror < thCopla_)
            {
                PairRelation pr(n1, n2);
                pr.point = point;
                pr.normal = normal;
                pr.deviation = error;
                pr.diameter = computePairDiameter(pr);
				coplanarPairs_.push_back(pr);
            }
        }
    }
}

//bool PairRelationDetector::has_rot_relation(int id1, int id2)
//{
//	Structure::Node * n1 = graph_->nodes[id1];
//	Structure::Node * n2 = graph_->nodes[id2];
//	if ( Structure::SHEET == n1->type() || Structure::SHEET == n2->type()) // jjcao todo, check planes have rot relation
//		return false;

//	Line_3 l1 = curve2line(n1);
//	Line_3 l2 = curve2line(n2);

//	Object result = intersection(l1, l2);
//	if (const Point_3<Kernel> *ipoint = object_cast<Point_3<Kernel> >(&result))
//	{
//		Eigen::Vector3d center(ipoint->x(), ipoint->y(), ipoint->z());
//
//		Vector_3 v1 = l1;
//		Vector_3 v2 = l2;
//		Eigen::Vector3d vec1(v1.x(), v1.y(), v1.z());
//		Eigen::Vector3d vec2(v2.x(), v2.y(), v2.z());
//		vec1.normalize(); vec2.normalize();
//		Eigen::Vector3d direction = vec1.cross(vec2);

//		//////////////////////////
//		double angle = asin( vec1.cross(vec2).norm() );
//		double error;
//		Structure::Node * n;
//		if ( vec1.dot(vec2) > 0)
//		{
//			n = n2->clone();
//			rotNodeCpts(n, center, direction, angle);
//			error = distanceBetweenTransNodes(n1, n);
//		}
//		else
//		{
//			n = n1->clone();
//			rotNodeCpts(n, center, direction, angle);
//			error = distanceBetweenTransNodes(n2, n);
//		}
//
//		//////////////////////////////
//		if (error < thRot_)
//		{
//			PairRelation pairRel;
//			pairRel.type = ROT;
//			if ( vec1.dot(vec2) > 0)
//			{
//				pairRel.id1 = n1->id;
//				pairRel.id2 = n2->id;
//			}
//			else
//			{
//				pairRel.id1 = n2->id;
//				pairRel.id2 = n1->id;
//			}
///*			pairRel.center = center;
//			pairRel.direction = direction;*/
//			pairRelations_.push_back(pairRel);
//			return true;
//		}
//
//	}
//	else
//	{
//		return false;
//	}
//}
