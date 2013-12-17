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
	double deviation(-1.0),mean_dist, max_dist;

	if ( n1->id == n2->id)
		return deviation;
	
	Structure::Link* link = RelationDetector::findLink(n1, n2, this->graph_);	
	if ( link == 0)
		return deviation;


	if ( this->bUseLink_)
		deviation = RelationDetector::computeDeviationByLink(link);
	else
	{	
		distanceBetween(m1, m2, deviation, mean_dist, max_dist);
	}
		
	deviation = deviation/this->normalizeCoef_;
	if ( deviation > this->maxAllowDeviation_)
	{
		this->maxAllowDeviation_ = deviation;
	}

	return deviation;
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

//////////////////////////////////////////////////////
PairRelationDetector::PairRelationDetector(Structure::Graph* g, int ith, double normalizeCoef, bool bUseLink, bool bModifyDeviation, int logLevel)
					 :RelationDetector(g, "PairRelationDetector-", ith, normalizeCoef, 1, logLevel)
{
	bSource_ = ith == 0;
	bUseLink_ = bUseLink;
	bModifyDeviation_ = bModifyDeviation;

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

Eigen::MatrixXd& PairRelationDetector::findCptsByNodeId(Structure::Node* node)
{
	for ( int i = 0; i < (int) graph_->nodes.size(); ++i)
	{
		Structure::Node * n = graph_->nodes[i];
		if (node == n)
		{
			return nodesCpts_[i];
		}
	}

	return nodesCpts_[0];
}
void PairRelationDetector::detectConnectedPairs(Structure::Graph* g, QVector<PART_LANDMARK> &corres)
{
	if ( this->logLevel_ > 0)
	{
		if ( this->bUseLink_)
		{
			logStream_ << "\nConnected pairs use link distance \n";
		}
		else
		{
			logStream_ << "\nConnected pairs use skeleton distance \n";
		}
	}

	QVector<PairRelation> pairs;	
	double deviation, mean_dist, max_dist;;
	int tmp1 = graph_->edges.size();
	for ( int i = 0; i < tmp1; ++i)
	{
		Structure::Link *link = graph_->edges[i];
		if ( this->bUseLink_)
			deviation = RelationDetector::computeDeviationByLink(link);
		else
		{
			Eigen::MatrixXd& ptsi = findCptsByNodeId(link->n1);
			Eigen::MatrixXd& ptsj = findCptsByNodeId(link->n2);			
			distanceBetween(ptsi, ptsj, deviation, mean_dist, max_dist);
		}
		
		PairRelation prb(link->n1,link->n2);
		prb.deviation = deviation/this->normalizeCoef_;

		pairs.push_back(prb);
	}

	if (bModifyDeviation_)
	{
		if ( this->logLevel_ > 0)
		{
			logStream_ << "\nConnected pairs, modify deviation, normalize coeff is: " << this->normalizeCoef_ << "\n";
		}
		modifyPairsDegree(pairs, ConnectedPairModifier(g, normalizeCoef_, bUseLink_, pointLevel_), g, corres, "Connected pairs");
	}
	else
	{
		if ( this->logLevel_ > 0)
		{
			logStream_ << "\nConnected pairs, not modify deviation, normalize coeff is: " << this->normalizeCoef_ << "\n";
			int i(0),j(0); double tmp(0.0);
			for ( QVector<PairRelation>::iterator it = pairs.begin(); it != pairs.end(); ++it, ++i)
			{
				logStream_ << i << " " << *it << "\n";
				if (it->deviation > tmp)
				{
					tmp = it->deviation;
					j = i;
				}
			}
			this->logStream_ << "pair " << j << " with max deviation: " << tmp << "\n\n";
		}
	}

	connectedPairs_.clear();
	for (QVector<PairRelation>::iterator it = pairs.begin(); it != pairs.end(); ++it)
	{
		if (it->tag)
		{
			it->type = CONNECTED;
			connectedPairs_.push_back(*it);		
		}
	}

	if( logLevel_ >0 )
	{
		logStream_ << connectedPairs_.size() << " pairs used in tracing!" << "\n\n\n";
	}
}
void PairRelationDetector::detectOtherPairs(Structure::Graph* g, QVector<PART_LANDMARK> &corres)
{
	transPairs_.clear();    refPairs_.clear();
	otherPairs_.clear();

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
        }
    }

	//
	//if (bModifyDeviation_)
	//{
		//modifyPairsDegree(transPairs_, TransPairModifier(pointLevel_), g, corres, "Trans pairs");
		//modifyPairsDegree(refPairs_, RefPairModifier(pointLevel_), g, corres, "Reflection pairs");
	//}

	otherPairs_.clear();
	pushToOtherPairs(transPairs_, TRANS);
	pushToOtherPairs(refPairs_, REF);
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
