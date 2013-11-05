#include "GlobalReflectionSymmScorer.h"
int extractCpts( Structure::Node * n, std::vector<Eigen::Vector3d>& mcpts, int pointsLevel)
{
    if ( pointsLevel == 2)
    {
    	SurfaceMesh::Model * m1 = n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >().data();
    	SurfaceMesh::Vector3VertexProperty pts1 = m1->vertex_coordinates();
    	double* tmp;
    	foreach(Vertex v1, m1->vertices())
    	{
    		tmp = pts1[v1].data();
    		mcpts.push_back( Eigen::Vector3d(tmp[0], tmp[1], tmp[2]) );
    	}
    }
	else
	{
		if ( Structure::CURVE == n->type() )
		{
			Structure::Curve* c = dynamic_cast<Structure::Curve *>(n);
			if ( pointsLevel == 1)
			{
				for (int i = 0; i < (int) c->numCtrlPnts(); ++i)
				{
					mcpts.push_back( c->controlPoint(i));
				}
			}
			else
			{
				mcpts.push_back( c->controlPoint(0) );
				mcpts.push_back( c->controlPoint( c->numCtrlPnts()-1 ) );
			}
		}
		else
		{
			Structure::Sheet* s = dynamic_cast<Structure::Sheet *>(n);
			if ( pointsLevel == 1)
			{
				for (int i = 0; i < (int) s->numCtrlPnts(); ++i)
				{
					mcpts.push_back( s->controlPoint(i));
				}
			}
			else
			{
				int nu = s->numUCtrlPnts(), nv = s->numVCtrlPnts();
				mcpts.push_back( s->surface.GetControlPoint(0,0) );
				mcpts.push_back( s->surface.GetControlPoint(nu-1,0) );
				mcpts.push_back( s->surface.GetControlPoint(nu-1,nv-1) );
				mcpts.push_back( s->surface.GetControlPoint(0,nv-1) );
			}
		}
	}

	return mcpts.size();
}
void GlobalReflectionSymmScorer::init()
{
	std::vector<Eigen::Vector3d> cptsV;
	std::vector<int> numcpts;
	for ( int i = 0; i < (int) graph_->nodes.size(); ++i)
	{
		Structure::Node * n = graph_->nodes[i];
		double nsize = n->bbox().diagonal().norm();
		if (  nsize == 0)
		{
			if(logLevel_>1)
			{
				logStream_ << "node " << i << ": " << n->id << ", diameter: " << nsize << ", cpts num: " << n->numCtrlPnts() << "\n";	

			}
			continue;
		}

		std::vector<Eigen::Vector3d> nodeCptsV;
		int tmp = extractCpts( n, nodeCptsV, pointsLevel_);
		numcpts.push_back( nodeCptsV.size() );

		Eigen::MatrixXd nodeCptsM;
		vectorPts2MatrixPts(nodeCptsV, nodeCptsM);
		nodesCpts_.push_back(nodeCptsM);

		nodesCenter_.push_back( Eigen::Vector3d(nodeCptsM.col(0).mean(), nodeCptsM.col(1).mean(), nodeCptsM.col(2).mean()) );
		nonDegeneratedNodes_.push_back( n );

		for ( std::vector<Eigen::Vector3d>::iterator it = nodeCptsV.begin(); it!= nodeCptsV.end(); ++it)
		{
			cptsV.push_back(*it);
		}
	}
	vectorPts2MatrixPts(cptsV, cpts_);

	//center_= this->graph_->bbox().center();// not extract for symm
	center_ = Eigen::Vector3d( cpts_.col(0).mean(), cpts_.col(1).mean(), cpts_.col(2).mean());



	////////////////////
	if(logLevel_>1)
	{
		logStream_ << nonDegeneratedNodes_.size() << " non null nodes in the graph: \n";
		for ( int i = 0; i < (int) nonDegeneratedNodes_.size(); ++i)
		{
			logStream_ << "    node " << i << ": " << nonDegeneratedNodes_[i]->id << ", diameter: " << nonDegeneratedNodes_[i]->bbox().diagonal().norm() 
				<< ", cpts num: " << numcpts[i]<< "\n";			
		}
		logStream_ << "\n\n";
	}
}


//////////////////////
double GlobalReflectionSymmScorer::evaluate()
{    
    normal_ = findReflectPlane(this->center_);
	return evaluate(this->center_, normal_, 1.0);
}
Eigen::Vector3d GlobalReflectionSymmScorer::findReflectPlane(const Eigen::Vector3d& center)
{
    Eigen::Vector3d rnormal;
    double meanScore = std::numeric_limits<double>::max();//, maxScore;
    Eigen::Vector3d axis(0,0,1);
    Eigen::Vector3d initNormal(1,0,0), normal;
    double angleStep = 3.1415926*0.5;
    for ( int i = 0; i < (int) 2; ++i)
    {
        normal = rotatedVec(initNormal, i*angleStep, axis);
        double meanDist, maxDist;
        Eigen::MatrixXd ptsout;
		reflectPoints(cpts_, center, normal, ptsout);
		distanceBetween(cpts_, ptsout, meanDist, maxDist);

        if ( meanDist < meanScore)
        {
            meanScore = meanDist;
            //maxScore = max_dist;
            rnormal = normal;
        }

        if(logLevel_>1)
        {
            logStream_ << "current plane's center: ["
                        << center_.x() << "," << center_.y() << "," << center_.z() << "], normal: ["
                        << normal.x() << "," << normal.y() << "," << normal.z() << "], error: " << meanDist << "\n";
        }
    }
    return rnormal;
}
double GlobalReflectionSymmScorer::evaluate(Eigen::Vector3d &center, Eigen::Vector3d &normal, double maxGlobalSymmScore)
{
	double meanScore(0.0), maxScore(0.0);
	double gsize = graph_->bbox().diagonal().norm();
    for ( int i = 0; i < (int) nonDegeneratedNodes_.size(); ++i)
    {
        Eigen::Vector3d nc = nodesCenter_[i];
        Eigen::Vector3d nc1;
        reflectPoint(nc, center, normal, nc1);
        double dist;
        int j = findNearestPart(nc1, nonDegeneratedNodes_[i]->type(), dist);
        
		Eigen::MatrixXd ptsout;
        reflectPoints(nodesCpts_[i], center, normal, ptsout);
        dist = distanceBetween(ptsout, nodesCpts_[j]);
		//dist = std::abs(nonDegeneratedNodes_[i]->bbox().diagonal().norm() - nonDegeneratedNodes_[j]->bbox().diagonal().norm());///graph_->bbox().diagonal().norm();
		//dist *= nonDegeneratedNodes_[i]->bbox().diagonal().norm();

        
        meanScore += dist;
        if ( dist > maxScore)
        {
            maxScore = dist;
        }

		if(logLevel_>1)
		{
			dist= dist/gsize;
			dist = 1/(1+dist);
			logStream_ << "part: " << nonDegeneratedNodes_[i]->id << ", deviation of symm: " << dist/maxGlobalSymmScore << "\n";
		}
    }
    meanScore = meanScore/nonDegeneratedNodes_.size();
    meanScore = meanScore/gsize;
    meanScore = 1/(1+meanScore);
	meanScore = meanScore/maxGlobalSymmScore;

    maxScore = maxScore/gsize;
    maxScore = 1/(1+maxScore);
	maxScore = maxScore/maxGlobalSymmScore;

	if(logLevel_>0)
	{
		logStream_ << "\n\n";
		logStream_ << "center of reflection plane: " << center.x() << ", " << center.y() <<", " << center.z() << "\n";
		logStream_ << "normal of reflection plane: " << normal.x() << ", " << normal.y() <<", " << normal.z() << "\n";
		logStream_ << "max global score for normalization: " << maxGlobalSymmScore << "\n";
		logStream_ << "mean score: " << meanScore << "\n";
		logStream_ << "max score: " << maxScore << "\n";
	}

    return maxScore;
}

int GlobalReflectionSymmScorer::findNearestPart(Eigen::Vector3d &nc, QString & type, double& dist)
{
    int no(0);
    dist = std::numeric_limits<double>::max();
    for ( int i = 0; i < (int) nonDegeneratedNodes_.size(); ++i)
    {
        Eigen::Vector3d& nc1 = nodesCenter_[i];
        double tdist = (nc-nc1).norm();
        if ( nonDegeneratedNodes_[i]->type() == type && tdist < dist)
        {
            dist = tdist;
            no = i;
        }
    }
    return no;
}