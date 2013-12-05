#include "GlobalReflectionSymmScorer.h"

void GlobalReflectionSymmScorer::init()
{
	std::vector<Eigen::Vector3d> cptsV;
	for ( int i = 0; i < (int) graph_->nodes.size(); ++i)
	{
		Structure::Node * n = graph_->nodes[i];

		std::vector<Eigen::Vector3d> nodeCptsV;
		extractCpts( n, nodeCptsV, pointLevel_);

		Eigen::MatrixXd nodeCptsM;
		vectorPts2MatrixPts(nodeCptsV, nodeCptsM);
		nodesCpts_.push_back(nodeCptsM);

		nodesCenter_.push_back( Eigen::Vector3d(nodeCptsM.col(0).mean(), nodeCptsM.col(1).mean(), nodeCptsM.col(2).mean()) );

		for ( std::vector<Eigen::Vector3d>::iterator it = nodeCptsV.begin(); it!= nodeCptsV.end(); ++it)
		{
			cptsV.push_back(*it);
		}
	}
	vectorPts2MatrixPts(cptsV, cpts_);

	//center_= this->graph_->bbox().center();// not extract for symm
	center_ = Eigen::Vector3d( cpts_.col(0).mean(), cpts_.col(1).mean(), cpts_.col(2).mean());

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
        double minDist, meanDist, maxDist;
        Eigen::MatrixXd ptsout;
		reflect_points3d(cpts_, center, normal, ptsout);
		distanceBetween(cpts_, ptsout, minDist, meanDist, maxDist);

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
double GlobalReflectionSymmScorer::evaluate(const Eigen::Vector3d &center, const Eigen::Vector3d &normal, double maxGlobalSymmScore)
{
	double maxScore(0.0), deviation(0.0);
	for ( int i = 0; i < (int) graph_->nodes.size(); ++i)
    {        
		Eigen::MatrixXd ptsout;
        reflect_points3d(nodesCpts_[i], center, normal, ptsout);
	
		deviation = minMeanDistance(ptsout);
        
        if ( deviation > maxScore)
        {
            maxScore = deviation;
        }

		if(logLevel_>1)
		{
			logStream_ << "part: " << graph_->nodes[i]->id << ", deviation of symm: " << deviation << "\n";
		}
    }

	maxScore = maxScore/this->normalizeCoef_;
    maxScore = 1/(1+maxScore);
	maxScore = maxScore/maxGlobalSymmScore;

	if(logLevel_>0)
	{
		logStream_ << "\n\n";
		logStream_ << "center of reflection plane: " << center.x() << ", " << center.y() <<", " << center.z() << "\n";
		logStream_ << "normal of reflection plane: " << normal.x() << ", " << normal.y() <<", " << normal.z() << "\n";
		logStream_ << "max global score for normalization: " << maxGlobalSymmScore << "\n";
		logStream_ << "max score: " << maxScore << "\n";
	}

    return maxScore;
}
double GlobalReflectionSymmScorer::minMeanDistance(Eigen::MatrixXd& pts)
{
	double minDist, meanDist, maxDist, minMean=std::numeric_limits<double>::max();
	for ( int i = 0; i < (int) graph_->nodes.size(); ++i)
	{		
        distanceBetween(pts, nodesCpts_[i], minDist, meanDist, maxDist);
		if ( meanDist < minMean)
			minMean = meanDist;
	}
	return minMean;
}
int GlobalReflectionSymmScorer::findNearestPart(const Eigen::Vector3d &nc, const QString & type, double size, double& dist)
{
    int no(0);	
    dist = std::numeric_limits<double>::max();
	double tdist, size2;
    for ( int i = 0; i < (int) graph_->nodes.size(); ++i)
    {	
		if ( graph_->nodes[i]->type() != type) continue;

        Eigen::Vector3d& nc1 = nodesCenter_[i];
        tdist = (nc-nc1).norm();

		size2 = graph_->nodes[i]->bbox().diagonal().norm();
		double tmp = abs(size-size2)/(size+size2);
		

        if ( tmp < 0.7 && tdist < dist)
        {
            dist = tdist;
            no = i;
        }
    }
    return no;
}
