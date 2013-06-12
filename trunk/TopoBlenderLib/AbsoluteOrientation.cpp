#include "AbsoluteOrientation.h"

void AbsoluteOrientation::compute( Eigen::Vector3d Xa,Eigen::Vector3d Ya,Eigen::Vector3d Za, Eigen::Vector3d Xb,Eigen::Vector3d Yb,Eigen::Vector3d Zb, Eigen::Quaterniond &result )
{
	std::vector<Eigen::Vector3d> A,B;
	A.push_back(Xa); A.push_back(Ya); A.push_back(Za);
	B.push_back(Xb); B.push_back(Yb); B.push_back(Zb);
	compute(A,B,result);
}

void AbsoluteOrientation::compute( std::vector<Eigen::Vector3d> &left, std::vector<Eigen::Vector3d> &right, Eigen::Quaterniond &result )
{
	int i, pairNum = left.size();

	Eigen::MatrixXd muLmuR = Eigen::MatrixXd::Zero(3,3), M = Eigen::MatrixXd::Zero(3,3), 
		curMat = Eigen::MatrixXd::Zero(3,3), N = Eigen::MatrixXd::Zero(4,4);

	Eigen::Vector3d meanFirst(0,0,0), meanSecond(0,0,0); //assume points set to zero by constructor

	//compute the mean of both point sets
	for (i=0; i<pairNum; i++) {
		meanFirst[0] += left[i][0];	    meanFirst[1] += left[i][1];	    meanFirst[2] += left[i][2];
		meanSecond[0] += right[i][0];	  meanSecond[1] += right[i][1];	  meanSecond[2] += right[i][2];
	}
	meanFirst[0]/=pairNum;	  meanFirst[1]/=pairNum;	  meanFirst[2]/=pairNum;
	meanSecond[0]/=pairNum;	  meanSecond[1]/=pairNum;	  meanSecond[2]/=pairNum;

	//compute the matrix muLmuR
	muLmuR(0,0) = meanFirst[0]*meanSecond[0];		
	muLmuR(0,1) = meanFirst[0]*meanSecond[1];		
	muLmuR(0,2) = meanFirst[0]*meanSecond[2];
	muLmuR(1,0) = meanFirst[1]*meanSecond[0];
	muLmuR(1,1) = meanFirst[1]*meanSecond[1];
	muLmuR(1,2) = meanFirst[1]*meanSecond[2];
	muLmuR(2,0) = meanFirst[2]*meanSecond[0];
	muLmuR(2,1) = meanFirst[2]*meanSecond[1];
	muLmuR(2,2) = meanFirst[2]*meanSecond[2];

	//compute the matrix M
	for (i=0; i<pairNum; i++) {
		Eigen::Vector3d &leftPoint = left[i];
		Eigen::Vector3d &rightPoint = right[i];
		curMat(0,0) = leftPoint[0]*rightPoint[0];		
		curMat(0,1) = leftPoint[0]*rightPoint[1];		
		curMat(0,2) = leftPoint[0]*rightPoint[2];
		curMat(1,0) = leftPoint[1]*rightPoint[0];
		curMat(1,1) = leftPoint[1]*rightPoint[1];
		curMat(1,2) = leftPoint[1]*rightPoint[2];
		curMat(2,0) = leftPoint[2]*rightPoint[0];
		curMat(2,1) = leftPoint[2]*rightPoint[1];
		curMat(2,2) = leftPoint[2]*rightPoint[2];
		M+=curMat;
	}
	M+= (muLmuR *(-pairNum));

	//compute the matrix N	
	Eigen::MatrixXd tmpMat = Eigen::MatrixXd::Zero(3,3);
	double A12, A20, A01;
	double traceM = 0.0;
	for(i=0; i<3; i++) traceM += M(i,i);

	tmpMat.diagonal() = Eigen::VectorXd::Constant(3, -traceM); //tmpMat.fill_diagonal(-traceM);
	tmpMat += (M + M.transpose());

	A12 = M(1,2) - M(2,1);
	A20 = M(2,0) - M(0,2);
	A01 = M(0,1) - M(1,0);

	N(0,0)=traceM; N(0,1)=A12; N(0,2)=A20; N(0,3)=A01;
	N(1,0)=A12;
	N(2,0)=A20;
	N(3,0)=A01;

	N.bottomRightCorner(3,3) = tmpMat; //N.update(tmpMat,1,1);

	////find the eigenvector that belongs to the maximal 
	////eigenvalue of N, eigenvalues are sorted from smallest to largest
	//vnl_symmetric_eigensystem<double> eigenSystem(N);
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;
	es.compute(N);
	Eigen::MatrixXd V = es.eigenvectors();

	//std::stringstream ss;ss << V;
	//qDebug() << qPrintable(ss.str().c_str());

	//setRotationQuaternion(eigenSystem.V(0,3),eigenSystem.V(1,3),eigenSystem.V(2,3),eigenSystem.V(3,3), true);
	result = Eigen::Quaterniond( V(0,3),V(1,3),V(2,3),V(3,3) ).normalized();
}

void AbsoluteOrientation::minOnT( std::vector<Eigen::Vector3d> &frameA, std::vector<Eigen::Vector3d> &frameB, Eigen::Quaterniond &result )
{
	// Find minimum rotation for T
	result = Eigen::Quaterniond::FromTwoVectors( frameA[2], frameB[2] );

	// Apply to frame A
	for(int i = 0;i < (int)frameA.size(); i++)
		frameA[i] = result * frameA[i];

	std::vector<Eigen::Vector3d> left, right;

	for(int i = 0;i < (int)frameA.size(); i++)
	{
		left.push_back( frameA[i] );
		right.push_back( frameB[i] );
	}

	Eigen::Quaterniond rot2;
	AbsoluteOrientation::compute(left, right, rot2);

	/*result = rot2result * rot2*/;
}
