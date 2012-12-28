#include "GraphCorresponder.h"

#include <algorithm>
#include <fstream>

#define INVALID_VALUE -1

GraphCorresponder::GraphCorresponder( Structure::Graph *source, Structure::Graph *target )
{
	this->sg = source;
	this->tg = target;
}

float GraphCorresponder::supInfDistance( std::vector<Vector3> &A, std::vector<Vector3> &B )
{
	float supinfDis = -1;
	for (int i = 0; i < (int)A.size(); i++)
	{
		float infDis = FLT_MAX;
		for (int j = 0; j < (int)B.size(); j++)
		{
			float dis = (A[i] - B[j]).norm();

			if (dis < infDis)
				infDis = dis;
		}

		if (infDis > supinfDis)
			supinfDis = infDis;
	}

	return supinfDis;
}

float GraphCorresponder::HausdorffDistance( std::vector<Vector3> &A, std::vector<Vector3> &B )
{
	float ABDis = supInfDistance(A, B);
	float BADis = supInfDistance(B, A);

	return std::max(ABDis, BADis);
}


void GraphCorresponder::computeHausdorffDistanceMatrix( std::vector< std::vector<float> > & M )
{
	// Init
	initializeMatrix(M);

	// The control points of each node in the target graph
	std::vector< std::vector<Vector3> > targetControlPointsArray;
	foreach (Structure::Node *tNode, this->tg->nodes)
		targetControlPointsArray.push_back(tNode->controlPoints());

	// The Hausdorff distance
	int sN = sg->nodes.size();
	int tN = tg->nodes.size();
	for(int i = 0; i < sN; i++)
	{
		std::vector<Vector3> sPoints = sg->nodes[i]->controlPoints();
		for (int j = 0; j < tN; j++)
		{
			std::vector<Vector3> &tPoints = targetControlPointsArray[j];
			M[i][j] = HausdorffDistance(sPoints, tPoints);
		}
	}

	normalizeMatrix(M);
}


void GraphCorresponder::computeDegreeDifferenceMatrix( std::vector< std::vector<float> > & M )
{
	// Init
	initializeMatrix(M);

	// Difference on Degrees
	int sN = sg->nodes.size();
	int tN = tg->nodes.size();
	for(int i = 0; i < sN; i++)
	{
		int sDegree = sg->nodes[i]->property["degree"].toInt();
		for (int j = 0; j < tN; j++)
		{
			int tDegree = tg->nodes[j]->property["degree"].toInt();

			M[i][j] = abs(sDegree - tDegree);
		}
	}

	normalizeMatrix(M);
}


void GraphCorresponder::computeDistanceMatrix()
{
	// Init
	initializeMatrix(disM);

	// All measure matrices
	std::vector< std::vector<float> > M1, M2;
	computeHausdorffDistanceMatrix(M1);
	computeDegreeDifferenceMatrix(M2);

	// Merge different measures
	int sN = sg->nodes.size();
	int tN = tg->nodes.size();
	for(int i = 0; i < sN; i++){
		for (int j = 0; j < tN; j++){
			disM[i][j] = M1[i][j] + 0.5 * M2[i][j];
		}
	}

	normalizeMatrix(disM);
}


void GraphCorresponder::initializeMatrix(std::vector< std::vector<float> > & M)
{
	int sN = sg->nodes.size();
	int tN = tg->nodes.size();
	std::vector<float> tmp(tN, INVALID_VALUE);
	M.resize(sN, tmp);
}


void GraphCorresponder::normalizeMatrix(std::vector< std::vector<float> > & M)
{
	float maxDis = -1;

	// Find the maximum value
	int sN = sg->nodes.size();
	int tN = tg->nodes.size();
	for(int i = 0; i < sN; i++){
		for (int j = 0; j < tN; j++)
		{
			if (M[i][j] > maxDis)
				maxDis = M[i][j];
		}
	}

	// Normalize
	for(int i = 0; i < sN; i++){
		for (int j = 0; j < tN; j++)
		{
			M[i][j] /= maxDis;
		}
	}
}


void GraphCorresponder::visualizePart2PartDistance( int sourceID )
{
	if (disM.empty()) 
		computeDistanceMatrix();

	// The source
	sourceID %= sg->nodes.size();
	for (int i = 0; i < sg->nodes.size(); i++)
	{
		Structure::Node *sNode = sg->nodes[i];
		float color = (i == sourceID)? 1.0 : 0.0;
		sNode->vis_property["color"] = qtJetColorMap(color);
		sNode->vis_property["showControl"] = false;
	}

	// Visualize by jet color
	for (int j = 0; j < tg->nodes.size(); j++)
	{
		Structure::Node *tNode = tg->nodes[j];
		float value = (1 - disM[sourceID][j]) / 4 + 0.75;
		tNode->vis_property["color"] = qtJetColorMap(value);
		tNode->vis_property["showControl"] = false;
	}
}


bool GraphCorresponder::minElementInMatrix( std::vector< std::vector<float> > &M, int &row, int &column )
{
	if (M.size() == 0 || M[0].size() == 0)
	{
		qDebug()  << "Warning: minElementInMatrix: the input matrix cannot be empty.";
		return false;
	}

	float minValue = FLT_MAX;
	for (int i = 0; i < (int)M.size(); i++){
		for (int j = 0; j < (int)M[0].size(); j++)
		{
			if (M[i][j] != INVALID_VALUE && M[i][j] < minValue)
			{
				minValue = M[i][j];
				row = i;
				column = j;
			}
		}
	}

	return minValue != FLT_MAX;
}

void GraphCorresponder::findOneToOneCorrespondences()
{
	if (disM.empty()) 
		computeDistanceMatrix();

	// Copy the disM
	std::vector< std::vector<float> > disMatrix = disM;

	int sN = sg->nodes.size();
	int tN = tg->nodes.size();
	
	std::ofstream outF("correspondences_one2one.txt", std::ios::out);

	outF << "Source shape: " << sg->property["name"].toString().toStdString() << '\n';
	outF << "Target shape: " << tg->property["name"].toString().toStdString() << "\n\n\n";

	int r, c;
	while (minElementInMatrix(disMatrix, r, c))
	{
		// To-Do: check the consistency

		// source:r <-> target:c
		outF << sg->nodes[r]->id.toStdString() << '\t' << tg->nodes[c]->id.toStdString() << '\n';

		// Remove scores of r and c
		for (int i = 0; i < sN; i++) // Column c
			disMatrix[i][c] = INVALID_VALUE;
		for (int j = 0; j < tN; j++) // Row r
			disMatrix[r][j] = INVALID_VALUE;
	}

	outF.close();
}

void GraphCorresponder::findOneToManyCorrespondences()
{
	if (disM.empty()) 
		computeDistanceMatrix();

	// Copy the disM
	std::vector< std::vector<float> > disMatrix = disM;

	int sN = sg->nodes.size();
	int tN = tg->nodes.size();

	std::ofstream outF("correspondences_one2many.txt", std::ios::out);

	outF << "Source shape: " << sg->property["name"].toString().toStdString() << '\n';
	outF << "Target shape: " << tg->property["name"].toString().toStdString() << "\n\n\n";

	int r, c;
	while (minElementInMatrix(disMatrix, r, c))
	{
		// source:r <-> target:c
		// The threshold
		float threshold  = disMatrix[r][c] * 0.9;

		// Search for "many" in source
		std::vector<int> r_many;
		for (int ri = 0; ri < sN; ri++)
		{
			if (ri == r || disMatrix[ri][c] == INVALID_VALUE)
				continue;

			// ri is close to c
			if (disMatrix[ri][c] >= threshold) 
			{
				bool verified = true;

				// c is the best choice for ri
				for(int cj = 0; cj < tN; cj++)
				{
					if (disMatrix[ri][cj] == INVALID_VALUE || cj == c)
						continue;

					if (disMatrix[ri][cj] < disMatrix[ri][c])
					{
						verified = false;
						break;
					}
				}

				if (verified)
					r_many.push_back(ri);
			}
		}

		// Search for "many" in target
		std::vector<int> c_many;
		for (int ci = 0; ci < tN; ci++)
		{
			if (ci == c || disMatrix[r][ci] == INVALID_VALUE)	
				continue;

			// ci is close to r
			if (disMatrix[r][ci] >= threshold) 
			{
				bool verified = true;

				// r is the best choice for ci
				for (int rj = 0; rj < sN; rj++)
				{
					if (disMatrix[rj][ci] == INVALID_VALUE || rj == r)
						continue;

					if (disMatrix[rj][ci] < disMatrix[r][ci])
					{
						verified = false;
						break;
					}
				}

				if (verified)
					c_many.push_back(ci);
			}
		}

		// Results
		if (c_many.size() > r_many.size()) // r <-> {c_i}
		{
			outF << sg->nodes[r]->id.toStdString() << "\t:\t"
				 << tg->nodes[c]->id.toStdString() << '\t';
			
			foreach(int ci, c_many)
			{
				outF << tg->nodes[ci]->id.toStdString() << '\t';

				for (int i = 0; i < sN; i++) // Column c
					disMatrix[i][ci] = INVALID_VALUE;
			}

			outF << '\n';
		}
		else // {r_i} <-> c
		{
			foreach(int ri, r_many)
			{
				outF << sg->nodes[ri]->id.toStdString() << '\t';

				for (int j = 0; j < tN; j++) // Row r
					disMatrix[ri][j] = INVALID_VALUE;
			}

			outF << sg->nodes[r]->id.toStdString() << "\t:\t"
				 << tg->nodes[c]->id.toStdString() << '\n';
		}			
		
		// Remove r and c in the disMatrix
		for (int i = 0; i < sN; i++) // Column c
			disMatrix[i][c] = INVALID_VALUE;
		for (int j = 0; j < tN; j++) // Row r
			disMatrix[r][j] = INVALID_VALUE;
	}

	outF.close();
}


void GraphCorresponder::correspondTwoCurves( Structure::Curve *sCurve, Structure::Curve *tCurve )
{
	std::vector<Vector3> sCtrPnts = sCurve->controlPoints();
	std::vector<Vector3> tCtrPnts = tCurve->controlPoints();

	// Euclidean for now, can use Geodesic distance instead if need
	Vector3 scenter = sCurve->center();
	Vector3 sfront = sCtrPnts.front() - scenter;
	Vector3 tcenter = tCurve->center();
	Vector3 tfront = tCtrPnts.front() - tcenter;
	Vector3 tback = tCtrPnts.back() - tcenter;

	float f2f = (sfront - tfront).norm();
	float f2b = (sfront - tback).norm();

	if (f2f > f2b)
	{
		// Flip the target
		std::vector<Scalar> tWeights = tCurve->controlWeights();
		std::reverse(tCtrPnts.begin(), tCtrPnts.end());
		std::reverse(tWeights.begin(), tWeights.end());

		NURBSCurve newCurve(tCtrPnts, tWeights);
		tCurve->curve = newCurve;
	}
}


void GraphCorresponder::correspondTwoSheets( Structure::Sheet *sSheet, Structure::Sheet *tSheet )
{
	Array2D_Vector3 sCtrPnts = sSheet->surface.mCtrlPoint;
	Array2D_Vector3 tCtrPnts = tSheet->surface.mCtrlPoint;

	Vector3 scenter = sSheet->center();
	Vector3 tcenter = tSheet->center();

	// Get the extreme points.
	Vector3 s0 = sCtrPnts.front().front();
	Vector3 s1 = sCtrPnts.front().back();
	Vector3 t0 = tCtrPnts.front().front();
	Vector3 t1 = tCtrPnts.front().back();

	Vector3 sDir = (s0 + s1) / 2 - scenter;
	Vector3 tDir = (t0 + t1) / 2 - tcenter;
 
	Scalar cosAngle = dot(sDir.normalized(), tDir.normalized());
	Scalar angle = acosf(cosAngle);
	if ()
	{
	}
}
