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
	for (int i = 0; i < A.size(); i++)
	{
		float infDis = FLT_MAX;
		for (int j = 0; j < B.size(); j++)
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




void GraphCorresponder::computeDistanceMatrix()
{
	// Init
	int sN = sg->nodes.size();
	int tN = tg->nodes.size();
	std::vector<float> tmp(tN, -1.0);
	disM.resize(sN, tmp);

	// The control points of each node in the target graph
	std::vector< std::vector<Vector3> > targetControlPointsArray;
	foreach (Structure::Node *tNode, this->tg->nodes)
		targetControlPointsArray.push_back(tNode->controlPoints());

	// Compute the distance matrix
	for(int i = 0; i < sN; i++)
	{
		std::vector<Vector3> sPoints = sg->nodes[i]->controlPoints();
		for (int j = 0; j < tN; j++)
		{
			std::vector<Vector3> &tPoints = targetControlPointsArray[j];
			disM[i][j] = HausdorffDistance(sPoints, tPoints);
		}
	}
}


void GraphCorresponder::normalizeDistanceMatrix()
{
	float maxDis = -1;

	int sN = sg->nodes.size();
	int tN = tg->nodes.size();
	for(int i = 0; i < sN; i++){
		for (int j = 0; j < tN; j++)
		{
			if (disM[i][j] > maxDis)
				maxDis = disM[i][j];
		}
	}

	for(int i = 0; i < sN; i++){
		for (int j = 0; j < tN; j++)
		{
				disM[i][j] /= maxDis;
				disM[i][j] = 1 - disM[i][j];
				disM[i][j] = 0.5 + disM[i][j] / 2;
		}
	}
}


void GraphCorresponder::visualizeHausdorffDistances( int sourceID )
{
	computeDistanceMatrix();
	normalizeDistanceMatrix();

	// The source
	sourceID %= sg->nodes.size();
	for (int i = 0; i < sg->nodes.size(); i++)
	{
		Structure::Node *tNode = sg->nodes[i];
		float color = (i == sourceID)? 1.0 : 0.0;
		tNode->vis_property["color"] = qtJetColorMap(color);
		tNode->vis_property["showControl"] = false;
	}

	// Visualize by jet color
	for (int j = 0; j < tg->nodes.size(); j++)
	{
		Structure::Node *tNode = tg->nodes[j];
		tNode->vis_property["color"] = qtJetColorMap(disM[sourceID][j]);
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
	for (int i = 0; i < M.size(); i++){
		for (int j = 0; j < M[0].size(); j++)
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
	computeDistanceMatrix();

	int sN = sg->nodes.size();
	int tN = tg->nodes.size();
	
	std::ofstream outF("correspondences_one2one.txt", std::ios::out);

	outF << "Source shape: " << sg->property["name"].toString().toStdString() << '\n';
	outF << "Target shape: " << tg->property["name"].toString().toStdString() << "\n\n\n";

	int r, c;
	while (minElementInMatrix(disM, r, c))
	{
		// source:r <-> target:c
		outF << sg->nodes[r]->id.toStdString() << '\t' << tg->nodes[c]->id.toStdString() << '\n';

		// Remove scores of r and c
		for (int i = 0; i < sN; i++) // Column c
			disM[i][c] = INVALID_VALUE;
		for (int j = 0; j < tN; j++) // Row r
			disM[r][j] = INVALID_VALUE;
	}

	outF.close();
}



void GraphCorresponder::findOneToManyCorrespondences()
{
	computeDistanceMatrix();

	int sN = sg->nodes.size();
	int tN = tg->nodes.size();

	std::ofstream outF("correspondences_one2many.txt", std::ios::out);

	outF << "Source shape: " << sg->property["name"].toString().toStdString() << '\n';
	outF << "Target shape: " << tg->property["name"].toString().toStdString() << "\n\n\n";

	int r, c;
	while (minElementInMatrix(disM, r, c))
	{
		// source:r <-> target:c
		// The threshold
		float threshold  = disM[r][c] * 0.9;

		// Search for "many" in source
		std::vector<int> r_many;
		for (int ri = 0; ri < sN; ri++)
		{
			if (ri == r || disM[ri][c] == INVALID_VALUE)
				continue;

			// ri is close to c
			if (disM[ri][c] >= threshold) 
			{
				bool verified = true;

				// c is the best choice for ri
				for(int cj = 0; cj < tN; cj++)
				{
					if (disM[ri][cj] == INVALID_VALUE || cj == c)
						continue;

					if (disM[ri][cj] < disM[ri][c])
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
			if (ci == c || disM[r][ci] == INVALID_VALUE)	
				continue;

			// ci is close to r
			if (disM[r][ci] >= threshold) 
			{
				bool verified = true;

				// r is the best choice for ci
				for (int rj = 0; rj < sN; rj++)
				{
					if (disM[rj][ci] == INVALID_VALUE || rj == r)
						continue;

					if (disM[rj][ci] < disM[r][ci])
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
					disM[i][ci] = INVALID_VALUE;
			}

			outF << '\n';
		}
		else // {r_i} <-> c
		{
			foreach(int ri, r_many)
			{
				outF << sg->nodes[ri]->id.toStdString() << '\t';

				for (int j = 0; j < tN; j++) // Row r
					disM[ri][j] = INVALID_VALUE;
			}

			outF << sg->nodes[r]->id.toStdString() << "\t:\t"
				 << tg->nodes[c]->id.toStdString() << '\n';
		}			
		
		// Remove r and c in the disM
		for (int i = 0; i < sN; i++) // Column c
			disM[i][c] = INVALID_VALUE;
		for (int j = 0; j < tN; j++) // Row r
			disM[r][j] = INVALID_VALUE;
	}

	outF.close();
}
