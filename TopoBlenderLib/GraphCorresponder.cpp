#include "GraphCorresponder.h"

#include <algorithm>
#include <fstream>

GraphCorresponder::GraphCorresponder( Structure::Graph *source, Structure::Graph *target )
{
	this->sg = source;
	this->tg = target;
}

float GraphCorresponder::supinfDistance( std::vector<Vector3> &A, std::vector<Vector3> &B )
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
	float ABDis = supinfDistance(A, B);
	float BADis = supinfDistance(B, A);

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
			if (M[i][j] != -1 && M[i][j] < minValue)
			{
				minValue = M[i][j];
				row = i;
				column = j;
			}
		}
	}

	return minValue != FLT_MAX;
}

void GraphCorresponder::findCorrespondences()
{
	computeDistanceMatrix();

	int sN = sg->nodes.size();
	int tN = tg->nodes.size();
	
	std::ofstream outF("one2one_node_correspondences.txt", std::ios::out);

	outF << "Source shape: " << sg->property["name"].toString().toStdString() << '\n';
	outF << "Target shape: " << tg->property["name"].toString().toStdString() << "\n\n\n";

	int r, c;
	while (minElementInMatrix(disM, r, c))
	{
		// r <-> c
		outF << sg->nodes[r]->id.toStdString() << '\t' << tg->nodes[c]->id.toStdString() << '\n';

		// Remove r and c in the disM
		for (int i = 0; i < sN; i++) // Column c
			disM[i][c] = -1;
		for (int j = 0; j < tN; j++) // Row r
			disM[r][j] = -1;
	}

	outF.close();
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
