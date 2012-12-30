#include "GraphCorresponder.h"

#include <algorithm>
#include <fstream>

#define INVALID_VALUE -1

GraphCorresponder::GraphCorresponder( Structure::Graph *source, Structure::Graph *target )
{
	this->sg = source;
	this->tg = target;

	sIsLandmark.resize(sg->nodes.size(), false);
	tIsLandmark.resize(tg->nodes.size(), false);
}


void GraphCorresponder::addLandmarks( std::set<QString> sParts, std::set<QString> tParts )
{
	// Check if those parts are available
	foreach(QString strID, sParts)
	{
		Structure::Node * node = sg->getNode(strID);
		if (!node)
		{
			qDebug() << "Add landmarks: " << strID << " doesn't exist in the source graph.";
			return;
		}
		int idx = node->property["index"].toInt();
		if (sIsLandmark[idx])
		{
			qDebug() << "Add landmarks: " << strID << " in the source graph has already been corresponded.";
			return;
		}
	}

	foreach(QString strID, tParts)
	{
		Structure::Node * node = tg->getNode(strID);
		if (!node)
		{
			qDebug() << "Add landmarks: " << strID << " doesn't exist in the target graph.";
			return;
		}
		int idx = node->property["index"].toInt();
		if (tIsLandmark[idx])
		{
			qDebug() << "Add landmarks: " << strID << " in the target graph has already been corresponded.";
			return;
		}
	}

	// Mark those parts
	foreach(QString strID, sParts)
	{
		int idx = sg->getNode(strID)->property["index"].toInt();
		sIsLandmark[idx] = true;
	}

	foreach(QString strID, tParts)
	{
		int idx = tg->getNode(strID)->property["index"].toInt();
		tIsLandmark[idx] = true;
	}

	// Store correspondence
	landmarks.push_back(std::make_pair(sParts, tParts));
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
			if (sg->nodes[i]->type() != tg->nodes[j]->type())
				disM[i][j] = INVALID_VALUE;
			else
				disM[i][j] = M1[i][j] + 0.5 * M2[i][j];
		}
	}

	// Add landmarks
	foreach (SET_PAIR set2set, landmarks)
	{
		// Convert strID to int id
		std::set<int> sIDs, tIDs;
		foreach (QString strID, set2set.first)
			sIDs.insert(sg->getNode(strID)->property["index"].toInt());
		foreach (QString strID, set2set.second)
			tIDs.insert(tg->getNode(strID)->property["index"].toInt());

		// Set distances to 0
		foreach (int r, sIDs)
			foreach (int c, tIDs)
				disM[r][c] = 0;
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
			if (M[i][j] != INVALID_VALUE && M[i][j] > maxDis)
				maxDis = M[i][j];
		}
	}

	// Normalize
	for(int i = 0; i < sN; i++){
		for (int j = 0; j < tN; j++)
		{
			if (M[i][j] != INVALID_VALUE)
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

		float value;
		if (disM[sourceID][j] == INVALID_VALUE)
			value = 0;
		else
			value = (1 - disM[sourceID][j]) / 4 + 0.75;
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
		QString sr =  sg->nodes[r]->id;
		QString tc =  tg->nodes[c]->id;

		std::set<QString> sSet, tSet;
		sSet.insert(sr);
		tSet.insert(tc);
		this->correspondences.push_back(std::make_pair(sSet, tSet));

		// Write into the file
		outF << sr.toStdString() << '\t' << tc.toStdString() << '\n';

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
		float threshold  = disMatrix[r][c] * 0.98;

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
		std::set<QString> sSet, tSet;
		sSet.insert(sg->nodes[r]->id);
		tSet.insert(tg->nodes[c]->id);

		if (c_many.size() > r_many.size()) // r <-> {c_i}
		{
			outF << sg->nodes[r]->id.toStdString() << "\t:\t"
				 << tg->nodes[c]->id.toStdString() << '\t';
			
			foreach(int ci, c_many)
			{
				tSet.insert(tg->nodes[ci]->id);

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
				sSet.insert(sg->nodes[ri]->id);

				outF << sg->nodes[ri]->id.toStdString() << '\t';

				for (int j = 0; j < tN; j++) // Row r
					disMatrix[ri][j] = INVALID_VALUE;
			}

			outF << sg->nodes[r]->id.toStdString() << "\t:\t"
				 << tg->nodes[c]->id.toStdString() << '\n';
		}

		// Save results
		this->correspondences.push_back(std::make_pair(sSet, tSet));
		
		// Remove r and c in the disMatrix
		for (int i = 0; i < sN; i++) // Column c
			disMatrix[i][c] = INVALID_VALUE;
		for (int j = 0; j < tN; j++) // Row r
			disMatrix[r][j] = INVALID_VALUE;
	}

	outF.close();
}


void GraphCorresponder::correspondTwoNodes( Structure::Node *sNode, Structure::Node *tNode )
{
	if (sNode->type() != tNode->type())
	{
		qDebug() << "Two nodes with different types cannot be corresponded. ";
		return;
	}

	if (sNode->type() == Structure::SHEET)
		correspondTwoSheets((Structure::Sheet*) sNode, (Structure::Sheet*) tNode);
	else
		correspondTwoCurves((Structure::Curve*) sNode, (Structure::Curve*) tNode);
}


void GraphCorresponder::correspondTwoCurves( Structure::Curve *sCurve, Structure::Curve *tCurve )
{
	std::vector<Vector3> sCtrlPoint = sCurve->controlPoints();
	std::vector<Vector3> tCtrlPoint = tCurve->controlPoints();

	// Euclidean for now, can use Geodesic distance instead if need
	Vector3 scenter = sCurve->center();
	Vector3 sfront = sCtrlPoint.front() - scenter;
	Vector3 tcenter = tCurve->center();
	Vector3 tfront = tCtrlPoint.front() - tcenter;
	Vector3 tback = tCtrlPoint.back() - tcenter;

	float f2f = (sfront - tfront).norm();
	float f2b = (sfront - tback).norm();

	if (f2f > f2b)
	{
		// Flip the target
		std::vector<Scalar> tCtrlWeight = tCurve->controlWeights();
		std::reverse(tCtrlPoint.begin(), tCtrlPoint.end());
		std::reverse(tCtrlWeight.begin(), tCtrlWeight.end());

		NURBSCurve newCurve(tCtrlPoint, tCtrlWeight);
		tCurve->curve = newCurve;

		// Update the coordinates of links
		foreach( Structure::Link * l, tg->getEdges(tCurve->id) )
		{
			l->setCoord(tCurve->id, inverseCoords( l->getCoord(tCurve->id) ));
		}
	}
}

// Helper function
template <class Type>
std::vector<std::vector<Type> > transpose(const std::vector<std::vector<Type> > data) 
{
	// this assumes that all inner vectors have the same size and
	// allocates space for the complete result in advance
	std::vector<std::vector<Type> > result(data[0].size(), std::vector<Type>(data.size()));
	for (int i = 0; i < (int)data[0].size(); i++){
		for (int j = 0; j < (int)data.size(); j++) 
		{
			result[i][j] = data[j][i];
		}
	}

	return result;
}


void GraphCorresponder::correspondTwoSheets( Structure::Sheet *sSheet, Structure::Sheet *tSheet )
{
	// Old properties
	NURBSRectangle &oldRect = tSheet->surface;
	int uDegree = oldRect.GetDegree(0);
	int vDegree = oldRect.GetDegree(1);
	bool uLoop = oldRect.IsLoop(0);
	bool vLoop = oldRect.IsLoop(1);
	bool uOpen = oldRect.IsOpen(0);
	bool vOpen = oldRect.IsOpen(1);
	bool isModified = false;
	bool isUVFlipped = false;

	// Control points and weights
	Array2D_Vector3 sCtrlPoint = sSheet->surface.mCtrlPoint;
	Array2D_Real sCtrlWeight = sSheet->surface.mCtrlWeight;

	Array2D_Vector3 tCtrlPoint = tSheet->surface.mCtrlPoint;
	Array2D_Real tCtrlWeight = tSheet->surface.mCtrlWeight;

	Array2D_Vector3 tCtrlPointNew;
	Array2D_Real tCtrlWeightNew;

	Vector3 scenter = sSheet->center();
	Vector3 tcenter = tSheet->center();

	// Get the extreme points.
	Vector3 s00 = sCtrlPoint.front().front();
	Vector3 s01 = sCtrlPoint.front().back();
	Vector3 s10 = sCtrlPoint.back().front();
	Vector3 sU = s10 - s00;
	Vector3 sV = s01 - s00;

	Vector3 t00 = tCtrlPoint.front().front();
	Vector3 t01 = tCtrlPoint.front().back();
	Vector3 t10 = tCtrlPoint.back().front();
	Vector3 tU = t10 - t00;
	Vector3 tV = t01 - t00;

	// Flip if need
	Vector3 sUV = cross(sU, sV);
	Vector3 tUV = cross(tU, tV);
	if (dot(sUV, tUV) < 0)
	{
		// Reverse the target along u direction
		std::reverse(tCtrlPoint.begin(), tCtrlPoint.end());
		std::reverse(tCtrlWeight.begin(), tCtrlWeight.end());

		// Update tU
		tU = -tU;
		tUV = -tUV;
		isModified = true;

		// Update the coordinates of links
		foreach( Structure::Link * l, tg->getEdges(tSheet->id) ){
			std::vector<Vec4d> oldCoord = l->getCoord(tSheet->id), newCoord;
			foreach(Vec4d c, oldCoord) newCoord.push_back(Vec4d(1-c[0], c[1], c[2], c[3]));
			l->setCoord(tSheet->id, newCoord);
		}
	}

	// Rotate if need
	Scalar cosAngle = dot(sU.normalized(), tU.normalized());
	Scalar cos45 = sqrtf(2.0) / 2;

	// Do Nothing
	if (cosAngle > cos45)
	{
		tCtrlPointNew = tCtrlPoint;
		tCtrlWeightNew = tCtrlWeight;
	}
	// Rotate 180 degrees
	else if (cosAngle < - cos45)
	{
		//  --> sV				tU
		// |					|
		// sU             tV <--
		// By flipping along both directions
		std::reverse(tCtrlPoint.begin(), tCtrlPoint.end());
		std::reverse(tCtrlWeight.begin(), tCtrlWeight.end());

		for (int i = 0; i < (int)tCtrlPoint.size(); i++)
		{
			std::reverse(tCtrlPoint[i].begin(), tCtrlPoint[i].end());
			std::reverse(tCtrlWeight[i].begin(), tCtrlWeight[i].end());
		}

		// The new control points and weights
		tCtrlPointNew = tCtrlPoint;
		tCtrlWeightNew = tCtrlWeight;
		isModified = true;

		// Update the coordinates of links
		foreach( Structure::Link * l, tg->getEdges(tSheet->id) ){
			std::vector<Vec4d> oldCoord = l->getCoord(tSheet->id), newCoord;
			foreach(Vec4d c, oldCoord) newCoord.push_back(Vec4d(1-c[0], 1-c[1], c[2], c[3]));
			l->setCoord(tSheet->id, newCoord);
		}
	}
	// Rotate 90 degrees 
	else
	{
		Vector3 stU = cross(sU, tU);
		if (dot(stU, sUV) >= 0)
		{
			//  --> sV		tV
			// |			|
			// sU           --> tU
			// Transpose and reverse along U
			tCtrlPointNew = transpose<Vector3>(tCtrlPoint);
			tCtrlWeightNew = transpose<Scalar>(tCtrlWeight);

			std::reverse(tCtrlPointNew.begin(), tCtrlPointNew.end());
			std::reverse(tCtrlWeightNew.begin(), tCtrlWeightNew.end());

			// Update the coordinates of links
			foreach( Structure::Link * l, tg->getEdges(tSheet->id) ){
				std::vector<Vec4d> oldCoord = l->getCoord(tSheet->id), newCoord;
				foreach(Vec4d c, oldCoord) newCoord.push_back(Vec4d(1- c[1], c[0], c[2], c[3]));
				l->setCoord(tSheet->id, newCoord);
			}
		}
		else
		{
			//  --> sV	tU<--
			// |			 |
			// sU			tV
			// Reverse along U and Transpose
			std::reverse(tCtrlPoint.begin(), tCtrlPoint.end());
			std::reverse(tCtrlWeight.begin(), tCtrlWeight.end());

			tCtrlPointNew = transpose<Vector3>(tCtrlPoint);
			tCtrlWeightNew = transpose<Scalar>(tCtrlWeight);

			// Update the coordinates of links
			foreach( Structure::Link * l, tg->getEdges(tSheet->id) ){
				std::vector<Vec4d> oldCoord = l->getCoord(tSheet->id), newCoord;
				foreach(Vec4d c, oldCoord) newCoord.push_back(Vec4d(c[1], 1- c[0], c[2], c[3]));
				l->setCoord(tSheet->id, newCoord);
			}
		}

		isModified = true;
		isUVFlipped = true;
	}

	// Create a new sheet if need
	if (isModified)
	{
		NURBSRectangle newRect; 
		if (isUVFlipped)
			newRect = NURBSRectangle(tCtrlPointNew, tCtrlWeightNew, vDegree, uDegree, vLoop, uLoop, vOpen, uOpen);
		else
			newRect = NURBSRectangle(tCtrlPointNew, tCtrlWeightNew, uDegree, vDegree, uLoop, vLoop, uOpen, vOpen);

		tSheet->surface = newRect;
	}
}

void GraphCorresponder::computeCorrespondences()
{
	// Landmarks
	loadLandmarks();

	// Part-to-Part correspondences
	correspondences.clear();
	findOneToOneCorrespondences();

	// Adjust the frames for corresponded parts
	// Then Point-to-Point correspondences can be easily retrieved via parameterized NURBS. 
	for (int i = 0; i < (int)correspondences.size(); i++)
	{
		std::set<QString> &sSet = correspondences[i].first;
		std::set<QString> &tSet = correspondences[i].second;

		// One to many
		if (sSet.size() == 1)
		{
			Structure::Node *sNode = sg->getNode(*sSet.begin());
			foreach(QString tID, tSet)
			{
				Structure::Node *tNode = tg->getNode(tID);
				correspondTwoNodes(sNode, tNode);
			}
		}
		// Many to one
		else if (tSet.size() == 1)
		{
			Structure::Node *tNode = tg->getNode(*tSet.begin());
			foreach(QString sID, sSet)
			{
				Structure::Node *sNode = sg->getNode(sID);
				correspondTwoNodes(sNode, tNode);
			}
		}
		// Many to many
		else
		{
			qDebug() << "Many to Many correspondence happened. ";
		}
	}
}

void GraphCorresponder::saveLandmarks()
{
	QString sgName = sg->property["name"].toString().section('\\', -1).section('.', 0, 0);
	QString tgName = tg->property["name"].toString().section('\\', -1).section('.', 0, 0);

	QString filename = "landmarks_" + sgName + "_" + tgName + ".txt";

	std::ofstream outF(filename.toStdString(), std::ios::out);

	foreach (SET_PAIR set2set, landmarks)
	{
		outF << set2set.first.size() << '\t';;
		foreach (QString strID, set2set.first)
			outF << strID.toStdString() << '\t';
		outF << '\n';

		outF << set2set.second.size() << '\t';
		foreach (QString strID, set2set.second)
			outF << strID.toStdString() << '\t';
		outF << "\n\n";
	}

	outF.close();
}

void GraphCorresponder::loadLandmarks()
{
	QString sgName = sg->property["name"].toString().section('\\', -1).section('.', 0, 0);
	QString tgName = tg->property["name"].toString().section('\\', -1).section('.', 0, 0);

	QString filename = "landmarks_" + sgName + "_" + tgName + ".txt";

	std::ifstream inF(filename.toStdString(), std::ios::in);

	landmarks.clear();
	sIsLandmark.resize(sg->nodes.size(), false);
	tIsLandmark.resize(tg->nodes.size(), false);

	int n;
	std::string strID;
	while(!inF.eof())
	{
		std::set<QString> sParts, tParts;

		inF >> n;
		for (int i = 0; i < n; i++)
		{
			inF >> strID;
			sParts.insert(QString(strID.c_str()));
		}

		inF >> n;
		for (int i = 0; i < n; i++)
		{
			inF >> strID;
			tParts.insert(QString(strID.c_str()));
		}

		this->addLandmarks(sParts, tParts);
	}

	inF.close();
}

