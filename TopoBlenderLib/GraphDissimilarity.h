#pragma once

#include "StructureGraph.h"

class GraphDissimilarity
{
public:
	GraphDissimilarity( Structure::Graph * graphInstance );
    void addGraph( Structure::Graph * g );
	void addGraphs( QVector<Structure::Graph*> fromGraphs );

    // Locals
	int N;
	QMap<QString, int> nodeIndex;

	// Input:
    QVector<Structure::Graph*> graphs;
    
	// Eigen decomposition of the normalized Laplacian matrix of input graphs
	QVector< VectorXd > eigenvalues;
	QVector< MatrixXd > eigenvectors;

	// Compute dissimilarity between two graphs of input
	double compute( int g1, int g2 );
	QVector<double> computeDissimilar( int gidx, int startidx = 2 );

    // DEBUG:
    void outputResults();
	static Structure::Graph * fromAdjFile(QString filename);
};
