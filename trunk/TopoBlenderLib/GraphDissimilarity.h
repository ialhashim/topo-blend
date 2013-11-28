#pragma once

#include "StructureGraph.h"

class GraphDissimilarity
{
public:
	GraphDissimilarity( Structure::Graph * graphInstance );
    void addGraph( Structure::Graph * g );

    QVector<Structure::Graph*> dissimilar( int k );

	void compute();

private:
    // Locals
	int N;
	QMap<QString, int> nodeIndex;

	// Input:
    QVector<Structure::Graph*> graphs;
    QVector< MatrixXd > laplacians; // normalized Laplacian matrix
	QVector< VectorXd > eigenvalues;
	QVector< MatrixXd > eigenvectors;

    // Output: dissimilarity w.r.t. source and target
    QVector< QPair<double,double> > dissimilarity;

	// Compute dissimilarity between two graphs of input
	double compute( int g1, int g2 );

    // DEBUG:
    void outputResults();
};
