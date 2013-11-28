#include "GraphDissimilarity.h"

#include <Eigen/Eigenvalues>

GraphDissimilarity::GraphDissimilarity( Structure::Graph *graphInstance )
{
	N = graphInstance->nodes.size();

	foreach(Structure::Node * node, graphInstance->nodes){
		nodeIndex[node->id] = nodeIndex.size();
	}
}

void GraphDissimilarity::addGraph( Structure::Graph *g )
{
	// Normalized Laplacian matrix (symmetric)
	MatrixXd L = MatrixXd::Zero(N,N);
	
	// Diagonal entires
	foreach(Structure::Node * node, g->nodes){
		int i = nodeIndex[node->id];
		L(i,i) = (g->valence(node) == 0) ? 0 : 1;
	}

	// i != j entires
	foreach(Structure::Link * edge, g->edges)
	{
		int i = nodeIndex[edge->n1->id];
		int j = nodeIndex[edge->n2->id];
		if(i == j) continue; // just in case the graph is weird

		int di = g->valence(edge->n1);
		int dj = g->valence(edge->n2);

		L(i,j) = L(j,i) = -(1.0 / (std::sqrt( double(di * dj) )));
	}

	// Store input
	laplacians.push_back( L );
	graphs.push_back( g );

	// Compute eigenvalues and eigenvectors
	SelfAdjointEigenSolver<MatrixXd> es( L );
	eigenvalues.push_back( es.eigenvalues() );
	eigenvectors.push_back( es.eigenvectors() );
}

double GraphDissimilarity::compute( int g1, int g2 )
{
	double d = 0;

	VectorXd lamda = eigenvalues[g1];
	VectorXd mu = eigenvalues[g2];

	MatrixXd u = eigenvectors[g1];
	MatrixXd v = eigenvectors[g2];

	for(int i = 0; i < N; i++)
	{
		for(int j = 0; j < N; j++)
		{
			double term1 = pow(lamda[i] - mu[j], 2) / (lamda[i] + mu[j]);
			double term2 = pow(u.col(i).dot(v.col(j)), 2);
			d += term1 * term2;
		}
	}

	return d;
}

void GraphDissimilarity::compute()
{

}

QVector<Structure::Graph *> GraphDissimilarity::dissimilar( int k )
{
    QVector<Structure::Graph *> result;

    return result;
}

void GraphDissimilarity::outputResults()
{

}
