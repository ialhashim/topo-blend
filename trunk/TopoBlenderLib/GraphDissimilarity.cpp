#include "GraphDissimilarity.h"

#include <Eigen/Eigenvalues>

GraphDissimilarity::GraphDissimilarity( Structure::Graph *graphInstance )
{
	N = graphInstance->nodes.size();

	foreach(Structure::Node * node, graphInstance->nodes)
		nodeIndex[node->id] = nodeIndex.size();
}

void GraphDissimilarity::addGraph( Structure::Graph *g )
{
	/// Normalized Laplacian matrix (symmetric):
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
	graphs.push_back( g );

	// Compute eigenvalues and eigenvectors
	SelfAdjointEigenSolver<MatrixXd> es( L );
	eigenvalues.push_back( es.eigenvalues() );
	eigenvectors.push_back( es.eigenvectors() );
}

void GraphDissimilarity::addGraphs( QVector<Structure::Graph*> fromGraphs )
{
	foreach(Structure::Graph* g, fromGraphs) 
		addGraph(g);
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
			double sum = (lamda[i] + mu[j]);
			if(sum == 0) continue;

			double quotientTerm = pow(lamda[i] - mu[j], 2) / sum;
			double dotproductTerm = pow(u.col(i).dot(v.col(j)), 2);
			d += quotientTerm * dotproductTerm;
		}
	}

	return d;
}

QVector<double> GraphDissimilarity::competeDissimilar( int gidx, int startidx )
{
	QVector<double> scores;

	for(int i = startidx; i < graphs.size(); i++)
		scores.push_back( compute(gidx, i) );

	return scores;
}

QVector<Structure::Graph *> GraphDissimilarity::dissimilar( int k )
{
    QVector<Structure::Graph *> result;

    return result;
}

void GraphDissimilarity::outputResults()
{

}

Structure::Graph * GraphDissimilarity::fromAdjFile(QString filename)
{
	Structure::Graph * g = new Structure::Graph;
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return g;
	QFileInfo fileInfo(file.fileName());
	QTextStream in(&file);
	int numNodes = 0;
	int lineNumber = 0;
	QVector< QVector<int> > allEdges;
	while (!in.atEnd()){
		QString line = in.readLine();
		if(line.isEmpty()) continue;
		// Convert line to edges
		QStringList e = line.split("\t");
		// Get number of nodes
		numNodes = e.size();
		QVector<int> edges;
		for(int i = 0; i < e.size(); i++){
			int edgeVal = e[i].toInt();
			if( edgeVal == 0 || i < lineNumber ) continue;
			edges.push_back(i);
		}
		allEdges.push_back(edges);
		lineNumber++;
	}
	// Add temp nodes
	for(int i = 0; i < numNodes; i++){
		NURBS::NURBSCurved curve = NURBS::NURBSCurved::createCurve(Vector3(0,0,0), Vector3(0,0,1));
		g->addNode( new Structure::Curve(curve, QString("%1").arg(i)) );
	}
	// Add edges
	for(int i = 0; i < numNodes; i++){
		Structure::Node * n1 = g->nodes[i];
		QVector<int> edges = allEdges[i];
		foreach(int j, edges){
			Structure::Node * n2 = g->nodes[j];
			g->addEdge(n1,n2);
		}
	}
	file.close();
	return g;
}
