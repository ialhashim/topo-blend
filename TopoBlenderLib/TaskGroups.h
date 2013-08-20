#pragma once
#include <QSet>
#include <QStack>

#include "Task.h"

struct TaskGroups{

	static QVector< QList<Task*> > split( QList<Task*> list, Structure::Graph * graph )
	{
		QVector< QList<Task*> > groups;

		// Visited tasks or not
		QMap<Node*,bool> visited;
		foreach(Task* t, list) visited[t->node()] = false;

		// Tasks from nodes
		QMap<Node*,Task*> taskMap;
		foreach(Task* t, list) taskMap[t->node()] = t;

		// Start splitting into groups
		foreach(Task* t, list)
		{
			QList<Task*> curSet;

			QStack<Task*> tasksToVisit;
			tasksToVisit.push( t );
			if( visited[t->node()] ) continue;

			while( !tasksToVisit.empty() )
			{
				Task * curTask = tasksToVisit.pop();
				Structure::Node * curNode = curTask->node();
				if(visited[curNode]) continue;

				visited[curNode] = true;
				curSet.push_back( curTask );

				QVector<Node*> adjNodes = graph->adjNodes(curNode);

				foreach( Structure::Node * adj, adjNodes )
				{
					if( visited.keys().contains(adj) && !visited[adj] )
					{
						Task * adjTask = taskMap[adj];

						tasksToVisit.push( adjTask );
					}
				}
			}

			groups.push_back( curSet );
		}

		return groups;
	}

	struct Graph
	{
		typedef QSet< Structure::Node* > QSetNode;
		typedef QVector<Structure::Node *> QVectorNode;

		QSetNode nodes;
		QMap< Structure::Node*, QSetNode > edges;
		QMap< Structure::Node*, QMap<QString,QVariant> > nodeProperty;

		Graph(QList<Task*> list, Structure::Graph * graph)
		{
			// Tasks from nodes
			QMap<Node*,Task*> taskMap;
			foreach( Task* t, list ) taskMap[t->node()] = t;

			// Add to graph
			foreach( Task* t, list )
			{
				Structure::Node * n = t->node();
				nodes.insert( n );
				nodeProperty[n]["task"].setValue( taskMap[n] );

				foreach( Structure::Node * adj, graph->adjNodes(n) )
				{
					if( !taskMap.keys().contains(adj) ) 
					{
						nodeProperty[n]["isBoundary"] = true;
						continue;
					}

					// Add node
					nodes.insert(adj);
					nodeProperty[n]["task"].setValue( taskMap[n] );

					// Add edges (undirected)
					edges[n].insert(adj);
					edges[adj].insert(n);
				}
			}

			/// Drop the "boundary" property when:

			// 1) For group of two, one will not be
			if(list.size() == 2)
			{
				Node * first = list.front()->node();
				Node * last = list.back()->node();
				Structure::Node * n = graph->valence(first) < graph->valence(last) ? first : last;
				nodeProperty[n]["isBoundary"] = false;
			}

			// 2) For boundary nodes connecting with many boundary
			foreach(Structure::Node * n, boundaryNodes()){

				// Count number of neighboring boundaries 
				int numBoundary = 0;
				foreach( Structure::Node * adj, graph->adjNodes(n) ){
					if(nodeProperty[adj]["isBoundary"].toBool())
						numBoundary++;
				}

				// Drop property
				if(numBoundary == 1)
					nodeProperty[n]["isBoundary"] = false;
			}
		}

		void clearProperty(QString propertyName, QVariant value){
			foreach(Structure::Node* n, nodes){
				nodeProperty[n][propertyName] = value;
			}
		}

		void removeNode(Structure::Node * n){
			foreach(Structure::Node * adj, edges[n])
			{
				// Assign neighbor as boundary
				nodeProperty[adj]["isBoundary"] = true;

				// Erase edge
				edges[adj].remove(n);
				edges[n].remove(adj);
			}

			// Erase node
			nodes.remove(n);
		}

		QVector<Structure::Node *> boundaryNodes(){
			QVector<Structure::Node *> b;
			foreach(Structure::Node* n, nodes){
				if(nodeProperty[n]["isBoundary"].toBool())
					b.push_back(n);
			}
			return b;
		}

		QVector< QList<Task*> > peel()
		{
			QVector< QList<Task*> > layers;

			while( nodes.size() )
			{
				QList<Task*> btasks;
				QVectorNode bnodes = boundaryNodes();

				foreach(Structure::Node * bnode, bnodes){
					btasks.push_back(nodeProperty[bnode]["task"].value<Task*>());
					removeNode(bnode);
				}
				
				if(!btasks.size()) break;

				layers.push_back( btasks );
			}

			return layers;
		}
	};

};
