#include "Scorer.h"

// bSource == true means that the id is from source relation groups
std::vector<Structure::Node*> Scorer::findNodes(QString id, Structure::Graph *graph, QVector<PART_LANDMARK> &corres, bool bSource)
{
    std::vector<Structure::Node*> result;

    if ( bSource)
    {
        for ( int i = 0; i < (int) corres.size(); ++i)
        {
            QVector<QString> ids = corres[i].first;
            int numCorres = corres[i].second.size();
            if ( 1 == numCorres && ids[0] == id)
            {
                Structure::Node* tmpNode = graph->getNode(id);
                if ( tmpNode ==NULL)
                {
                    for ( int j = 0; j < (int) numCorres; ++j)
                    {
                        result.push_back( graph->getNode(corres[i].second[j]) );
                    }
                }
                else
                {
                    result.push_back( graph->getNode(id) );
                }
                return result;
            }
            else if ( 1 < numCorres )
            {
                if ( ids[0].startsWith( id + "_", Qt::CaseSensitive) )
                {
                    Structure::Node* tmpNode = graph->getNode(ids[0]);
                    if ( tmpNode ==NULL)
                    {
                        result.push_back( graph->getNode(corres[i].second[0]) );
                    }
                    else
                    {
                        for ( int j = 0; j < (int) ids.size(); ++j)
                        {
                            result.push_back( graph->getNode(ids[j]));
                        }
                    }
                    return result;
                }
            }
        }
    }
    else // is target
    {
        for ( int i = 0; i < (int) corres.size(); ++i)
        {
            QVector<QString> ids1 = corres[i].first;
            QString id2 = corres[i].second[0];
            if ( id2 == id)
            {
                Structure::Node* tmpNode = graph->getNode(ids1[0]);
                if ( tmpNode ==NULL)
                {
                    result.push_back( graph->getNode(id2) );
                }
                else
                {
                    for ( int j = 0; j < (int) ids1.size(); ++j)
                    {
                        result.push_back( graph->getNode(ids1[j]) );
                    }
                }
                return result;
            }
        }
        //
        Structure::Node* tmpNode = graph->getNode(id + "_null");
        if ( tmpNode != NULL)
        {
            result.push_back( tmpNode );            
        }
		return result;
    }

    result.push_back( graph->getNode(id) );
    return result;
}
Eigen::MatrixXd Scorer::node2matrix(Structure::Node* node, int pointLevel)
{
	std::vector<Eigen::Vector3d> nodeCptsV;
	int tmp = extractCpts( node, nodeCptsV, pointLevel);
	Eigen::MatrixXd nodeCptsM;
	vectorPts2MatrixPts(nodeCptsV, nodeCptsM);
	return nodeCptsM;
}
