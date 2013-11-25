#include "PairRelationScorer.h"
void PairRelationScorer::isParalOrthoCoplanar(PairRelation &cpr)
{
	Structure::Node * n1 = cpr.n1; Structure::Node * n2 = cpr.n2;

    Vector_3 d1, d2;
    bool iscurve1 = node2direction(n1, d1);
    bool iscurve2 = node2direction(n2, d2);
    double err(0.0);
    if ( iscurve1 == iscurve2 )
    {
        if ( cpr.type == PARALLEL)
        {
            err = errorOfParallel(d1, d2);
        }
        else if ( cpr.type == ORTHOGONAL)
        {
            err = errorOfOrthogonal(d1, d2);
        }		
    }
    else// curve & sheet
    {
        if ( cpr.type == ORTHOGONAL)
        {
            err = errorOfParallel(d1, d2);
        }
    }

    cpr.deviation = fixDeviationByPartName(n1->id, n2->id, err);
    cpr.diameter = computePairDiameter(cpr);
}
double PairRelationScorer::evaluate(QVector<QVector<PairRelation> > &pairss, QVector<PART_LANDMARK> &corres)
{
	double resultMean(0.0), resultMax(0.0); int num(0);
	double dist = graph_->bbox().diagonal().norm();

	for ( int j = 0; j < pairss.size(); ++j)
	{
		QVector<PairRelation>& pairs = pairss[j];
		//if (logLevel_>0)
		//{
		//	if ( j == 0 )
		//		logStream_ << "pairs in source shape \n";
		//	else
		//		logStream_ << "\n\n pairs in target shape \n";
		//}
		for ( int i = 0; i < pairs.size(); ++i)
		{
			PairRelation& pr = pairs[i];

			std::vector<Structure::Node*> nodes1 = findNodesInB(pr.n1->id, graph_, corres, j==0);
			std::vector<Structure::Node*> nodes2 = findNodesInB(pr.n2->id, graph_, corres, j==0);
			//if (logLevel_>0)
			//{
			//	logStream_ << pr << "\n correspond to: \n";
			//}
			for ( int i1 = 0; i1 < (int) nodes1.size(); ++i1)
			{
				for ( int i2 = 0; i2 < (int) nodes2.size(); ++i2)
				{
					PairRelation cpr(nodes1[i1], nodes2[i2]);
					cpr.type = pr.type;
					isParalOrthoCoplanar(cpr);					
					
					if (cpr.deviation < pr.deviation)
						cpr.deviation = 0;
					else
					{
						cpr.deviation = cpr.deviation - pr.deviation;
						cpr.deviation = cpr.deviation * cpr.diameter / dist;
					}

					resultMean += cpr.deviation;
					++num;
					if ( resultMax < cpr.deviation)
					{
						resultMax = cpr.deviation;
					}					
					
					if (logLevel_>0 && cpr.deviation > 0) //
					{
						logStream_ << num << "\n";
						if ( j == 0 )
							logStream_ << "pairs in source shape \n";
						else
							logStream_ << "pairs in target shape \n";

						logStream_ << pr << "correspond to: \n";
						logStream_ << "<" << nodes1[i1]->id << ", " << nodes2[i2]->id << "> size <" 
							<< nodes1[i1]->bbox().diagonal().norm() << ", " << nodes2[i2]->bbox().diagonal().norm()
							<< ", "<< dist << ">:" << cpr.deviation << "\n\n";
					}
					
				}
			}
		}
	}

	if (logLevel_>0 )
	{
		logStream_ << "mean score: " << 1-resultMean/num << "\n";
		logStream_ << "max score: " << 1-resultMax << "\n";		
	}
	return 1-resultMax;
}