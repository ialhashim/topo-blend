#include "GroupRelationDetector.h"

void GroupRelationDetector::detect(QVector<PairRelation>& prRelations)
{
    detectSymmGroupByRefPair( prRelations);
    detectRefSymmGroupByTransPair( prRelations);
    //detectCoplanarGroup( prRelations);
    //mergeCoplanarGroup();
    removeRedundantGroup();
}

void GroupRelationDetector::detectSymmGroupByRefPair(QVector<PairRelation>& prRelations)
{
    int nPrs = prRelations.size();
    for (int i=0; i<nPrs; ++i)
        prRelations[i].tag = false;

    double dotVal1(0.0);
    ///////////
    // build group from REF pair with assumption that no TRANS pair is detected as REF pair
    for (int i=0; i<nPrs; ++i)
    {
		//////////////////// detect group
        PairRelation& pr1 = prRelations[i];
        if ( pr1.tag || REF.compare(pr1.type)) continue;
                
		QSet<QString> ids;
		ids.insert( pr1.n1->id);			ids.insert(pr1.n2->id);
		pr1.tag = true;
		
		for (int j=i+1; j<nPrs; ++j)
        {
            PairRelation& pr2 = prRelations[j];
            if ( pr2.tag || REF.compare(pr2.type)) continue;
            if ( isIntersected(ids, pr2) )
            {
				ids.insert(pr2.n1->id);	ids.insert(pr2.n2->id);
                pr2.tag = true;
			}
		}

		//////////////////// sort
		GroupRelation gr;
		gr.ids.push_back( *ids.begin() );
		ids.erase(ids.begin());		
		
		Structure::Node *n1, *n2;
		while(!ids.isEmpty())
		{
			n1 = graph_->getNode(gr.ids.last());
			Vector_3 v1, v2;
			node2direction(n1,v1); 

			double angle = -2;
			QSet<QString>::iterator itor;
			for ( QSet<QString>::iterator it = ids.begin(); it != ids.end(); ++it)
			{
				n2 = graph_->getNode(*it);
				node2direction(n2,v2); 				
				dotVal1 = v1.dot(v2);
				
				if ( angle < dotVal1)
				{
					angle = dotVal1;
					itor = it;
				}
			}
			gr.ids.push_back( *itor);
			ids.erase(itor);
		}

		//////////////////// detect axis
		int n = gr.ids.size();
		gr.direction = Eigen::Vector3d(0.0,0.0,0.0);
		for (int j=0; j<n; ++j)
		{
			n1 = graph_->getNode(gr.ids[j]);
			n2 = graph_->getNode(gr.ids[(j+1)%n]);

			Vector_3 v1, v2;
			node2direction(n1,v1); node2direction(n2,v2);
            Vector_3 dir = cross_product(v1, v2);
			dir.normalize();
			if ( dir.dot(gr.direction) < 0)
				gr.direction = gr.direction - dir;
			else
				gr.direction = gr.direction + dir;
		}
		gr.direction.normalize();

		////////////////////
        gr.diameter = computeGroupDiameter(gr);
		computeGroupCenter(gr);

        ////////////////
		gr.note = REF;
        gr.deviation = computeAxisSymmetryGroupDeviationSortParts(gr, pointLevel_);
		gr.type = AXIS_SYMMETRY;
		if ( this->logLevel_ > 0)
		{
			logStream_ << gr;
			logStream_ << "Normalized axis gourp deviation: " << gr.deviation / this->normalizeCoef_ << "\n";
		}        
        if ( gr.deviation / this->normalizeCoef_ < thAxisGroup_)
        {			
            groupRelations_.push_back(gr);
        }
        else
        {            
            gr.deviation = computeRefSymmetryGroupDeviation(gr,pointLevel_);
			gr.type = REF_SYMMETRY;
			if ( this->logLevel_ > 0)
			{
				logStream_ << gr;
				logStream_ << "Normalized ref gourp deviation: " << gr.deviation / this->normalizeCoef_ << "\n";
			}
            if ( gr.deviation / this->normalizeCoef_ < thRefGroup_)
            {				
                groupRelations_.push_back(gr);
            }
        }

    }
    ////////////// remove pairs have been put into groups
    prRelations.erase( std::remove_if(prRelations.begin(), prRelations.end(), isTaged), prRelations.end() );
}

void GroupRelationDetector::detectRefSymmGroupByTransPair(QVector<PairRelation>& prRelations)
{
	if ( this->logLevel_ > 0)
	{
		logStream_ << "GroupRelationDetector::detectSymmGroupByTransPair called! \n";
	}

    int nPrs = prRelations.size();
    for (int i=0; i<nPrs; ++i)
        prRelations[i].tag = false;

    ///////////
    // build group from pair
    for (int i=0; i<nPrs; ++i)
    {
        PairRelation& pr1 = prRelations[i];
        if ( pr1.tag || TRANS.compare(pr1.type)) continue;

        QSet<QString> ids;
        GroupRelation gr;
        gr.type = symmTypeOfTansPair(pr1);
        if ( NONE != gr.type)
        {
            ids.insert( pr1.n1->id);
            ids.insert( pr1.n2->id);
            pr1.tag = true;
        }
        else
            continue;

        ///////////////
        for (int j=i+1; j<nPrs; ++j)
        {
            PairRelation& pr2 = prRelations[j];
            if ( pr2.tag || TRANS.compare(pr2.type)) continue;

            if ( isIntersected(ids, pr2) )
            {
                QString type2 = symmTypeOfTansPair(pr2);
                if ( NONE != type2)
                {
                    if ( REF_SYMMETRY == gr.type)
                    {
                        if ( std::abs( pr1.trans_vec.norm()-pr2.trans_vec.norm() ) >
                            pr1.n1->bbox().diagonal().norm()*thRefGroup_*2 )
                        {
                            continue;
                        }
                    }
                    ids.insert(pr2.n1->id);
                    ids.insert(pr2.n2->id);
                    pr2.tag = true;
                }
            }
        }

        ///////////////
        computeTransGroupInfo(gr, ids);
		gr.note = TRANS;       
        gr.type = REF_SYMMETRY;
		gr.deviation = computeRefSymmetryGroupDeviation(gr, pointLevel_);
		if ( this->logLevel_ > 0)
		{
			logStream_ << gr;
			logStream_ << "Normalized ref gourp deviation: " << gr.deviation / this->normalizeCoef_ << "\n";
		}
        if ( gr.deviation / this->normalizeCoef_ < thRefGroup_)
        {
            groupRelations_.push_back(gr);
        }

    }
    ////////////// remove pairs have been put into groups
    prRelations.erase( std::remove_if(prRelations.begin(), prRelations.end(), isTaged), prRelations.end() );
}

void GroupRelationDetector::removeRedundantGroup()
{
    for ( int i = 0; i < (int) groupRelations_.size(); ++i)
    {
        groupRelations_[i].tag = false;
    }
    for ( int i = 0; i < (int) groupRelations_.size(); ++i)
    {
        if ( groupRelations_[i].tag )
            continue;

        for ( int j = i+1; j < (int) groupRelations_.size(); ++j)
        {
            if ( isSubgroup(groupRelations_[i].ids, groupRelations_[j].ids) )
                groupRelations_[j].tag = true;
        }
    }
    groupRelations_.erase( std::remove_if(groupRelations_.begin(), groupRelations_.end(), isTagedGr), groupRelations_.end() );
}
bool GroupRelationDetector::isIntersected(QSet<QString>& ids, PairRelation& pr2)
{
    for ( QSet<QString>::iterator it = ids.begin(); it != ids.end(); ++it)
    {
        if ( *it == pr2.n1->id || *it == pr2.n2->id)
            return true;
    }
    return false;
}

bool GroupRelationDetector::isSubgroup(QVector<QString>& ids1, QVector<QString>& ids2)
{
    int count(0);
    for ( int i = 0; i < (int) ids2.size(); ++i)
    {
        for ( int j = 0; j < (int) ids1.size(); ++j)
        {
            if ( ids2[i] == ids1[j] )
            {
                ++count;
                break;
            }
        }
    }
    if ( ids2.size() == count)
        return true;
    else
        return false;
}
QString GroupRelationDetector::symmTypeOfTansPair(PairRelation& pr1)
{
    QString result(NONE);
    Vector_3 d1;
    bool iscurve1 = node2direction(pr1.n1, d1);
    if ( iscurve1 )
    {
        result = AXIS_SYMMETRY;
    }
    else
    {
        result = REF_SYMMETRY;
    }

    return result;
}

void vectorId2SetId(QVector<QString>& vec, QSet<QString>& set)
{
	for ( QVector<QString>::iterator it = vec.begin(); it!=vec.end(); ++it)
	{
		set.insert(*it);
	}
}
void setId2VectorId(QSet<QString>& set, QVector<QString>& vec)
{
	for ( QSet<QString>::iterator it = set.begin(); it!=set.end(); ++it)
	{
		vec.push_back(*it);
	}
}
