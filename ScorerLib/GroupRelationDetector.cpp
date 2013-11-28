#include "GroupRelationDetector.h"

void GroupRelationDetector::detect(QVector<PairRelation>& prRelations)
{
    detectSymmGroupByRefPair( prRelations);
    detectSymmGroupByTransPair( prRelations);
    detectCoplanarGroup( prRelations);
    mergeCoplanarGroup();
    removeRedundantGroup();
}

void GroupRelationDetector::detectSymmGroupByRefPair(QVector<PairRelation>& prRelations)
{
    int nPrs = prRelations.size();
    for (int i=0; i<nPrs; ++i)
        prRelations[i].tag = false;

    double dotVal1(0.0), dotVal2(0.0);
    ///////////
    // build group from REF pair
    for (int i=0; i<nPrs; ++i)
    {
        PairRelation& pr1 = prRelations[i];
        if ( pr1.tag || REF.compare(pr1.type)) continue;
                
        Structure::Node* n1 = pr1.n1;
        Structure::Node* n2 = pr1.n2;
		QSet<QString> ids;
		GroupRelation gr;
		ids.insert( n1->id);			ids.insert(n2->id);
        pr1.tag = true;

        Vector_3 v1, v2;
        node2direction(n1,v1); node2direction(n2,v2);
        dotVal1 = std::abs(dot(v1,v2));
        if ( dotVal1 > thAxisDeviationRadio_)
        {
            if ( dot(v1,v2) < 0)
                gr.direction = v1 -v2;
            else
                gr.direction = v1+v2;
        }
        else
        {
            gr.direction = cross_product(v1, v2);
        }
		gr.direction.normalize();

        ///////////////
        for (int j=i+1; j<nPrs; ++j)
        {
            PairRelation& pr2 = prRelations[j];
            if ( pr2.tag || REF.compare(pr2.type)) continue;

            if ( isIntersected(ids, pr2) )
            {
                n1 = pr2.n1;
                n2 = pr2.n2;
                node2direction(n1,v1); node2direction(n2,v2);
                Vector_3 dir;
                if ( dotVal1 > thAxisDeviationRadio_)
                {
                    dir = v1 + v2;
                }
                else
                {
                    dotVal2 = std::abs( dot(v1,v2) );
                    if ( dotVal2 > thAxisDeviationRadio_)
                        continue;
                    else
                        dir = cross_product(v1, v2);
                }
				dir.normalize();

                double tmp = std::abs( dot(gr.direction, dir) );
                if ( tmp > thAxisDeviationRadio_)
                {
                    if ( dot(gr.direction, dir) < 0)
                        gr.direction = gr.direction - dir;
                    else
                        gr.direction = gr.direction + dir;

					gr.direction.normalize();
                    ids.insert(n1->id);	ids.insert(n2->id);
                    pr2.tag = true;
                }
            }
        }

        ///////////////
        for ( QSet<QString>::iterator it = ids.begin(); it != ids.end(); ++it)
        {
            gr.ids.push_back(*it);
        }
        gr.diameter = computeGroupDiameter(gr);
		computeGroupCenter(gr);

        ////////////////
		gr.note = REF;
        gr.deviation = computeAxisSymmetryGroupDeviation(gr, pointLevel_);
        gr.type = AXIS_SYMMETRY;
        if ( gr.deviation / graph_->bbox().diagonal().norm() < thAxisGroup_)
        {
            groupRelations_.push_back(gr);
        }
        else
        {
            gr.type = REF_SYMMETRY;
            gr.deviation = computeRefSymmetryGroupDeviation(gr,pointLevel_);
            if ( gr.deviation / graph_->bbox().diagonal().norm() < thRefGroup_)
            {
                groupRelations_.push_back(gr);
            }
        }

    }
    ////////////// remove pairs have been put into groups
    prRelations.erase( std::remove_if(prRelations.begin(), prRelations.end(), isTaged), prRelations.end() );
}

void GroupRelationDetector::detectSymmGroupByTransPair(QVector<PairRelation>& prRelations)
{
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
        gr.deviation = computeAxisSymmetryGroupDeviation(gr, pointLevel_);
        if ( gr.deviation / graph_->bbox().diagonal().norm() < thAxisGroup_)
        {
            groupRelations_.push_back(gr);
            continue;
        }
        else
        {
            gr.type = REF_SYMMETRY;
			gr.deviation = computeRefSymmetryGroupDeviation(gr, pointLevel_);
            if ( gr.deviation / graph_->bbox().diagonal().norm() < thRefGroup_)
            {
                groupRelations_.push_back(gr);
            }
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

void GroupRelationDetector::detectCoplanarGroup(QVector<PairRelation>& prRelations)
{
    int nPrs = prRelations.size();
    for (int i=0; i<nPrs; ++i)
    {
        prRelations[i].tag = false;
    }

    ///////////
    for (int i=0; i<nPrs; ++i)
    {
        PairRelation& pr1 = prRelations[i];
        if (pr1.tag || COPLANAR.compare(pr1.type)) continue;

        GroupRelation gr;
        QSet<QString> ids;
        gr.type = COPLANAR;
        ids.insert(pr1.n1->id);
        ids.insert(pr1.n2->id);
        gr.point = pr1.point;
        gr.normal = pr1.normal;
        gr.deviation = pr1.deviation;

        for (int j=i+1; j<nPrs; ++j)
        {
            PairRelation& pr2 = prRelations[j];
            if (pr2.tag || COPLANAR.compare(pr2.type)) continue;

            double error = errorOfCoplanar(pr1.point, pr1.normal, pr2.point, pr2.normal);

            if ( error < thCoplaGroup_)
            {
                ids.insert(pr2.n1->id);
                ids.insert(pr2.n2->id);
                pr2.tag = true;
                gr.deviation += error;
            }
        }

        ///////////////
        gr.deviation = gr.deviation/(ids.size()-1);
        if ( gr.deviation < thCoplaGroup_)
        {
            for ( QSet<QString>::iterator it = ids.begin(); it!=ids.end(); ++it)
            {
                gr.ids.push_back(*it);
            }
            gr.diameter = computeGroupDiameter(gr);
            groupRelations_.push_back(gr);
            pr1.tag = true;
        }
    }

    ////////////// remove pairs have been put into groups
    prRelations.erase( std::remove_if(prRelations.begin(), prRelations.end(), isTaged), prRelations.end() );
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
void GroupRelationDetector::mergeCoplanarGroup()
{
    for ( int i = 0; i < (int) groupRelations_.size(); ++i)
    {
        groupRelations_[i].tag = false;
    }
    for ( int i = 0; i < (int) groupRelations_.size(); ++i)
    {
        if ( COPLANAR != groupRelations_[i].type || groupRelations_[i].tag)
            continue;

        Vector3 pt1 = groupRelations_[i].point;
        Vector3 normal1 = groupRelations_[i].normal;
        QSet<QString> ids;
        vectorId2SetId(groupRelations_[i].ids, ids);

        for ( int j = i+1; j < (int) groupRelations_.size(); ++j)
        {
            if ( COPLANAR != groupRelations_[j].type || groupRelations_[j].tag)
                continue;

            Vector3 pt2 = groupRelations_[j].point;
            Vector3 normal2 = groupRelations_[j].normal;
            double err = errorOfCoplanar(pt1, normal1, pt2, normal2);

            if ( err < thCoplaGroup_)
            {
                groupRelations_[j].tag = true;
                vectorId2SetId(groupRelations_[j].ids, ids);
            }
        }

        groupRelations_[i].ids.clear();
        setId2VectorId(ids, groupRelations_[i].ids);
    }

    groupRelations_.erase( std::remove_if(groupRelations_.begin(), groupRelations_.end(), isTagedGr), groupRelations_.end() );
}