#pragma once

#include <QObject>

class topoblend;
#include "RelationDetector.h"

class RelationManager : public QObject
{
    Q_OBJECT
public:
    RelationManager(topoblend * topo_blender);
    topoblend * tb;
    QVector< QVector<PairRelation> > prGroups; // each element is a list of pair constraint of a graph in tb->graphs
    QVector< QVector<GroupRelation> > grGroups;
    QVector< double> diagonals;
    bool isTracePairs;
    bool isTraceGroups;
    bool isCheckGlobalSymm;
    double globalSymmWeight;
    double pairWeight;
    double graphWeight;
    double maxPairScore;
    double maxGroupScore;
    double maxGlobalSymmScore;

signals:
    
public slots:
    void parseGlobalReflectionSymm();//parseGlobalSymm
    void parseModelConstraintGroup();
    void parseModelConstraintPair();
    void traceModelConstraintsAuto();
    void traceModelConstraints();
    void setIsCheckGlobalSymm(bool);
    void setGlobalSymmWeight(const QString&);
    void setIsTracePairs(bool);
    void setPairWeight(const QString&);
    void setIsTraceGroups(bool);
    void setGraphWeight(const QString&);
    double computeScore(double groupScore, double pairScore);
};
