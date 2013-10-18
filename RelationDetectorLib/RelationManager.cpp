#include "RelationManager.h"
#include "../topo-blend/topo-blend.h"
#include "Scheduler.h"

RelationManager::RelationManager(topoblend *topo_blender) : tb(topo_blender)
{
    isTracePairs = false;
    isTraceGroups = true;
    isCheckGlobalSymm = true;
    pairWeight = 1;
    graphWeight = 4;
    globalSymmWeight = 0.7;
    maxPairScore = 0;
    maxGroupScore = 0;
    maxGlobalSymmScore = 0;
}

void saveScores(QVector<double>& scoreKeptGr, //QVector<double>& scoreAddGr,
    QVector<double>& scoreKeptPr,//QVector<double>& scoreAddPr,
    QTextStream& out)
{
    out << "    basic group kept score: " << std::accumulate(scoreKeptGr.begin(),scoreKeptGr.end(),0.0) << "\n";
    //out << "    basic group add score: " << std::accumulate(scoreAddGr.begin(),scoreAddGr.end(),0.0) << "\n";
    double scoreFinal(0.0), scoreGroup;
    scoreFinal += 4.0 * std::accumulate(scoreKeptGr.begin(),scoreKeptGr.end(),0.0);
    //scoreFinal += 0.5 * std::accumulate(scoreAddGr.begin(),scoreAddGr.end(),0.0);
    scoreGroup = scoreFinal;
    out << "    weighted group score: " << scoreGroup << "\n";

    scoreFinal += 1.0 * std::accumulate(scoreKeptPr.begin(),scoreKeptPr.end(),0.0);
    //scoreFinal += 0.1 * std::accumulate(scoreAddPr.begin(),scoreAddPr.end(),0.0);
    out << "    basic pair kept score: " << std::accumulate(scoreKeptPr.begin(),scoreKeptPr.end(),0.0) << "\n";
    //out << "    basic pair add score: " << std::accumulate(scoreAddPr.begin(),scoreAddPr.end(),0.0) << "\n";
    out << "    weighted pair score: " << scoreFinal - scoreGroup<< "\n";

    out << "    final score: "  << scoreFinal << "\n";
}
void RelationManager::parseGlobalReflectionSymm()
{
    if ( this->tb->graphs.size() < 2)
    {
        this->tb->setStatusBarMessage("Two graphs needed!");
        return;
    }

    //////////////////
    this->tb->setStatusBarMessage("Parse global symmetry starts: ");
    double symmScore[2];
    for (int i = 0; i < this->tb->graphs.size(); ++i)
    {
        GlobalReflectionSymmDetector gsd(this->tb->graphs[i], i);
        symmScore[i] = gsd.detecting();
    }

    this->maxGlobalSymmScore = std::max(symmScore[0], symmScore[1]);
    this->tb->setStatusBarMessage("Parse global symmetry end. ");
}

void RelationManager::parseModelConstraintGroup()
{
    if ( this->tb->graphs.size() < 2)
    {
        this->tb->setStatusBarMessage("Two graphs needed!");
        return;
    }
    if ( this->prGroups.empty())
    {
        this->tb->setStatusBarMessage("Parse pair relation first!");
        return;
    }

    //////////////////
    this->grGroups.clear();
    this->diagonals.clear();
    this->tb->setStatusBarMessage("Parse group relation starts: ");
    double groupScore[2];
    double pairScore[2];
    for (int ii = 0; ii < this->tb->graphs.size(); ++ii)
    {
        GroupRelationDetector grd(this->tb->graphs[ii], ii);
        grd.detecting(prGroups[ii],groupScore[ii], pairScore[ii]);

        //qSort(grd.groupRelations_.begin(), grd.groupRelations_.end(), typeLessThan);
        saveToFile("group_relation-" + QString::number(ii) + ".txt", grd.groupRelations_);
        saveToFile("pair_relation-" + QString::number(ii) + ".txt", this->prGroups[ii]);
        this->grGroups.push_back(grd.groupRelations_);
        this->diagonals.push_back( this->tb->graphs[ii]->bbox().diagonal().norm());
    }

    this->maxPairScore = std::max(pairScore[0], pairScore[1]);
    this->maxGroupScore = std::max(groupScore[0], groupScore[1]);

    this->tb->setStatusBarMessage("Parse group relation end. ");
}
double RelationManager::computeScore(double groupScore, double pairScore)
{
    return groupScore*this->graphWeight + pairScore*this->pairWeight;
}
void RelationManager::parseModelConstraintPair()
{
    this->prGroups.clear();
    this->tb->setStatusBarMessage("Parse pair relation starts: ");
    if ( this->tb->graphs.size() < 2)
    {
        this->tb->setStatusBarMessage("Two graphs needed!");
        return;
    }

    ///////////
    for (int ii = 0; ii < this->tb->graphs.size(); ++ii)
    {

        PairRelationDetector prd(this->tb->graphs[ii], ii);
        prd.detecting();

        //qSort(prd.pairRelations_.begin(), prd.pairRelations_.end(), typeLessThan);
        saveToFile("pair_relation-" + QString::number(ii) + ".txt", prd.pairRelations_);
        this->prGroups.push_back(prd.pairRelations_);
    }
    this->tb->setStatusBarMessage("Parse pair relation end. ");
}
void RelationManager::traceModelConstraintsAuto()
{
    if(!this->tb->scheduler->allGraphs.size()) return;

    if ( !tb->gcoor->isReady)
    {
        tb->mainWindow()->setStatusBarMessage( "No correspondence, load a job first!\n");
        return;
    }
    if ( this->grGroups.size() != 2 || this->prGroups.size()!=2)
    {
        tb->mainWindow()->setStatusBarMessage( "compute pair & group constraints first!\n");
        return;
    }

    ///////////////
    std::vector<double> constraintScore;
    std::vector<double> globalSymmScore;

    for ( int j = 0; j < this->tb->scheduler->allGraphs.size(); ++j)
    {
        double tmpScore[2] = {0,0};
        for ( int i = 0; i < this->grGroups.size(); ++i)
        {
            double gs(0.0), ps(0.0);
            if ( isTraceGroups)
            {
                GroupRelationTracer grt(this->tb->scheduler->allGraphs[j], this->diagonals[i], i);
                gs = grt.detecting(this->grGroups[i], tb->gcoor->correspondences,i);
            }
            if ( isTracePairs)
            {
                PairRelationTracer prt(this->tb->scheduler->allGraphs[j], this->diagonals[i], i);
                ps = prt.detecting(this->prGroups[i], tb->gcoor->correspondences,i);

            }
            tmpScore[i] = computeScore(gs, ps);
        }

        /////////////
        constraintScore.push_back( (tmpScore[0] + tmpScore[1]) );
        //constraintScore.push_back( (tmpScore[1]) );
        //if ( isTraceGroups && isTracePairs)
        //{
        //	constraintScore.push_back( (tmpScore[0] + tmpScore[1])/computeScore(this->maxGroupScore, this->maxPairScore) );
        //}
        //else if ( isTraceGroups)
        //{
        //	constraintScore.push_back( (tmpScore[0] + tmpScore[1])/computeScore(this->maxGroupScore, 0) );
        //}
        //else if (isTracePairs)
        //{
        //	constraintScore.push_back( (tmpScore[0] + tmpScore[1])/computeScore(0, this->maxPairScore) );
        //}

        if ( isCheckGlobalSymm)
        {
            GlobalReflectionSymmDetector gsd(this->tb->scheduler->allGraphs[j], j);
            globalSymmScore.push_back( gsd.detecting());
        }
    }

    /////////// output
    QString filename("trace_constraints_auto.txt");
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream out(&file);
    double maxConstraintScore = 0;
    for ( int j = 0; j < this->tb->scheduler->allGraphs.size(); ++j)
    {
        if ( maxConstraintScore < constraintScore[j])
            maxConstraintScore = constraintScore[j];
    }
    for ( int j = 0; j < this->tb->scheduler->allGraphs.size(); ++j)
    {
        out << "trace constraints: " << j << "\n";
        //double totalScore = constraintScore[j]/maxConstraintScore;
        double totalScore = constraintScore[j];

        if ( isCheckGlobalSymm)
        {
            out << "sym score is " << globalSymmScore[j] << "\n";
            out << "constraint score is " << totalScore << "\n";
            totalScore = globalSymmWeight*globalSymmScore[j] + (1-globalSymmWeight)*totalScore;
        }
        out << "total score is " << totalScore << "\n\n";
    }
    file.close();
}
void RelationManager::traceModelConstraints()
{
    if ( !tb->gcoor->isReady)
    {
        tb->mainWindow()->setStatusBarMessage( "No correspondence, load a job first!\n");
        return;
    }
    if ( this->grGroups.size() != 2 || this->prGroups.size()!=2)
    {
        tb->mainWindow()->setStatusBarMessage( "compute pair & group constraints first!\n");
        return;
    }

    ////////////////////
    QString filename("trace_constraints.txt");
    QFile file(filename);
    if (!file.open(QIODevice::Append | QIODevice::Text)) return;

    QTextStream out(&file);
    int ct = this->tb->scheduler->slider->currentTime();
    double tmp = (double(ct) / this->tb->scheduler->totalExecutionTime());
    out << "trace constraints: " << tmp << "\n";
    out << "graph & pair relation weight: " << this->graphWeight << ", " << this->pairWeight << "\n";

    for ( int j = 0; j < this->tb->graphs.size(); ++j)
    {
        double score[2] = {0,0};
        for ( int i = 0; i < this->grGroups.size(); ++i)
        {
            double gs(0.0), ps(0.0);
            if ( isTraceGroups)
            {
                GroupRelationTracer grt(this->tb->graphs[j], this->diagonals[i], i);
                gs = grt.detecting(this->grGroups[i], tb->gcoor->correspondences,i);
                out << "group score of shape " << j << " relative to " << i << " is " << gs << "\n";
            }
            if ( isTracePairs)
            {
                PairRelationTracer prt(this->tb->graphs[j], this->diagonals[i], i);
                ps = prt.detecting(this->prGroups[i], tb->gcoor->correspondences,i);
                out << "pair score of shape " << j << " relative to " << i << " is " << ps << "\n";
            }
            score[i] = computeScore(gs, ps);
        }

        ///////////
        double totalScore = (score[0] + score[1]);// /computeScore(this->maxGroupScore, this->maxPairScore);
        //double totalScore = (score[1]);
        if ( isCheckGlobalSymm)
        {
            GlobalReflectionSymmDetector gsd(this->tb->graphs[j], j, true);
            double symScore = gsd.detecting();
            out << "sym score is " << symScore << "\n";
            out << "constraint score is " << totalScore << "\n";
            totalScore = globalSymmWeight*symScore + (1-globalSymmWeight)*totalScore;
        }
        out << "total score is " << totalScore << "\n\n";
    }

    file.close();
}
void RelationManager::setIsCheckGlobalSymm(bool checked)
{
    this->isCheckGlobalSymm = checked;
}
void RelationManager::setGlobalSymmWeight(const QString& text)
{
    bool ok;
    double tmp = text.toDouble(&ok);
    if (ok)
        this->globalSymmWeight = tmp;
}
void RelationManager::setIsTracePairs(bool checked)
{
    this->isTracePairs = checked;
}
void RelationManager::setIsTraceGroups(bool checked)
{
    this->isTraceGroups = checked;
}
void RelationManager::setPairWeight(const QString& text)
{
    bool ok;
    double tmp = text.toDouble(&ok);
    if (ok)
        this->pairWeight = tmp;
}
void RelationManager::setGraphWeight(const QString& text)
{
    bool ok;
    double tmp = text.toDouble(&ok);
    if (ok)
        this->graphWeight = tmp;
}
