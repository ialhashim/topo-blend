#include "RelationManager.h"
#include "../topo-blend/topo-blend.h"
#include "Scheduler.h"

RelationManager::RelationManager( GraphCorresponder * graph_corresponder, 
	Scheduler * scheduler, QVector<Structure::Graph*> input_graphs )
	: gcorr(graph_corresponder), scheduler(scheduler), inputGraphs(input_graphs)
{
	init();
}

void RelationManager::init()
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

void RelationManager::clear()
{
	init();

	prGroups.clear();
	grGroups.clear();
	diagonals.clear();
}

void RelationManager::parseGlobalReflectionSymm()
{
    //////////////////
    emit( message("Parse global symmetry starts: ") );
    double symmScore[2];
    for (int i = 0; i < this->inputGraphs.size(); ++i)
    {
        GlobalReflectionSymmDetector gsd(this->inputGraphs[i], i);
        symmScore[i] = gsd.detecting();
    }

    this->maxGlobalSymmScore = std::max(symmScore[0], symmScore[1]);
    emit( message("Parse global symmetry end. ") );
}

void RelationManager::parseModelConstraintGroup( bool isSaveFile )
{
    this->grGroups.clear();
    this->diagonals.clear();
    emit( message("Parse group relation starts: ") );
    double groupScore[2];
    double pairScore[2];
    for (int ii = 0; ii < this->inputGraphs.size(); ++ii)
    {
        GroupRelationDetector grd(this->inputGraphs[ii], ii);
        grd.detecting(prGroups[ii],groupScore[ii], pairScore[ii]);

        //qSort(grd.groupRelations_.begin(), grd.groupRelations_.end(), typeLessThan);

		if( isSaveFile )
		{
			saveToFile("group_relation-" + QString::number(ii) + ".txt", grd.groupRelations_);
			saveToFile("pair_relation-" + QString::number(ii) + ".txt", this->prGroups[ii]);
		}

        this->grGroups.push_back(grd.groupRelations_);
        this->diagonals.push_back( this->inputGraphs[ii]->bbox().diagonal().norm());
    }

    this->maxPairScore = std::max(pairScore[0], pairScore[1]);
    this->maxGroupScore = std::max(groupScore[0], groupScore[1]);

    emit( message("Parse group relation end. ") );
}

void RelationManager::parseModelConstraintPair( bool isSaveFile )
{
    this->prGroups.clear();
    emit( message("Parse pair relation starts: ") );
    if ( this->inputGraphs.size() < 2)
    {
        emit( message("Two graphs needed!") );
        return;
    }

    ///////////
    for (int ii = 0; ii < this->inputGraphs.size(); ++ii)
    {
        PairRelationDetector prd(this->inputGraphs[ii], ii);
        prd.detecting();

        //qSort(prd.pairRelations_.begin(), prd.pairRelations_.end(), typeLessThan);

		if( isSaveFile )
		{
			saveToFile("pair_relation-" + QString::number(ii) + ".txt", prd.pairRelations_);
		}

        this->prGroups.push_back(prd.pairRelations_);
    }
    emit( message("Parse pair relation end. ") );
}

void RelationManager::traceModelConstraintsAuto()
{
    if(!scheduler->allGraphs.size()) return;

    if ( !gcorr->isReady)
    {
        emit( message( "No correspondence, load a job first!\n") );
        return;
    }
    if ( this->grGroups.size() != 2 || this->prGroups.size()!=2)
    {
        emit( message( "compute pair & group constraints first!\n") );
        return;
    }

    ///////////////
    std::vector<double> constraintScore;
    std::vector<double> globalSymmScore;

    for ( int j = 0; j < scheduler->allGraphs.size(); ++j)
    {
        double tmpScore[2] = {0,0};
        for ( int i = 0; i < this->grGroups.size(); ++i)
        {
            double gs(0.0), ps(0.0);
            if ( isTraceGroups)
            {
                GroupRelationTracer grt(scheduler->allGraphs[j], this->diagonals[i], i);
                gs = grt.detecting(this->grGroups[i], gcorr->correspondences,i);
            }
            if ( isTracePairs)
            {
                PairRelationTracer prt(scheduler->allGraphs[j], this->diagonals[i], i);
                ps = prt.detecting(this->prGroups[i], gcorr->correspondences,i);

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
            GlobalReflectionSymmDetector gsd(scheduler->allGraphs[j], j);
            globalSymmScore.push_back( gsd.detecting());
        }
    }

    /////////// output
    QString filename("trace_constraints_auto.txt");
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream out(&file);

	// Find maximum constraint score of entire path
    double maxConstraintScore = 0;
    for ( int j = 0; j < scheduler->allGraphs.size(); ++j)
    {
        if ( maxConstraintScore < constraintScore[j])
            maxConstraintScore = constraintScore[j];
    }

	// Output info for each step in path
    for ( int j = 0; j < scheduler->allGraphs.size(); ++j)
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
    QString filename("trace_constraints.txt");
    QFile file(filename);
    if (!file.open(QIODevice::Append | QIODevice::Text)) return;

    QTextStream out(&file);
    int ct = scheduler->slider->currentTime();
    double tmp = (double(ct) / scheduler->totalExecutionTime());
    out << "trace constraints: " << tmp << "\n";
    out << "graph & pair relation weight: " << this->graphWeight << ", " << this->pairWeight << "\n";

    for ( int j = 0; j < this->inputGraphs.size(); ++j)
    {
        double score[2] = {0,0};
        for ( int i = 0; i < this->grGroups.size(); ++i)
        {
            double gs(0.0), ps(0.0);
            if ( isTraceGroups)
            {
                GroupRelationTracer grt(this->inputGraphs[j], this->diagonals[i], i);
                gs = grt.detecting(this->grGroups[i], gcorr->correspondences,i);
                out << "group score of shape " << j << " relative to " << i << " is " << gs << "\n";
            }
            if ( isTracePairs)
            {
                PairRelationTracer prt(this->inputGraphs[j], this->diagonals[i], i);
                ps = prt.detecting(this->prGroups[i], gcorr->correspondences,i);
                out << "pair score of shape " << j << " relative to " << i << " is " << ps << "\n";
            }
            score[i] = computeScore(gs, ps);
        }

        ///////////
        double totalScore = (score[0] + score[1]);// /computeScore(this->maxGroupScore, this->maxPairScore);
        //double totalScore = (score[1]);
        if ( isCheckGlobalSymm)
        {
            GlobalReflectionSymmDetector gsd(this->inputGraphs[j], j, true);
            double symScore = gsd.detecting();
            out << "sym score is " << symScore << "\n";
            out << "constraint score is " << totalScore << "\n";
            totalScore = globalSymmWeight*symScore + (1-globalSymmWeight)*totalScore;
        }
        out << "total score is " << totalScore << "\n\n";
    }

    file.close();
}

double RelationManager::traceModelConstraints( QVector<Structure::Graph*> graphs )
{
	std::vector<double> constraintScore;
	std::vector<double> globalSymmScore;

	for (int j = 0; j < graphs.size(); ++j){
		double tmpScore[2] = {0,0};
		for (int i = 0; i < this->grGroups.size(); ++i){
			double gs(0.0), ps(0.0);
			if ( isTraceGroups )
			{
				GroupRelationTracer grt(graphs[j], this->diagonals[i], i);
				gs = grt.detecting(this->grGroups[i], gcorr->correspondences,i);
			}
			if ( isTracePairs )
			{
				PairRelationTracer prt(graphs[j], this->diagonals[i], i);
				ps = prt.detecting(this->prGroups[i], gcorr->correspondences,i);
			}
			tmpScore[i] = computeScore(gs, ps);
		}

		constraintScore.push_back( (tmpScore[0] + tmpScore[1]) );

		if( isCheckGlobalSymm )
		{
			GlobalReflectionSymmDetector gsd(graphs[j], j);
			globalSymmScore.push_back( gsd.detecting());
		}
	}

	// Find maximum constraint score of entire path
	double maxConstraintScore = 0;
	for ( int j = 0; j < graphs.size(); ++j){
		if ( maxConstraintScore < constraintScore[j])
			maxConstraintScore = constraintScore[j];
	}

	// Output info for each step in path
	double allScore = 0;

	for ( int j = 0; j < graphs.size(); ++j){
		double totalScore = constraintScore[j];
		if ( isCheckGlobalSymm )
		{
			totalScore = globalSymmWeight*globalSymmScore[j] + (1-globalSymmWeight)*totalScore;
		}

		allScore += totalScore;
	}

	return allScore;
}

double RelationManager::computeScore(double groupScore, double pairScore)
{
	return groupScore*this->graphWeight + pairScore*this->pairWeight;
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
