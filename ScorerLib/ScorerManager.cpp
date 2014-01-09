#include "ScorerManager.h"
#include "../topo-blend/topo-blend.h"
#include "Scheduler.h"

#include "GlobalReflectionSymmScorer.h"

#include "PairRelationDetector.h"
#include "ConnectivityScorer.h"
//#include "PairRelationScorer.h"
#include "GroupRelationScorer.h"

QTextStream& operator << (QTextStream& os, const ScorerManager::PathScore& pr)
{    
	os << "Min connectivity " << pr.connectivity.minCoeff() << "\n";
	os << "Min localSymmetry " << pr.localSymmetry.minCoeff() << "\n";
	os << "Min globalSymmetry " << pr.globalSymmetry.minCoeff() << "\n";
	//int N = pr.connectivity.size();
	//os << "Connectivity \n";
	//for(int i = 0; i < N; ++i)
	//{
	//	os << pr.connectivity[i] << "\n";
	//}
	//os << "local symm \n";
	//for(int i = 0; i < N; ++i)
	//{
	//	os << pr.localSymmetry[i] << "\n";
	//}
	//os << "global symm \n";
	//for(int i = 0; i < N; ++i)
	//{
	//	os << pr.globalSymmetry[i] << "\n";
	//}
    return os;
}

void saveScore(QString filename, QVector<double> scores, QString headline)
{
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;		
	QTextStream out(&file);	
	if ( !headline.isEmpty() )
		out << headline;
	
	int size = scores.size();
	out << size << "\n";
	for (int i = 0; i < size; ++i)
    {
		out << scores[i] << "\n";
    }
	file.close();
}

ScorerManager::ScorerManager( GraphCorresponder * graph_corresponder, 
	Scheduler * scheduler, QVector<Structure::Graph*> input_graphs, int logLevel )
{
	this->logLevel_ = logLevel;
	this->isUseSourceCenter_ = false;
	this->isUseLink_ = false;
	this->bUsePart_ = true;
	init( graph_corresponder, scheduler, input_graphs);
}

void ScorerManager::init(GraphCorresponder * graph_corresponder, Scheduler * scheduler, QVector<Structure::Graph*> input_graphs)
{
	this->gcorr_ = graph_corresponder;
	this->scheduler_ = scheduler;
	this->normalizeCoef_ = 0.0;
	for ( int i = 0; i < input_graphs.size(); ++i)
	{
		this->actualInputGraphs_.push_back(Structure::Graph::actualGraph( input_graphs[i] ));
		normalizeCoef_ += this->actualInputGraphs_[i]->bbox().diagonal().norm();
	}
	normalizeCoef_ *= 0.5;

	maxGlobalSymmScore_ = -1;	
}


void ScorerManager::parseConstraintPair()
{
    emit( message("Parse constraint pair starts: ") );

	this->connectPairs_.clear();	this->otherPairs_.clear();
	for ( int i = 0; i < this->actualInputGraphs_.size(); ++i)
	{
		PairRelationDetector cpd(this->actualInputGraphs_[i], i, normalizeCoef_, this->isUseLink_, true, logLevel_);
		cpd.detect(this->actualInputGraphs_[(i+1)%this->actualInputGraphs_.size()], this->gcorr_->correspondences);
		this->connectPairs_.push_back(cpd.connectedPairs_);
		this->otherPairs_.push_back(cpd.otherPairs_);

		if (this->logLevel_)
		{
			saveToFile("pair_relation-" + QString::number(i) + ".txt", cpd.otherPairs_);
		}
	}

    emit( message("Parse constraint pairs end. ") );
}
void ScorerManager::evaluateTopology()
{
	/////////////////
    emit( message("Evaluate topology starts: ") );

	if ( connectPairs_.empty() )
	{
		emit( message("Parse topology first! ") );
		return;
	}

	int idx(0);
	Structure::Graph* g = getCurrentGraph(idx);
	ConnectivityScorer cs(g, idx, this->normalizeCoef_, this->isUseLink_, this->logLevel_);	
	cs.evaluate(this->connectPairs_, this->gcorr_->correspondences);

    emit( message("Evaluate topology end. ") );
}
QVector<double> ScorerManager::evaluateTopology( QVector<Structure::Graph*> const &graphs )
{
	QVector<double> topoScore;
	int logLevel = 0;
	double score;
    for (int i = 0; i < graphs.size(); ++i)
    {
		Structure::Graph * g = Structure::Graph::actualGraph(graphs[i] );
		ConnectivityScorer cs(g, i, this->normalizeCoef_, this->isUseLink_, logLevel);	
		score = cs.evaluate(this->connectPairs_, this->gcorr_->correspondences);
		topoScore.push_back(score);
    }
	return topoScore;
}
void ScorerManager::evaluateTopologyAuto()
{
    emit( message("Evaluate topology auto starts: ") );

	if ( connectPairs_.empty() )
	{
		emit( message("Parse topology first! ") );
		return;
	}
	QVector<double> topoScore = evaluateTopology(this->scheduler_->allGraphs);
    saveScore(QString("evaluate_connectivity_auto.txt"), topoScore, QString());

    emit( message("Evaluate topology auto end. ") );
}

void ScorerManager::parseConstraintGroup()
{
	//////////////////
    emit( message("Parse constraint group starts: ") );
	if ( this->otherPairs_.size() < 2)
	{
		emit( message("Parse pairs first!") );
		return;
	}

	///////////////////////
	groupRelations_.clear();
	for ( int i = 0; i < this->actualInputGraphs_.size(); ++i)
	{
		Structure::Graph * g = Structure::Graph::actualGraph( this->actualInputGraphs_[i] );
		GroupRelationDetector grd(g, i, this->normalizeCoef_, this->logLevel_);
		grd.detect(this->otherPairs_[i]);
		this->groupRelations_.push_back(grd.groupRelations_);
		if (this->logLevel_)
		{
			saveToFile("group_relation-" + QString::number(i) + ".txt", this->groupRelations_[i]);
			saveToFile("pair_relation-" + QString::number(i) + ".txt", this->otherPairs_[i]);
		}
	}	

    emit( message("Parse constraint group end. ") );
}
void ScorerManager::evaluateGroups()
{
	/////////////////
    emit( message("Evaluate group starts: ") );

	if ( this->groupRelations_.empty() )
	{
		emit( message("Parse constraint group first! ") );
		return;
	}

	int idx(0);
	Structure::Graph* g = getCurrentGraph(idx);
	GroupRelationScorer grs(g, idx, this->normalizeCoef_, this->logLevel_);
	grs.evaluate(groupRelations_, this->gcorr_->correspondences);

    emit( message("Evaluate group end. ") );
}
QVector<double> ScorerManager::evaluateGroups( QVector<Structure::Graph*> const &graphs )
{
	QVector<double> groupScore;
	int logLevel = 0;
	double score;
    for (int i = 0; i < graphs.size(); ++i)
    {
		Structure::Graph * g = Structure::Graph::actualGraph(graphs[i] );
		GroupRelationScorer grs(g, i, this->normalizeCoef_, logLevel);
		score = grs.evaluate(groupRelations_, this->gcorr_->correspondences);
		groupScore.push_back(score);
    }
	return groupScore;
}
void ScorerManager::evaluateGroupsAuto()
{
    emit( message("Evaluate group auto starts: ") );

	if ( groupRelations_.empty() )
	{
		emit( message("Parse constraint group first! ") );
		return;
	}


	QVector<double> groupScore = evaluateGroups(this->scheduler_->allGraphs);
	saveScore(QString("evaluate_group_auto.txt"), groupScore, QString());	

    emit( message("Evaluate group auto end. ") );
	return;
}

void ScorerManager::parseGlobalReflectionSymm()
{
	//////////////////
    emit( message("Parse global symmetry starts: ") );
	if ( this->actualInputGraphs_.size() < 2)
	{
		emit( ("Two graphs needed!") );
		return;
	}

    double symmScore[2];
    for (int i = 0; i < this->actualInputGraphs_.size(); ++i)
    {
		Structure::Graph * g = Structure::Graph::actualGraph( this->actualInputGraphs_[i] );
		GlobalReflectionSymmScorer gss(g, i, this->normalizeCoef_, this->bUsePart_, this->logLevel_);
        symmScore[i] = gss.evaluate();
		if ( i == 0)
		{
			this->refCenter_ = gss.center_;
			this->refNormal_ = gss.normal_;
		}
    }

    this->maxGlobalSymmScore_ = std::max(symmScore[0], symmScore[1]);
	//this->maxGlobalSymmScore_ = symmScore[0];
    emit( message("Parse global symmetry end. ") );
}
void ScorerManager::evaluateGlobalReflectionSymm()
{
	/////////////////
    emit( message("Evaluate global symmetry starts: ") );

	if ( !isGlobalReflectionSymmParsed() )
	{
		emit( message("Parse global symmetry first! ") );
		return;
	}

	int idx(0);
	Structure::Graph* g = getCurrentGraph(idx);
	GlobalReflectionSymmScorer gss(g, idx, this->normalizeCoef_, this->bUsePart_, this->logLevel_);

	double symmScore;
	if (isUseSourceCenter_)
		symmScore = gss.evaluate( this->refCenter_, this->refNormal_, this->maxGlobalSymmScore_);
	else
		symmScore = gss.evaluate( gss.center_, this->refNormal_, this->maxGlobalSymmScore_);

    emit( message("Evaluate global symmetry end. ") );
}
QVector<double> ScorerManager::evaluateGlobalReflectionSymm( QVector<Structure::Graph*> const &graphs )
{
	QVector<double> symmScore;
	int logLevel = 0;
	double score;
    for (int i = 0; i < graphs.size(); ++i)
    {
		Structure::Graph * g = Structure::Graph::actualGraph(graphs[i] );
        GlobalReflectionSymmScorer gss(g, i, this->normalizeCoef_, this->bUsePart_, logLevel);

		if (isUseSourceCenter_)
			score = gss.evaluate( this->refCenter_, this->refNormal_, this->maxGlobalSymmScore_);
		else
			score = gss.evaluate( gss.center_, this->refNormal_, this->maxGlobalSymmScore_);

		symmScore.push_back(score);
    }
	return symmScore;
}
void ScorerManager::evaluateGlobalReflectionSymmAuto()
{
	///////////////// pre-condition
    emit( message("Evaluate global symmetry auto starts: ") );

	if ( !isGlobalReflectionSymmParsed() )
	{
		emit( message("Parse global symmetry first! ") );
		return;
	}

	///////////////// compute
	QVector<double> symmScore = evaluateGlobalReflectionSymm(this->scheduler_->allGraphs);    

	QString headline = "is use source center: ";
	if ( isUseSourceCenter_)
	{
		headline.append("true");
	}
	else
	{
		headline.append("false");
	}
	headline.append("\n");
	saveScore(QString("evaluate_global_symm_auto.txt"), symmScore, headline);		

    emit( message("Evaluate global symmetry auto end. ") );
}


void ScorerManager::setIsUseSourceCenter(bool bUse)
{
	isUseSourceCenter_ = bUse;
	if (isUseSourceCenter_)
		emit( message("use source center") );
	else
		emit( message("not use source center") );
}
void ScorerManager::setIsUsePart(bool bUse)
{
	this->bUsePart_ = bUse;
	if (bUsePart_)
		emit( message("use part for global symm") );
	else
		emit( message("not use part for global symm") );
}

void ScorerManager::setIsUseLink(bool bUse)
{
	isUseLink_ = bUse;
	if (isUseLink_)
		emit( message("use link for connectivity") );
	else
		emit( message("not link for connectivity") );
}
Structure::Graph * ScorerManager::getCurrentGraph(int& idx)
{
	int ct = this->scheduler_->slider->currentTime();
	idx = this->scheduler_->allGraphs.size() * (double(ct) / this->scheduler_->totalExecutionTime());
	
	Structure::Graph * g = Structure::Graph::actualGraph(this->scheduler_->allGraphs[idx]);
	return g;
}

ScorerManager::PathScore ScorerManager::pathScore( Scheduler * scheduler )
{
	ScorerManager::PathScore score;

	QVector<Structure::Graph*> graphs = scheduler->allGraphs;
	int N = graphs.size();
	int logLevel = 0;

	QVector<double> connectivity, localSymmetry, globalSymmetry;

	// Compute all scores at once
	for (int i = 0; i < N; ++i)
	{
		Structure::Graph * g = Structure::Graph::actualGraph(graphs[i] );

		// Connectivity
		ConnectivityScorer cs(g, i, this->normalizeCoef_, this->isUseLink_, logLevel);	
		connectivity.push_back( cs.evaluate(this->connectPairs_, this->gcorr_->correspondences) );

		// Local symmetry
		GroupRelationScorer grs(g, i, this->normalizeCoef_, logLevel);
		localSymmetry.push_back( grs.evaluate(groupRelations_, this->gcorr_->correspondences) );

		// Global symmetry
		GlobalReflectionSymmScorer gss(g, i, this->normalizeCoef_, logLevel);
		if (isUseSourceCenter_)
			globalSymmetry.push_back( gss.evaluate( this->refCenter_, this->refNormal_, this->maxGlobalSymmScore_) );
		else
			globalSymmetry.push_back( gss.evaluate( gss.center_, this->refNormal_, this->maxGlobalSymmScore_) );

		// Clean up
		delete g;
	}

	// Fill scores into vectors
	score.connectivity = VectorXd::Zero(N);
	score.localSymmetry = VectorXd::Zero(N);
	score.globalSymmetry = VectorXd::Zero(N);

	for(int i = 0; i < N; i++)
	{
		score.connectivity[i] = connectivity[i];
		score.localSymmetry[i] = localSymmetry[i];
		score.globalSymmetry[i] = globalSymmetry[i];

		scheduler->allGraphs[i]->property["scoreConnectivity"] = connectivity[i];
		scheduler->allGraphs[i]->property["scoreSymLocal"] = localSymmetry[i];
		scheduler->allGraphs[i]->property["scoreSymGlobal"] = globalSymmetry[i];
	}

	return score;
}

MatrixXd ScorerManager::PathScore::computeRange()
{
	// Columns: min, max, range, average
	MatrixXd R(3, 4);

	R(0,0) = connectivity.minCoeff();	R(0,1) = connectivity.maxCoeff();	R(0,2) = R(0,1) - R(0,0); R(0,3) = connectivity.mean();
	R(1,0) = localSymmetry.minCoeff();	R(1,1) = localSymmetry.maxCoeff();	R(1,2) = R(1,1) - R(1,0); R(1,3) = localSymmetry.mean();
	R(2,0) = globalSymmetry.minCoeff(); R(2,1) = globalSymmetry.maxCoeff(); R(2,2) = R(2,1) - R(2,0); R(2,3) = globalSymmetry.mean();

	return R;
}

double ScorerManager::PathScore::score()
{
	int N = connectivity.size();

	double errConnect = 0, errLocal = 0, errSymmetry = 0;

	for(int i = 0; i < N; i++)
	{
		errConnect += 1.0 - connectivity[i];
		errLocal += 1.0 - localSymmetry[i];
		errSymmetry +=  1.0 - globalSymmetry[i];
	}

	double error = qMax(qMax(errConnect, errLocal), errSymmetry);

	return -error;
}
