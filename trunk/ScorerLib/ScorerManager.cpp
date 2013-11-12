#include "ScorerManager.h"
#include "../topo-blend/topo-blend.h"
#include "Scheduler.h"
#include "GlobalReflectionSymmScorer.h"
#include "ConnectivityScorer.h"
void saveScore(QString &filename, QVector<double> scores, QString& headline)
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
	Scheduler * scheduler, QVector<Structure::Graph*> input_graphs )
	: gcorr(graph_corresponder), scheduler(scheduler), inputGraphs(input_graphs)
{
	logLevel_ = 2;
	init();
}

void ScorerManager::init()
{
	//isTracePairs = false;
	//isTraceGroups = true;
	//isCheckGlobalSymm = true;

	//pairWeight = 1;
	//graphWeight = 4;
	//globalSymmWeight = 0.7;

	//maxPairScore = 0;
	//maxGroupScore = 0;
	maxGlobalSymmScore_ = -1;
	isUseSourceCenter_ = false;
}

void ScorerManager::clear()
{
	init();

	//prGroups.clear();
	//grGroups.clear();
	//diagonals.clear();
}

void ScorerManager::parseConstraintPair()
{
	//////////////////
    emit( message("Parse constraint pair starts: ") );
	if ( this->inputGraphs.size() < 2)
	{
		emit( ("Two graphs needed!") );
		return;
	}

	///////////////////////
	connectPairs_.clear();
    for (int i = 0; i < this->inputGraphs.size(); ++i)
    {
		Structure::Graph * g = Structure::Graph::actualGraph( this->inputGraphs[i] );
        ConnectedPairDetector cpd(g, i, logLevel_);
		cpd.detect();
		connectPairs_.push_back(cpd.pairs_);        
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
	ConnectivityScorer cs(g, idx, logLevel_);
	double score = cs.evaluate(connectPairs_, gcorr->correspondences);

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
        ConnectivityScorer cs(g, i, logLevel);
		score = cs.evaluate(connectPairs_, gcorr->correspondences);
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

	///////////////// 
	QVector<double> topoScore = evaluateTopology(this->scheduler->allGraphs);
	saveScore(QString("evaluate_connectivity_auto.txt"), topoScore, QString());	


    emit( message("Evaluate topology auto end. ") );
}

void ScorerManager::parseGlobalReflectionSymm()
{
	//////////////////
    emit( message("Parse global symmetry starts: ") );
	if ( this->inputGraphs.size() < 2)
	{
		emit( ("Two graphs needed!") );
		return;
	}

    double symmScore[2];
    for (int i = 0; i < this->inputGraphs.size(); ++i)
    {
		Structure::Graph * g = Structure::Graph::actualGraph( this->inputGraphs[i] );
        GlobalReflectionSymmScorer gss(g, i, logLevel_);
        symmScore[i] = gss.evaluate();
		if ( i == 0)
		{
			refCenter_ = gss.center_;
			refNormal_ = gss.normal_;
		}
    }

    //this->maxGlobalSymmScore_ = std::max(symmScore[0], symmScore[1]);
	this->maxGlobalSymmScore_ = symmScore[0];
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
	GlobalReflectionSymmScorer gss(g, idx, logLevel_);

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
        GlobalReflectionSymmScorer gss(g, i, logLevel);

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
	QVector<double> symmScore = evaluateGlobalReflectionSymm(this->scheduler->allGraphs);    

	QString headline = "is use source center: ";
	headline.append(isUseSourceCenter_);
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
Structure::Graph * ScorerManager::getCurrentGraph(int& idx)
{
	int ct = this->scheduler->slider->currentTime();
	idx = this->scheduler->allGraphs.size() * (double(ct) / this->scheduler->totalExecutionTime());
	
	Structure::Graph * g = Structure::Graph::actualGraph(this->scheduler->allGraphs[idx]);
	return g;
}
