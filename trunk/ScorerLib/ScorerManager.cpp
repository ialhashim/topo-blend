#include "ScorerManager.h"
#include "../topo-blend/topo-blend.h"
#include "Scheduler.h"
#include "GlobalReflectionSymmScorer.h"

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
        ConnectedPairDetector cpd(g);
		cpd.detect();
		connectPairs_.push_back(cpd.pairs_);
        
		if( logLevel_ >1 )
		{
			QFile file("connected_pair_relation-" + QString::number(i) + ".txt");
			if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;		
			QTextStream out(&file);

			for ( QVector<PairRelationBasic>::iterator it = cpd.pairs_.begin(); it != cpd.pairs_.end(); ++it)
				out << *it << "\n";

			file.close();
		}
    }

    emit( message("Parse constraint pairs end. ") );
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

	int ct = this->scheduler->slider->currentTime();
	int idx = this->scheduler->allGraphs.size() * (double(ct) / this->scheduler->totalExecutionTime());
	
	Structure::Graph * g = Structure::Graph::actualGraph(this->scheduler->allGraphs[idx]);
	GlobalReflectionSymmScorer gss(g, idx, logLevel_);


	double symmScore;
	if (isUseSourceCenter_)
		symmScore = gss.evaluate( this->refCenter_, this->refNormal_, this->maxGlobalSymmScore_);
	else
		symmScore = gss.evaluate( gss.center_, this->refNormal_, this->maxGlobalSymmScore_);

    emit( message("Evaluate global symmetry end. ") );
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
	
	///////////////// output to file
	QString filename("evaluate_global_symm_auto.txt");
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;		
	QTextStream out(&file);	
	out << "is use source center: " << isUseSourceCenter_ << "\n";
	
	for (int i = 0; i < symmScore.size(); ++i)
    {
		out << symmScore[i] << "\n";
    }
	file.close();

    emit( message("Evaluate global symmetry auto end. ") );
}
QVector<double> ScorerManager::evaluateGlobalReflectionSymm( QVector<Structure::Graph*> const &graphs )
{
	QVector<double> symmScore;
	int logLevel = 0;
	double tmp;
    for (int i = 0; i < graphs.size(); ++i)
    {
		Structure::Graph * g = Structure::Graph::actualGraph(graphs[i] );
        GlobalReflectionSymmScorer gss(g, i, logLevel);

		if (isUseSourceCenter_)
			tmp = gss.evaluate( this->refCenter_, this->refNormal_, this->maxGlobalSymmScore_);
		else
			tmp = gss.evaluate( gss.center_, this->refNormal_, this->maxGlobalSymmScore_);
		symmScore.push_back(tmp);
    }
	return symmScore;
}

void ScorerManager::setIsUseSourceCenter(bool bUse)
{
	isUseSourceCenter_ = bUse;
	if (isUseSourceCenter_)
		emit( message("use source center") );
	else
		emit( message("not use source center") );
}
