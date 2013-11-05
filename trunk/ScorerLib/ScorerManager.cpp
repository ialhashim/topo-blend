#include "ScorerManager.h"
#include "../topo-blend/topo-blend.h"
#include "Scheduler.h"
#include "GlobalReflectionSymmScorer.h"

ScorerManager::ScorerManager( GraphCorresponder * graph_corresponder, 
	Scheduler * scheduler, QVector<Structure::Graph*> input_graphs )
	: gcorr(graph_corresponder), scheduler(scheduler), inputGraphs(input_graphs)
{
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
	maxGlobalSymmScore = -1;
	isUseSourceCenter_ = false;
}

void ScorerManager::clear()
{
	init();

	//prGroups.clear();
	//grGroups.clear();
	//diagonals.clear();
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
	int logLevel = 2;
    for (int i = 0; i < this->inputGraphs.size(); ++i)
    {
        GlobalReflectionSymmScorer gss(this->inputGraphs[i], i, logLevel);
        symmScore[i] = gss.evaluate();
		if ( i == 0)
		{
			refCenter_ = gss.center_;
			refNormal_ = gss.normal_;
		}
    }

    //this->maxGlobalSymmScore = std::max(symmScore[0], symmScore[1]);
	this->maxGlobalSymmScore = symmScore[0];
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
	
	int logLevel = 2;
	GlobalReflectionSymmScorer gss(this->scheduler->allGraphs[idx], idx, logLevel);


	double symmScore;
	if (isUseSourceCenter_)
		symmScore = gss.evaluate( this->refCenter_, this->refNormal_, this->maxGlobalSymmScore);
	else
		symmScore = gss.evaluate( gss.center_, this->refNormal_, this->maxGlobalSymmScore);

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
	if (!file.open(QIODevice::Append | QIODevice::Text)) return;		
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
        GlobalReflectionSymmScorer gss(graphs[i], i, logLevel);

		if (isUseSourceCenter_)
			tmp = gss.evaluate( this->refCenter_, this->refNormal_, this->maxGlobalSymmScore);
		else
			tmp = gss.evaluate( gss.center_, this->refNormal_, this->maxGlobalSymmScore);
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
