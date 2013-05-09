#include <QFileDialog>

#include "Task.h"
#include "SchedulerWidget.h"
#include "ui_SchedulerWidget.h"

SchedulerWidget::SchedulerWidget(Scheduler * scheduler, QWidget *parent) : QWidget(parent), ui(new Ui::SchedulerWidget), s(scheduler)
{
    ui->setupUi(this);
    ui->timelineView->setScene(scheduler);
	ui->timelineView->updateSceneRect(scheduler->sceneRect());

	// Add nodes to list
	foreach(Task * t, scheduler->tasks)
	{
		ui->nodesList->addItem(t->property["nodeID"].toString());
	}

	scheduler->connect( ui->blendButton, SIGNAL(clicked()), SLOT(doBlend()) );

	connect( scheduler, SIGNAL(progressChanged(int)), ui->progressBar, SLOT(setValue(int)) );

	this->connect( scheduler, SIGNAL(progressStarted()), SLOT(progressStarted()) );
	this->connect( scheduler, SIGNAL(progressDone()), SLOT(progressDone()) );

	scheduler->connect( ui->stopButton, SIGNAL(clicked()), SLOT(stopExecution()) );
	scheduler->connect( ui->sameTimeButton, SIGNAL(clicked()), SLOT(startAllSameTime()) );
	scheduler->connect( ui->diffTimeButton, SIGNAL(clicked()), SLOT(startDiffTime()) );

	scheduler->connect( ui->renderCurrentButton, SIGNAL(clicked()), SLOT(doRenderCurrent()) );
	scheduler->connect( ui->renderAllButton, SIGNAL(clicked()), SLOT(doRenderAll()) );

	scheduler->connect( ui->draftRenderButton, SIGNAL(clicked()), SLOT(doDraftRender()));

	// Render options
	this->connect( ui->reconLevel, SIGNAL(valueChanged(int)), SLOT(changeReconLevel(int)));
	scheduler->property["reconLevel"] = ui->reconLevel->value();

	this->connect( ui->renderCount, SIGNAL(valueChanged(int)), SLOT(changeRenderCount(int)));
	scheduler->property["renderCount"] = ui->renderCount->value();

	// Loading / saving
	connect( ui->loadButton, SIGNAL(clicked()), SLOT(loadSchedule()) );
	connect( ui->saveButton, SIGNAL(clicked()), SLOT(saveSchedule()) );

	ui->progressBar->setVisible(false);
}

SchedulerWidget::~SchedulerWidget()
{
    delete ui;
}

void SchedulerWidget::progressStarted()
{
	ui->progressBar->setVisible(true);
}

void SchedulerWidget::progressDone()
{
	ui->progressBar->setVisible(false);
}

void SchedulerWidget::changeReconLevel(int value)
{
	s->property["reconLevel"] = value;
}

void SchedulerWidget::changeRenderCount(int value)
{
	s->property["renderCount"] = value;
}

void SchedulerWidget::loadSchedule()
{
	QString filename = QFileDialog::getOpenFileName(0, tr("Load Schedule"), "schedule", tr("Schedule Files (*.txt)"));

	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
	QFileInfo fileInfo(file.fileName());

	QTextStream in(&file);

	int N = 0; 
	QString nodeID;
	int start = 0, length = 0;

	in >> N; if(N != s->tasks.size()) { qDebug() << "Invalid schedule!"; return; }

	for(int i = 0; i < N; i++)
	{
		in >> nodeID >> start >> length;
		Task * t = s->getTaskFromNodeID( nodeID );

		if( t )
		{
			t->setStart( start );
			t->setLength( length );
		}
	}

	file.close();
}

void SchedulerWidget::saveSchedule()
{
	QString filename = QFileDialog::getSaveFileName(0, tr("Save Schedule"), "schedule", tr("Schedule Files (*.txt)"));

	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QFileInfo fileInfo(file.fileName());

	QTextStream out(&file);
	out << s->tasks.size() << "\n";
	foreach(Task * t, s->tasks)	out << t->nodeID << " " << t->start << " " << t->length << "\n";
	file.close();
}
