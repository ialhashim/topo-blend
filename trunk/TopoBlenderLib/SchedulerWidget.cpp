#include "Task.h"
#include "SchedulerWidget.h"
#include "ui_SchedulerWidget.h"

SchedulerWidget::SchedulerWidget(Scheduler * scheduler, QWidget *parent) : QWidget(parent),
    ui(new Ui::SchedulerWidget), s(scheduler)
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
