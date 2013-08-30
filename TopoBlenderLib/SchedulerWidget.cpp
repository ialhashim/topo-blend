#include <QFileDialog>

#include "Task.h"
#include "SchedulerWidget.h"
#include "ui_SchedulerWidget.h"

SchedulerWidget::SchedulerWidget(Scheduler * scheduler, QWidget *parent) : QWidget(parent), ui(new Ui::SchedulerWidget), s(scheduler)
{
    ui->setupUi(this);

	// Execute blending sequence
	scheduler->connect( ui->blendButton, SIGNAL(clicked()), SLOT(doBlend()) );

	// Controls show / hide
	isShowControls = true;
	toggleOptions(); // Hide all controls by default
	this->connect(ui->showAllButton, SIGNAL(clicked()), SLOT(toggleOptions()));

    ui->timelineView->setScene(scheduler);
	ui->timelineView->updateSceneRect(scheduler->sceneRect());

	connect( scheduler, SIGNAL(progressChanged(int)), ui->progressBar, SLOT(setValue(int)) );

	this->connect( scheduler, SIGNAL(progressStarted()), SLOT(progressStarted()) );
	this->connect( scheduler, SIGNAL(progressDone()), SLOT(progressDone()) );

	scheduler->connect( ui->stopButton, SIGNAL(clicked()), SLOT(stopExecution()) );
	scheduler->connect( ui->sameTimeButton, SIGNAL(clicked()), SLOT(startAllSameTime()) );
	scheduler->connect( ui->diffTimeButton, SIGNAL(clicked()), SLOT(startDiffTime()) );
	scheduler->connect( ui->defaultScheduleButton, SIGNAL(clicked()), SLOT(defaultSchedule()) );
	scheduler->connect( ui->shuffleButton, SIGNAL(clicked()), SLOT(shuffleSchedule()) );

	scheduler->connect( ui->renderCurrentButton, SIGNAL(clicked()), SLOT(doRenderCurrent()) );
	scheduler->connect( ui->renderAllButton, SIGNAL(clicked()), SLOT(doRenderAll()) );

	scheduler->connect( ui->draftRenderButton, SIGNAL(clicked()), SLOT(doDraftRender()));

	// Discretization
	scheduler->connect( ui->gdResolution, SIGNAL(valueChanged(double)), SLOT(setGDResolution(double)));
	scheduler->connect( ui->timeStep, SIGNAL(valueChanged(double)), SLOT(setTimeStep(double)));

	// Render options
	this->connect( ui->reconLevel, SIGNAL(valueChanged(int)), SLOT(changeReconLevel(int)));
	scheduler->property["reconLevel"] = ui->reconLevel->value();

	this->connect( ui->renderCount, SIGNAL(valueChanged(int)), SLOT(changeRenderCount(int)));
	scheduler->property["renderCount"] = ui->renderCount->value();

	this->connect( ui->startPercentage, SIGNAL(valueChanged(int)), SLOT(changeRenderStartPercentage(int)));
	scheduler->property["renderStartPercentage"] = ui->startPercentage->value();

	// Loading / saving
	connect( ui->loadButton, SIGNAL(clicked()), SLOT(loadSchedule()) );
	connect( ui->saveButton, SIGNAL(clicked()), SLOT(saveSchedule()) );

	ui->progressBar->setVisible(false);

	// List nodes from scheduler
	updateNodesList();
}

void SchedulerWidget::toggleOptions()
{
	if (isShowControls)
	{
		ui->blendControls->hide();
		ui->nodesWrapperWidget->hide();
		ui->synthesisControl->hide();
		ui->timelineLabel->hide();

		isShowControls = false;
	}
	else
	{
		ui->blendControls->show();
		ui->nodesWrapperWidget->show();
		ui->synthesisControl->show();
		ui->timelineLabel->show();

		isShowControls = true;
	}
}

void SchedulerWidget::updateNodesList()
{
	ui->nodesList->clear();

	// Add nodes to list
	foreach(Task * t, s->tasks)
		ui->nodesList->addItem(t->property["nodeID"].toString());
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

void SchedulerWidget::changeRenderStartPercentage(int value)
{
	s->property["renderStartPercentage"] = value;
}

void SchedulerWidget::loadSchedule()
{
	s->loadSchedule(QFileDialog::getOpenFileName(0, tr("Load Schedule"), "schedule", tr("Schedule Files (*.txt)")));
}

void SchedulerWidget::saveSchedule()
{
	s->saveSchedule(QFileDialog::getSaveFileName(0, tr("Save Schedule"), "schedule", tr("Schedule Files (*.txt)")));
}

double SchedulerWidget::gdResolution()
{
	return ui->gdResolution->value();
}

double SchedulerWidget::timeStep()
{
	return ui->timeStep->value();
}

int SchedulerWidget::reconLevel()
{
	return ui->reconLevel->value();
}

int SchedulerWidget::renderCount()
{
	return ui->renderCount->value();
}

void SchedulerWidget::setParams( double gdRes, double tStep, int rLevel, int rCount )
{
	ui->gdResolution->setValue( gdRes );
	ui->timeStep->setValue( tStep );
	ui->reconLevel->setValue( rLevel );
	ui->renderCount->setValue( rCount );

	s->property["reconLevel"] = ui->reconLevel->value();
	s->property["renderCount"] = ui->renderCount->value();
	s->property["renderStartPercentage"] = ui->startPercentage->value();
}
