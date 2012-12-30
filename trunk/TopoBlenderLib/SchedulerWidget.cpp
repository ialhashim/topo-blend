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
}

SchedulerWidget::~SchedulerWidget()
{
    delete ui;
}
