#pragma once

#include <QWidget>
#include "Scheduler.h"

namespace Ui {
class SchedulerWidget;
}

class SchedulerWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit SchedulerWidget(Scheduler * scheduler, QWidget *parent = 0);
    ~SchedulerWidget();
    
    Scheduler * s;

	// Parameters
	void setParams( double gdRes, double tStep, int rLevel, int rCount );
	double gdResolution();
	double timeStep();
	int reconLevel();
	int renderCount();

private:
    Ui::SchedulerWidget *ui;
	bool isShowControls;

public slots:
	void updateNodesList();
	void progressStarted();
	void progressDone();
	void changeReconLevel(int value);
	void changeRenderCount(int value);
	void changeRenderStartPercentage(int value);
	
	void loadSchedule();
	void saveSchedule();

	void cleanUp();
	void toggleOptions();
};
