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

private:
    Ui::SchedulerWidget *ui;

public slots:
	void progressStarted();
	void progressDone();
	void changeReconLevel(int value);
	void changeRenderCount(int value);

	void loadSchedule();
	void saveSchedule();
};
