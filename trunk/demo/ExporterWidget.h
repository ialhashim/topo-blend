#pragma once

#include "Session.h"
#include <QWidget>

namespace Ui {
class ExporterWidget;
}

class ExporterWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit ExporterWidget(Session * session, QWidget *parent = 0);
    ~ExporterWidget();

    Ui::ExporterWidget *ui;
    Session * session;

public slots:
	void addInBetween();
	void removeInBetween();

	void exportSet();

	void generateLog(QMap<QString, QVariant> log);

	void keyUp(QKeyEvent*);
signals:

};
