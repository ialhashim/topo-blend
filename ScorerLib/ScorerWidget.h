#ifndef SCORERWIDGET_H
#define SCORERWIDGET_H

#include <QWidget>
#include "ScorerManager.h"

namespace Ui {class ScorerWidget;};

class ScorerWidget : public QWidget
{
	Q_OBJECT

public:
	ScorerWidget(QWidget *parent = 0);
	~ScorerWidget();

	ScorerManager * s_manager;
private:
	Ui::ScorerWidget *ui;
};

#endif // SCORERWIDGET_H
