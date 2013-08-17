#pragma once

#include <QWidget>
#include <QLabel>
namespace Ui { class Wizard; }

#include "QtAwesome/QtAwesome.h"
extern QtAwesome* awesome;

class Wizard : public QWidget
{
    Q_OBJECT
    
public:
    explicit Wizard(QWidget *parent = 0);
    ~Wizard();

	QVector<QLabel*> icons;
	QVector<QLayout*> layouts;
	
	void setOpacity( QString objName, double toValue );

public slots:
	void loadShapeA();
	void loadShapeB();

private:
    Ui::Wizard *ui;
};
