#pragma once
#include <QObject>

#include "DemoGlobal.h"
#include "Scene.h"

enum DemoPages{ SELECT_PAGE, MATCH_PAGE, CREATE_PAGE };

class DemoPage : public QObject{
    Q_OBJECT
public:
    DemoPage(Scene * scene, QString title = "") : s(scene)
    {
        visible = false;

        QFont titleFont("", 25);
        QGraphicsTextItem * titleItem = s->addText(title, titleFont);
        titleItem->setPos( (s->width()  * 0.50) - (titleItem->boundingRect().width() * 0.5), 0);
        titleItem->setDefaultTextColor(Qt::white);
        titleItem->setVisible(false);
        items.push_back(titleItem);
    }

    PropertyMap property;
    bool isVisible() { return visible; }

protected:
    Scene * s;
    QVector<QGraphicsItem*> items;
    bool visible;

public slots:
    void hide()
    {
        this->visible = false;
        foreach(QGraphicsItem * item, items) item->hide();
    }

    void show()
    {
        this->visible = true;
        foreach(QGraphicsItem * item, items) item->show();
    }
};
