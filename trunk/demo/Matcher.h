#pragma once

#include "DemoPage.h"

class Matcher : public DemoPage
{
    Q_OBJECT
public:
    explicit Matcher(Scene * scene, QString title);
    
signals:
    
public slots:
    void show();
    void hide();
};
