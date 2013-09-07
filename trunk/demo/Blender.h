#pragma once

#include "DemoPage.h"

class Blender : public DemoPage
{
    Q_OBJECT
public:
    explicit Blender(Scene * scene, QString title);
    
	int numSuggestions;

signals:
    
public slots:
    void show();
    void hide();
};
