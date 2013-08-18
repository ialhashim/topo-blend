#pragma once
#include "topo-blend.h"

class CorrespondenceManager : public QObject
{
	Q_OBJECT
public:
    CorrespondenceManager(topoblend * topo_blender) : tb(topo_blender) {}
	topoblend * tb;

	GraphCorresponder* makeCorresponder();
	void assignCorrespondence();
	void correspondenceMode();
	void clearCorrespondence();
	void drawWithNames();
	void exitCorrespondenceMode();
	void visualizeAsSolids();
};
