#include "Session.h"

Session::Session(Scene *scene, ShapesGallery *gallery, QObject *parent) : QObject(parent),
    s(scene), g(gallery)
{

}

void Session::shapeChanged(int shapeID, QGraphicsItem * shapeItem)
{

}
