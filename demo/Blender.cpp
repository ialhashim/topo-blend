#include "Blender.h"
#include "ProgressItem.h"

Blender::Blender(Scene * scene, QString title) : DemoPage(scene,title)
{
	this->numSuggestions = 4;

	int padding = 5;
	int blendPathHeight = (s->height() / (numSuggestions * 1.5)) * 0.98;
	int totalHeight = numSuggestions * (blendPathHeight + padding);
	int startY = (s->height() * 0.5 - totalHeight * 0.5) - 45;

	for(int i = 0; i < numSuggestions; i++)
	{
		QGraphicsItem * blendPathBack = scene->addRect(0,0,s->width() * 0.5, blendPathHeight, QPen(), QColor::fromRgbF(0,0,0.25,0.5) );

		blendPathBack->setY( startY + (i * (blendPathHeight + padding)) );
		blendPathBack->setX( 0.5*s->width() - 0.5*blendPathBack->boundingRect().width() );

		blendPathBack->setZValue(-999);
		blendPathBack->setVisible(false);

		items.push_back( blendPathBack );
	}
}

void Blender::show()
{
    if(!s->isInputReady()) return;

    QRectF r0 = s->inputGraphs[0]->boundingRect();
    QRectF r1 = s->inputGraphs[1]->boundingRect();

    property["r0"] = r0;
    property["r1"] = r1;

    // Scale in place
    QPointF oldCenter0 = r0.center();
    QPointF oldCenter1 = r1.center();

    double s_factor = (s->width() * 0.25) / r0.width();

    QMatrix m;
    m.scale(s_factor,s_factor);
    r0 = m.mapRect(r0);
    r1 = m.mapRect(r1);
    r0.translate(oldCenter0 - r0.center());
    r1.translate(oldCenter1 - r1.center());
    r0.moveLeft(0);
    r1.moveRight(s->width());

    QParallelAnimationGroup * animGroup = new QParallelAnimationGroup;
    animGroup->addAnimation( s->inputGraphs[0]->animateTo(r0) );
    animGroup->addAnimation( s->inputGraphs[1]->animateTo(r1) );
    animGroup->start( QAbstractAnimation::DeleteWhenStopped );

    DemoPage::show();
}

void Blender::hide()
{
    if(!s->isInputReady() || !property.contains("r0")) return;

    QParallelAnimationGroup * animGroup = new QParallelAnimationGroup;
    animGroup->addAnimation( s->inputGraphs[0]->animateTo( property["r0"].toRectF() ) );
    animGroup->addAnimation( s->inputGraphs[1]->animateTo( property["r1"].toRectF() ) );
    animGroup->start( QAbstractAnimation::DeleteWhenStopped );

    DemoPage::hide();
}
