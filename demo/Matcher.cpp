#include "Matcher.h"
using namespace Structure;

Matcher::Matcher(Scene * scene, QString title) : DemoPage(scene,title)
{
	// Fill in color sets
	for(int i = 0; i < 20; i++)
	{
		double blueH = 2.0 / 3.0;
		double redH = 0;
		double range = 0.1;

		double coldH = fmod(1.0 + uniformRand( -range + blueH, range + blueH ), 1.0);
		double warmH = fmod(1.0 + uniformRand( -range + redH , range + redH  ), 1.0);

		double sat = 0.5;

		coldColors.push_back(QColor::fromHsvF(coldH, sat, 1));
		warmColors.push_back(QColor::fromHsvF(warmH, sat, 1));
	}
}

void Matcher::show()
{
    if(!s->isInputReady()) return;

    QRectF r0 = s->inputGraphs[0]->boundingRect();
    QRectF r1 = s->inputGraphs[1]->boundingRect();

    property["r0"] = r0;
    property["r1"] = r1;

    // Scale in place
    QPointF oldCenter0 = r0.center();
    QPointF oldCenter1 = r1.center();

    double s_factor = (s->width() * 0.5) / r0.width();

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

    // Make corresponder
    gcorr = new GraphCorresponder(s->inputGraphs[0]->g, s->inputGraphs[1]->g);
    gcorr->computeCorrespondences();
	emit( corresponderCreated(gcorr) );

    // Visualize computed correspondences
    visualize();

	// Enable graph picking
	s->setProperty("graph-pick", true);

    DemoPage::show();
}

void Matcher::hide()
{
    if(!s->isInputReady() || !property.contains("r0")) return;

    QParallelAnimationGroup * animGroup = new QParallelAnimationGroup;
    animGroup->addAnimation( s->inputGraphs[0]->animateTo( property["r0"].toRectF() ) );
    animGroup->addAnimation( s->inputGraphs[1]->animateTo( property["r1"].toRectF() ) );
    animGroup->start( QAbstractAnimation::DeleteWhenStopped );

	// Restore colors
	QVector<Structure::Graph*> graphs; 
	graphs << s->inputGraphs[0]->g << s->inputGraphs[1]->g;
	foreach(Structure::Graph * g, graphs){
		foreach(Node * n, g->nodes){
			n->vis_property["meshSolid"] = true;
			n->vis_property["meshColor"].setValue( QColor(180,180,180) );
		}
	}

	// Disable graph picking
	s->setProperty("graph-pick", false);

    DemoPage::hide();
}

void Matcher::visualize()
{
    // Color previously assigned correspondences
    if( gcorr ) {
        Graph * sourceGraph = s->inputGraphs[0]->g;
        Graph * targetGraph = s->inputGraphs[1]->g;

		int c_cold = 0, c_warm = 0;

        foreach (PART_LANDMARK vector2vector, gcorr->correspondences){
            QColor curColor = coldColors[c_cold++ % coldColors.size()];

            foreach (QString strID, vector2vector.first){
                sourceGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );
                sourceGraph->getNode( strID )->property["is_corresponded"] = true;
            }

            foreach (QString strID, vector2vector.second){
                targetGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );
                targetGraph->getNode( strID )->property["is_corresponded"] = true;
            }
        }

        foreach (PART_LANDMARK vector2vector, gcorr->landmarks){
            QColor curColor = warmColors[c_warm++ % warmColors.size()];

            foreach (QString strID, vector2vector.first){
                sourceGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );
                sourceGraph->getNode( strID )->property["is_corresponded"] = true;
            }

            foreach (QString strID, vector2vector.second){
                targetGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );
                targetGraph->getNode( strID )->property["is_corresponded"] = true;
            }
        }
    }
}
