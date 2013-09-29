#include <QDebug>
#include <QGraphicsScene>
#include <QPainter>
#include "ProgressItem.h"

ProgressItem::ProgressItem(QString message, bool isLoading, QGraphicsScene * scene) :
    message(message), isLoading(isLoading), scene(scene), progress(0)
{
    items = new QGraphicsItemGroup(0,scene);
    connect(this, SIGNAL(visibleChanged()), SLOT(visiblityChanged()));

    // Spinner
    spinner = new SpinnerItem(80, 10, QColor(255,153,0));
    this->connect(spinner->timer, SIGNAL(timeout()), this, SLOT(applyProgress()));
    int spinnerX = (scene->width() * 0.5) - (spinner->boundingRect().width() * 0.5);
    int spinnerY = (scene->height() * 0.5) - (spinner->boundingRect().height() * 0.5);
    spinner->setPos( spinnerX, spinnerY );
    items->addToGroup(spinner);

	items->setZValue(10000);
	this->setZValue(100000);

    isSmoothAnimation = false;

    this->setVisible(false);
	scene->addItem(this);
}

ProgressItem::~ProgressItem()
{
    //close();
}

QRectF ProgressItem::boundingRect() const
{
    return items->boundingRect() | messageRect;
}

void ProgressItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)

    painter->setFont(QFont("", 20));

    if(message != oldMessage)
    {
		message = extra + message;

        QRect r;
        r.setWidth( painter->fontMetrics().width(message) );
        r.setHeight( painter->fontMetrics().height() );
        r.moveCenter( QPoint( scene->width() * 0.5, scene->height() * 0.5 ) );
        messageRect = r;

        oldMessage = message;
    }

	painter->translate(1,1);
	painter->setPen(Qt::black);
	painter->drawText(messageRect, message);
	painter->translate(-1,-1);

	painter->setPen(Qt::white);
    painter->drawText(messageRect, message);

    //painter->drawRect(boundingRect()); // DEBUG
}

void ProgressItem::startProgress()
{
	this->progress = 0;
	this->extra = "";

    this->show();
    spinner->timer->start( isSmoothAnimation ? 50 : 100 );
}

void ProgressItem::setProgress(double new_progress)
{
    this->progress = new_progress;
}

void ProgressItem::stopProgress()
{
    if(spinner->timer->isActive())
        spinner->timer->stop();
}

void ProgressItem::applyProgress()
{
    if( isLoading )
    {
        int x = spinner->width;
        int y = spinner->height;

        spinner->translate( 0.5 * x,  0.5 * y);
        spinner->rotate(10);
        spinner->translate(-0.5 * x, -0.5 * y);
    }
    else
    {
        int progressInt = qMin(100, int(progress * 100) + 1);
        message = QString::number(progressInt) + " %";
        spinner->spanAngle = 360 * progress;

        QRectF myRect = boundingRect();
        this->update( myRect.x(), myRect.y(), myRect.width(), myRect.height() );

        if(progress >= 1.0) spinner->timer->stop();
    }
}

void ProgressItem::show()
{
    this->setVisible( true );
}

void ProgressItem::close()
{
    spinner->timer->stop();
}

void ProgressItem::visiblityChanged()
{
    items->setVisible( this->isVisible() );
}

void ProgressItem::setExtra( QString extraText )
{
	extra = extraText;
}
