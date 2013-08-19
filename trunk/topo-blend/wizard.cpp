#include "wizard.h"
#include "ui_wizard.h"

#include "topo-blend.h"
#include "graphs-manager.h"
#include "correspondence-manager.h"
#include "Scheduler.h"
#include "SchedulerWidget.h"

#include <QParallelAnimationGroup>
#include <QPropertyAnimation>
#include <QGraphicsOpacityEffect>
Q_DECLARE_METATYPE(QGraphicsOpacityEffect*)

QtAwesome* awesome = new QtAwesome( qApp );

// Styling
QString style_ok = "color: rgb(81, 163, 81)";
QString style_wait = "color: rgb(248, 148, 6)";
QString style_bad = "color: rgb(189, 54, 47)";
QString style_info = "color: rgb(47, 150, 180)";

Wizard::Wizard(topoblend *tobo_blend, QWidget * parentWidget) : QWidget(parentWidget), tb(tobo_blend), ui(new Ui::Wizard)
{
	awesome->initFontAwesome();

    ui->setupUi(this);

	// Connections
	{
		this->connect(ui->loadAButton, SIGNAL(clicked()), SLOT(loadShapeA()));
		this->connect(ui->loadBButton, SIGNAL(clicked()), SLOT(loadShapeB()));
		this->connect(ui->matchingButton, SIGNAL(clicked()), SLOT(matchingButton()));
		this->connect(ui->generateButton, SIGNAL(clicked()), SLOT(generateBlend()));
	}

	// Layouts
	{
		layouts.push_back(ui->layoutLoading);
		layouts.push_back(ui->layoutMatching);
		layouts.push_back(ui->layoutOrder);
		layouts.push_back(ui->layoutQuality);
		layouts.push_back(ui->layoutGenerate);

		foreach(QLayout * l, layouts){
			for(int i = 0; i < l->count(); i++){
				QWidget * item = l->itemAt(i)->widget();
				if(!item) continue;
				
				// Add transparency effect
				QGraphicsOpacityEffect * effect = new QGraphicsOpacityEffect(item);
				effect->setOpacity(0);
				item->setGraphicsEffect(effect);
				QVariant var; var.setValue(effect);
				item->setProperty("opacity", var);
			}
		}

	}

	// Icons / check marks
	{
		icons.push_back(ui->checkLoad);
		icons.push_back(ui->checkMatch);
		icons.push_back(ui->checkOrder);
		icons.push_back(ui->checkQuality);

		QFont fnt = awesome->font(16);
		foreach(QLabel * l, icons) {
			l->setFont( fnt );
			l->setStyleSheet( style_wait ); 
		}
	}

	this->setWindowTitle("Blending Tool");
	this->show();

	// Fade window in
	if(false){
		this->setWindowOpacity(0);

		QPropertyAnimation *animation = new QPropertyAnimation(this,"windowOpacity");
		animation->setDuration(1000);
		animation->setStartValue(0.0);
		animation->setEndValue(1.0);
		animation->setEasingCurve(QEasingCurve::OutCubic);
		animation->start();
	}

	// Fade first step in
	setOpacity("layoutLoading", 1.0);
}

void Wizard::setOpacity( QString objName, double toValue )
{
	QParallelAnimationGroup * animGroup = new QParallelAnimationGroup;

	foreach(QLayout * l, layouts){
		if(l->objectName() != objName) continue;
		for(int i = 0; i < l->count(); i++){
			QWidget * item = l->itemAt(i)->widget();
			if(!item) continue;

			QGraphicsOpacityEffect * opacityEffect = item->property("opacity").value<QGraphicsOpacityEffect*>();
			QPropertyAnimation* anim = new QPropertyAnimation;
			anim->setTargetObject(opacityEffect);
			anim->setPropertyName("opacity");
			anim->setDuration(1000);
			anim->setStartValue(opacityEffect->opacity());
			anim->setEndValue(toValue);
			anim->setEasingCurve(QEasingCurve::OutQuad);

			animGroup->addAnimation(anim);
		}
	}

	animGroup->start( QAbstractAnimation::DeleteWhenStopped );
}

Wizard::~Wizard()
{
    delete ui;
}

void Wizard::loadShapeA()
{
	tb->g_manager->loadModel();
	tb->c_manager->visualizeAsSolids();

	ui->loadAButton->setStyleSheet(style_ok);

	tb->updateDrawArea();
}

void Wizard::loadShapeB()
{
	tb->g_manager->loadModel();
	tb->c_manager->visualizeAsSolids();

	ui->loadBButton->setStyleSheet(style_ok);

	setOpacity("layoutMatching", 1.0);

	tb->updateDrawArea();
}

void Wizard::matchingButton()
{
	if(ui->matchingButton->text().contains("Set..")){
		ui->matchingButton->setText("Done?");
		tb->c_manager->correspondenceMode();
	}else{
		ui->matchingButton->setText("Done");
		ui->matchingButton->setStyleSheet(style_ok);

		//setOpacity("layoutOrder", 1.0);
		setOpacity("layoutQuality", 1.0);
		setOpacity("layoutGenerate", 1.0);

		tb->doBlend();
	}

	tb->updateDrawArea();
}

void Wizard::generateBlend()
{
	tb->scheduler->doBlend();

	tb->updateDrawArea();
}
