#include "RelationWidget.h"
#include "ui_RelationWidget.h"
#include "../topo-blend/topo-blend.h"

RelationWidget::RelationWidget(QWidget *parent) : QWidget(parent), ui(new Ui::RelationWidget)
{
    ui->setupUi(this);

	QVector<Structure::Graph*> empty;

    r_manager = new RelationManager( NULL, NULL, empty );

	// Compute
    r_manager->connect(ui->parseGlobalSymm, SIGNAL(clicked()), SLOT(parseGlobalReflectionSymm()));
    r_manager->connect(ui->parseConstraintGroup, SIGNAL(clicked()), SLOT(parseModelConstraintGroup()));
    r_manager->connect(ui->parseConstraintPair, SIGNAL(clicked()), SLOT(parseModelConstraintPair()));

	// Evaluate
    r_manager->connect(ui->traceConstraintsAuto, SIGNAL(clicked()), SLOT(traceModelConstraintsAuto()));
    r_manager->connect(ui->traceConstraints, SIGNAL(clicked()), SLOT(traceModelConstraints()));

	// Options
    r_manager->connect(ui->bWithSymm, SIGNAL(clicked(bool)), SLOT(setIsCheckGlobalSymm(bool)));
    r_manager->connect(ui->symmWeight, SIGNAL(textChanged(const QString&)), SLOT(setGlobalSymmWeight(const QString&)) );
    r_manager->connect(ui->bWithGroups, SIGNAL(clicked(bool)), SLOT(setIsTraceGroups(bool)));
    r_manager->connect(ui->groupWeight, SIGNAL(textChanged(const QString&)), SLOT(setGraphWeight(const QString&)) );
    r_manager->connect(ui->bWithPairs, SIGNAL(clicked(bool)), SLOT(setIsTracePairs(bool)));
    r_manager->connect(ui->pairWeight, SIGNAL(textChanged(const QString&)), SLOT(setPairWeight(const QString&)) );
}

RelationWidget::~RelationWidget()
{
    delete ui;
}
