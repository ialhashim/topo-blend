#include "RelationWidget.h"
#include "ui_RelationWidget.h"

RelationWidget::RelationWidget(topoblend * tb, QWidget *parent) : QWidget(parent), ui(new Ui::RelationWidget)
{
    ui->setupUi(this);

    r_manager = new RelationManager( tb );

    r_manager->connect(ui->parseGlobalSymm, SIGNAL(clicked()), SLOT(parseGlobalReflectionSymm()));
    r_manager->connect(ui->parseConstraintGroup, SIGNAL(clicked()), SLOT(parseModelConstraintGroup()));
    r_manager->connect(ui->parseConstraintPair, SIGNAL(clicked()), SLOT(parseModelConstraintPair()));
    r_manager->connect(ui->traceConstraintsAuto, SIGNAL(clicked()), SLOT(traceModelConstraintsAuto()));
    r_manager->connect(ui->traceConstraints, SIGNAL(clicked()), SLOT(traceModelConstraints()));
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
