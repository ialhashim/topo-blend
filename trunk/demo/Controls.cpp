#include "DemoPage.h"
#include "DemoGlobal.h"
#include "Controls.h"
#include "ui_Controls.h"
#include <QPaintEvent>
#include <QPainter>

Controls::Controls(QWidget *parent) : QWidget(parent), ui(new Ui::Controls)
{
    ui->setupUi(this);

    // Hide the stupid frame line
    QPalette p = QApplication::palette();
    p.setColor(QPalette::Dark, QColor(0,0,0,0));
    QApplication::setPalette(p);

    // Connections
    this->connect(ui->tabWidget, SIGNAL(currentChanged(int)), SLOT(tabChanged(int)));
}

Controls::~Controls()
{
    delete ui;
}

void Controls::tabChanged(int index)
{
    DemoPages pageType = (DemoPages)index;

    emit(hideAll());

    switch(pageType)
    {
    case SELECT_PAGE:
        emit(showSelect());
        break;

    case MATCH_PAGE:
        emit(showMatch());
        break;

    case CREATE_PAGE:
        emit(showCreate());
        break;
    }
}

