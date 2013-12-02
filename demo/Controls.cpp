#include "DemoPage.h"
#include "DemoGlobal.h"
#include "Controls.h"
#include "ui_Controls.h"
#include <QPaintEvent>
#include <QPainter>

Controls::Controls(QWidget *parent) : QWidget(parent), ui(new Ui::Controls)
{
    ui->setupUi(this);

	ui->categoriesBox->addItem("All");

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

void Controls::loadCategories( QString datasetPath )
{
	QFile file( datasetPath + "/categories.txt" );
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;

	PropertyMap categories;

	QTextStream in(&file);
	while (!in.atEnd()){
		QStringList line = in.readLine().split("|");
		if(line.size() < 2) continue;
	
		QString catName = line.front().trimmed();
		QStringList catElements = line.back().split(QRegExp("[ \t]"), QString::SkipEmptyParts);

		ui->categoriesBox->addItem(catName);

		categories[catName] = catElements;
	}

	emit( categoriesLoaded(categories) );
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
		ui->autoButton->setChecked(true);
        emit(showMatch());
        break;

    case CREATE_PAGE:
        emit(showCreate());
        break;
    }
}

void Controls::forceManualMatch()
{
	ui->manualButton->setChecked(true);
}

void Controls::disableTabs()
{
	ui->tabWidget->setEnabled(false);
}

void Controls::enableTabs()
{
	ui->tabWidget->setEnabled(true);
}
