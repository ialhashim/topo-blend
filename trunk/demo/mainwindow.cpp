#include "DemoGlobal.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Controls.h"
#include "ExporterWidget.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->resize(this->sizeHint());

	ui->logWidget->setVisible(false);

    // Anti-aliasing when using QGLWidget or subclasses
    QGLFormat glf = QGLFormat::defaultFormat();
    glf.setSamples(8);
    QGLFormat::setDefaultFormat(glf);

    viewport = new QGLWidget(glf);
    viewport->makeCurrent();

    ui->graphicsView->setViewport( viewport );
    ui->graphicsView->setViewportUpdateMode( QGraphicsView::FullViewportUpdate );
    ui->graphicsView->setScene( (scene = new Scene(this)) );
    ui->graphicsView->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setRenderHint(QPainter::HighQualityAntialiasing, true);
    ui->graphicsView->setRenderHint(QPainter::SmoothPixmapTransform, true);

	this->connect(scene, SIGNAL(message(QString)), SLOT(message(QString)));
	this->connect(scene, SIGNAL(keyUpEvent(QKeyEvent*)), SLOT(keyUpEvent(QKeyEvent*)));

    // Create controls
    control = new Controls;
    QGraphicsProxyWidget * proxy = scene->addWidget(control);
    proxy->setPos((scene->width() * 0.5) - (proxy->boundingRect().width() * 0.5),
                   scene->height() - proxy->boundingRect().height());

    /// Create [Shape gallery + Matcher + Blender]
    ShapesGallery * gallery = new ShapesGallery(scene, "Select two shapes");
	Matcher * matcher = new Matcher(scene, "Match parts");
	Blender * blender = new Blender(scene, "Blended shapes");

	// Connect all pages to logger
	this->connect(gallery, SIGNAL(message(QString)), SLOT(message(QString)));
	this->connect(matcher, SIGNAL(message(QString)), SLOT(message(QString)));
	this->connect(blender, SIGNAL(message(QString)), SLOT(message(QString)));

	// Place log window nicely
	this->ui->logWidget->setGeometry(geometry().x() - (width() * 0.21), geometry().y(), width() * 0.2, height() * 0.5);

	// Connect matcher and blender
	blender->connect(matcher, SIGNAL(corresponderCreated(GraphCorresponder *)), SLOT(setGraphCorresponder(GraphCorresponder *)), Qt::DirectConnection);
	this->connect(blender, SIGNAL(showLogWindow()), SLOT(showLogWindow()));

	// Connect
    gallery->connect(scene, SIGNAL(wheelEvents(QGraphicsSceneWheelEvent*)), SLOT(wheelEvent(QGraphicsSceneWheelEvent*)), Qt::DirectConnection);

    // Create session
    session = new Session(scene, gallery, control, matcher, blender, this);
    session->connect(gallery, SIGNAL(shapeChanged(int,QGraphicsItem*)), SLOT(shapeChanged(int,QGraphicsItem*)), Qt::DirectConnection);
    scene->connect(session, SIGNAL(update()), SLOT(update()));

	// Everything is ready, load shapes now:
	QString datasetFolder = "dataset";

	gallery->loadDataset( getDataset(datasetFolder) );
	gallery->layout();

	control->loadCategories( datasetFolder );

	// Lastly, create exporter widget
	ewidget = new ExporterWidget( session );
	ewidget->move(20,120);
	ewidget->resize(ewidget->sizeHint());

	// Show inside the scene
	if( false )
	{
		QGraphicsProxyWidget * eproxy = scene->addWidget( ewidget, Qt::Tool | Qt::WindowTitleHint );
		eproxy->setZValue(1e30);
	}
}

MainWindow::~MainWindow()
{
    delete ui;
}

DatasetMap MainWindow::getDataset(QString datasetPath)
{
    DatasetMap dataset;

    QDir datasetDir(datasetPath);
    QStringList subdirs = datasetDir.entryList(QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);

    foreach(QString subdir, subdirs)
    {
		// Special folders
		if(subdir == "corr") continue;

        QDir d(datasetPath + "/" + subdir);

        dataset[subdir]["Name"] = subdir;
        dataset[subdir]["graphFile"] = d.absolutePath() + "/" + d.entryList(QStringList() << "*.xml", QDir::Files).join("");
        dataset[subdir]["thumbFile"] = d.absolutePath() + "/" + d.entryList(QStringList() << "*.png", QDir::Files).join("");
        dataset[subdir]["objFile"] = d.absolutePath() + "/" + d.entryList(QStringList() << "*.obj", QDir::Files).join("");
    }

    return dataset;
}

void MainWindow::message(QString m)
{
	ui->logger->addItem(m);
	ui->logger->scrollToBottom();
}

void MainWindow::keyUpEvent(QKeyEvent* keyEvent)
{
	// Toggle log window visibility
	if(keyEvent->key() == Qt::Key_L)
	{
		ui->logWidget->setVisible(!ui->logWidget->isVisible());
	}

	if(keyEvent->key() == Qt::Key_E)
	{
		int y = (QDesktopWidget().screenGeometry().height() - this->height()) * 0.5;
		this->move(QDesktopWidget().screenGeometry().width() - this->width() - 12, y);
		ewidget->move( this->pos().x() - ewidget->width() - 12, y );
		this->setFocus();
	}
}

void MainWindow::showLogWindow()
{
	ui->logWidget->setVisible(true);
}
