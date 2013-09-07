#include "DemoGlobal.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Controls.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->resize(this->sizeHint());

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

    // Create controls
    control = new Controls;
    QGraphicsProxyWidget * proxy = scene->addWidget(control);
    proxy->setPos((scene->width() * 0.5) - (proxy->boundingRect().width() * 0.5),
                   scene->height() - proxy->boundingRect().height());

    // Loading shapes
    gallery = new ShapesGallery(scene, "Select two shapes");
    gallery->loadDataset( getDataset() );
    gallery->connect(scene, SIGNAL(wheelEvents(QGraphicsSceneWheelEvent*)), SLOT(wheelEvent(QGraphicsSceneWheelEvent*)));

    // Match shapes
    Matcher * matcher = new Matcher(scene, "Match parts");

    // Create shapes
    Blender * blender = new Blender(scene, "Blend shapes");

	// Connect matcher and blender
	blender->connect(matcher, SIGNAL(corresponderCreated(GraphCorresponder *)), SLOT(setGraphCorresponder(GraphCorresponder *)));

    // Create session
    session = new Session(scene, gallery, control, matcher, blender, this);
    session->connect(gallery, SIGNAL(shapeChanged(int,QGraphicsItem*)), SLOT(shapeChanged(int,QGraphicsItem*)));
    scene->connect(session, SIGNAL(update()), SLOT(update()));

    gallery->layout();
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
        QDir d(datasetPath + "/" + subdir);

        dataset[subdir]["Name"] = subdir;
        dataset[subdir]["graphFile"] = d.absolutePath() + "/" + d.entryList(QStringList() << "*.xml", QDir::Files).join("");
        dataset[subdir]["thumbFile"] = d.absolutePath() + "/" + d.entryList(QStringList() << "*.png", QDir::Files).join("");
        dataset[subdir]["objFile"] = d.absolutePath() + "/" + d.entryList(QStringList() << "*.obj", QDir::Files).join("");
    }

    return dataset;
}

