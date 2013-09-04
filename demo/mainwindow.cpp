#include "DemoGlobal.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Anti-aliasing when using QGLWidget or subclasses
    QGLFormat glf = QGLFormat::defaultFormat();
    glf.setSamples(8);
    QGLFormat::setDefaultFormat(glf);

    viewport = new QGLWidget(glf);
    viewport->makeCurrent();

    ui->graphicsView->setViewport( viewport );
    ui->graphicsView->setViewportUpdateMode( QGraphicsView::FullViewportUpdate );
    ui->graphicsView->setScene( (s = new Scene(this)) );
    ui->graphicsView->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setRenderHint(QPainter::HighQualityAntialiasing, true);
    ui->graphicsView->setRenderHint(QPainter::SmoothPixmapTransform, true);

    // Load shapes
    gallery = new ShapesGallery(s);
    gallery->loadDataset( getDataset() );
    gallery->layout();
    gallery->connect(s, SIGNAL(wheelEvents(QGraphicsSceneWheelEvent*)), SLOT(wheelEvent(QGraphicsSceneWheelEvent*)));

    // Create session
    session = new Session(s, gallery, this);
    session->connect(gallery, SIGNAL(shapeChanged(int,QGraphicsItem*)), SLOT(shapeChanged(int,QGraphicsItem*)));
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

