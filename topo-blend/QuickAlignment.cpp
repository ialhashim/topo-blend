#include "QuickAlignment.h"
#include "ui_QuickAlignment.h"
#include <QMatrix4x4>

#define M_PI_4     0.785398163397448309616
#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

QuickAlignment::QuickAlignment(Structure::Graph *graphA, Structure::Graph *graphB,
                               QWidget *parent) : QDialog(parent), ui(new Ui::QuickAlignment)
{
	ui->setupUi(this);

    this->ga = graphA;
    this->gb = graphB;

    // Populate lists
    foreach(Structure::Node * n, ga->nodes) ui->list1->addItem(new QListWidgetItem(n->id));
    foreach(Structure::Node * n, gb->nodes) ui->list2->addItem(new QListWidgetItem(n->id));

	// Connections
	this->connect(ui->alignButton, SIGNAL(clicked()), SLOT(doPartAlignment()));
	this->connect(ui->list1, SIGNAL(itemSelectionChanged()), SLOT(visualizeSelections()));
	this->connect(ui->list2, SIGNAL(itemSelectionChanged()), SLOT(visualizeSelections()));
}

QuickAlignment::~QuickAlignment()
{
    delete ui;
}

int QuickAlignment::quadrant(Vector3 v)
{
	// Treat as degenerate vector
	if(v.norm() < 1e-10) return -1;

	v.normalize();

	double threshold = 0.1;
	if(abs(v[0]) < threshold) return ((v[1] > 0) ? 2 : 6);	// On y-axis
	if(abs(v[1]) < threshold) return ((v[0] > 0) ? 0 : 4);	// On x-axis

	if(v[0] * v[1] > 0 && v[0] > 0) return 1;	// First quadrant
	if(v[0] < 0 && v[1] > 0) return 3;			// Second
	if(v[0] < 0 && v[1] < 0) return 5;			// Third
	return 7;									// Fourth
}

void QuickAlignment::doPartAlignment()
{
	QString nodeA = ui->list1->selectedItems().first()->text();
	QString nodeB = ui->list2->selectedItems().first()->text();

	Structure::Node * na = ga->getNode( nodeA );
	Structure::Node * nb = gb->getNode( nodeB );
	
	Eigen::AlignedBox3d boxA, boxB;
	QMatrix4x4 mat;

	// Scaling
	boxA = na->bbox(); boxB = nb->bbox();
	double maxDimA = qMax( qMax(boxA.diagonal().x(), boxA.diagonal().y()), boxA.diagonal().z() );
	double maxDimB = qMax( qMax(boxB.diagonal().x(), boxB.diagonal().y()), boxB.diagonal().z() );
	double scale = maxDimA / maxDimB;
	mat.setToIdentity();
	mat.scale(scale);
	gb->transform(mat);

	// Rotation
	boxA = na->bbox(); boxB = nb->bbox();
	Vector3d va = Vector3d(boxA.center().x(), boxA.center().y(), 0).normalized();
	Vector3d vb = Vector3d(boxB.center().x(), boxB.center().y(), 0).normalized();
	int qa = quadrant(va);
	int qb = quadrant(vb);
	int diff = qb - qa;

	mat.setToIdentity();
	mat.rotate( RADIANS_TO_DEGREES(diff * M_PI_4), QVector3(0,0,1) );
	ga->transform( mat );

	// Translation
	boxA = na->bbox(); boxB = nb->bbox();
	mat.setToIdentity();
	mat.translate( 0, 0, boxB.center().z() - boxA.center().z() );
	ga->transform( mat );


	emit( updateView() );
}

void QuickAlignment::visualizeSelections()
{
	QList<QListWidgetItem *> items1 = ui->list1->selectedItems();
	QList<QListWidgetItem *> items2 = ui->list2->selectedItems();

	// Set black for all
	foreach (Structure::Node * node,  ga->nodes)
	{
		node->vis_property["color"] = Qt::lightGray;
		node->vis_property["showControl"] = false;
	}
	foreach (Structure::Node * node,  gb->nodes)
	{
		node->vis_property["color"] = Qt::lightGray;
		node->vis_property["showControl"] = false;
	}

	// Set red for landmark
	foreach (QListWidgetItem * item, items1)
		ga->getNode(item->text())->vis_property["color"] = Qt::red;
	
	foreach (QListWidgetItem * item, items2)
		gb->getNode(item->text())->vis_property["color"] = Qt::green;

	emit( updateView() );
}
