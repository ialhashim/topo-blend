#include "landmarks_dialog.h"
#include "ui_landmarks_dialog.h"
#include "GraphCorresponder.h"

#include <QFileDialog>
 #include <QItemSelectionModel>

LandmarksDialog::LandmarksDialog(topoblend *topo_blender, QWidget *parent) :  QDialog(parent), ui(new Ui::LandmarksDialog)
{
	tp = topo_blender;
	gcorr = tp->corresponder();
    ui->setupUi(this);
	
	// Set up the table widgets
	QStringList headers;
	headers.push_back("Source");
	headers.push_back("Target");
	ui->landmarksTable->setHorizontalHeaderLabels(headers);
	ui->landmarksTable->setColumnWidth(0,150);
	ui->landmarksTable->setColumnWidth(1, 150);

	headers.push_back("Score");
	ui->corrTable->setHorizontalHeaderLabels(headers);
	ui->corrTable->setColumnWidth(0,100);
	ui->corrTable->setColumnWidth(1,100);
	ui->corrTable->setColumnWidth(2,100);

	// Update the contents
	updateAll();

	/// Landmarks
	// Selections
	this->connect(ui->sClearButton, SIGNAL(clicked()), SLOT(sClearSelections()));
	this->connect(ui->tClearButton, SIGNAL(clicked()), SLOT(tClearSelections()));
	this->connect(ui->sList, SIGNAL(itemSelectionChanged()), SLOT(visualizeSelections()));
	this->connect(ui->tList, SIGNAL(itemSelectionChanged()), SLOT(visualizeSelections()));
	this->connect(ui->landmarksTable, SIGNAL(itemSelectionChanged()), SLOT(visualizeLandmark()));

	// Editing
	this->connect(ui->addButton, SIGNAL(clicked()), SLOT(addLandmark()));
	this->connect(ui->removeButton, SIGNAL(clicked()), SLOT(removeLandmark()));

	// I/O
	this->connect(ui->loadButton, SIGNAL(clicked()), SLOT(loadLandmarks()));
	this->connect(ui->saveButton, SIGNAL(clicked()), SLOT(saveLandmarks()));


	/// Correspondences
	this->connect(ui->sourceID, SIGNAL(valueChanged(int)), SLOT(visualizePart2PartDistance(int)));
	this->connect(ui->runButton, SIGNAL(clicked()), SLOT(computeCorrespondences()));
	tp->connect(ui->p2pButton, SIGNAL(clicked()), SLOT(testPoint2PointCorrespondences()));

	// Reload
	this->connect(ui->reloadButton, SIGNAL(clicked()), SLOT(reload()));
}

LandmarksDialog::~LandmarksDialog()
{
    delete ui;
}

Structure::Graph * LandmarksDialog::sg()
{
	return gcorr->sg;
}

Structure::Graph * LandmarksDialog::tg()
{
	return gcorr->tg;
}

bool LandmarksDialog::sIsLandmark( int i )
{
	return gcorr->sIsLandmark[i];
}

bool LandmarksDialog::tIsLandmark( int i )
{
	return gcorr->tIsLandmark[i];
}

Structure::Node* LandmarksDialog::sNode( int i )
{
	return sg()->nodes[i];
}

Structure::Node* LandmarksDialog::tNode( int i )
{
	return tg()->nodes[i];
}

void LandmarksDialog::updateSList()
{
	// Clear
	ui->sList->clear();

	// Add items
	for (int i = 0; i < sg()->nodes.size(); i++){
		if (!sIsLandmark(i))
			ui->sList->addItem(new QListWidgetItem(sNode(i)->id));
	}
}

void LandmarksDialog::updateTList()
{
	// Clear
	ui->tList->clear();

	// Add items
	for (int i = 0; i < tg()->nodes.size(); i++){
		if (!tIsLandmark(i))
			ui->tList->addItem(new QListWidgetItem(tNode(i)->id));
	}
}

void LandmarksDialog::updateLandmarksTableWidget()
{
	// Clear
	ui->landmarksTable->clearContents();

	// Resize the table
	int nLandmarks = gcorr->landmarks.size();
	int nRows = ui->landmarksTable->rowCount();
	if (nLandmarks > nRows)	{
		for (int i = nRows; i < nLandmarks; i++)
			ui->landmarksTable->insertRow(i);
	}
	if (nLandmarks < nRows)	{
		for (int i = nRows - 1; i > nLandmarks - 1; i--)
			ui->landmarksTable->removeRow(i);
	}

	// Add items
	for (int i = 0; i < nLandmarks; i++)
	{
		QString sItem, tItem;
		VECTOR_PAIR set2set = gcorr->landmarks[i];
		
		foreach (QString s, set2set.first)	sItem += s + ", ";
		sItem.remove(sItem.size()-2, 2);
		ui->landmarksTable->setItem(i, 0, new QTableWidgetItem(sItem));

		foreach (QString t, set2set.second)	tItem += t + ", ";
		tItem.remove(tItem.size()-2, 2);
		ui->landmarksTable->setItem(i, 1, new QTableWidgetItem(tItem));
	}

	ui->landmarksTable->resizeColumnsToContents();
}


void LandmarksDialog::updateLandmarksTab()
{
	updateSList();
	updateTList();
	updateLandmarksTableWidget();
}


void LandmarksDialog::addLandmark()
{
	QList<QListWidgetItem *> sItems = ui->sList->selectedItems();
	QList<QListWidgetItem *> tItems = ui->tList->selectedItems();

	if (sItems.empty() || tItems.empty()) return;


	std::vector<QString> sParts, tParts;

	foreach (QListWidgetItem* item, sItems)
		sParts.push_back(item->text());

	foreach (QListWidgetItem* item, tItems)
		tParts.push_back(item->text());

	gcorr->addLandmarks(sParts, tParts);

	updateLandmarksTab();
}

void LandmarksDialog::removeLandmark()
{
	QModelIndexList selectedIDs = ui->landmarksTable->selectionModel()->selectedRows();
	int n = selectedIDs.size();
	if (n == 0) return;

	int pos = selectedIDs.size();
	int displace = 0;
	foreach (QModelIndex modeID, selectedIDs)
	{
		int row = modeID.row();

		if (row < pos)	pos = row;

		ui->landmarksTable->removeRow(row - displace);

		displace++;
	}
	
	gcorr->removeLandmarks(pos, n);

	updateLandmarksTab();
}

void LandmarksDialog::sClearSelections()
{
	ui->sList->clearSelection();
}

void LandmarksDialog::tClearSelections()
{
	ui->tList->clearSelection();
}

void LandmarksDialog::visualizeSelections()
{
	/// Source
	// Set black for all
	for (int i = 0; i < sg()->nodes.size(); i++)
	{
		Structure::Node * node = sNode(i);
		node->vis_property["color"] = Qt::lightGray;
		node->vis_property["showControl"] = false;
	}

	// Set yellow for selection
	foreach (QListWidgetItem* item, ui->sList->selectedItems())
		sg()->getNode(item->text())->vis_property["color"] = Qt::yellow;


	/// Target
	// Set black for all
	for (int i = 0; i < tg()->nodes.size(); i++)
	{
		Structure::Node * node = tNode(i);
		node->vis_property["color"] = Qt::lightGray;
		node->vis_property["showControl"] = false;
	}

	// Set yellow for selection
	foreach (QListWidgetItem* item, ui->tList->selectedItems())
		tg()->getNode(item->text())->vis_property["color"] = Qt::yellow;

	tp->updateDrawArea();
}


void LandmarksDialog::visualizeLandmark()
{
	QList<QTableWidgetItem *> items = ui->landmarksTable->selectedItems();

	if (! items.empty())
	{
		// Set black for all
		for (int i = 0; i < sg()->nodes.size(); i++)
		{
			Structure::Node * node = sNode(i);
			node->vis_property["color"] = Qt::lightGray;
			node->vis_property["showControl"] = false;
		}

		for (int i = 0; i < tg()->nodes.size(); i++)
		{
			Structure::Node * node = tNode(i);
			node->vis_property["color"] = Qt::lightGray;
			node->vis_property["showControl"] = false;
		}

		// Get the landmark
		int id = ui->landmarksTable->row(items.front());
		VECTOR_PAIR set2set = gcorr->landmarks[id];

		// Set red for landmark
		foreach (QString sID, set2set.first)
			sg()->getNode(sID)->vis_property["color"] = Qt::red;

		foreach (QString tID, set2set.second)
			tg()->getNode(tID)->vis_property["color"] = Qt::red;

		tp->updateDrawArea();
	}

}


void LandmarksDialog::saveLandmarks()
{
	QString filename = QFileDialog::getSaveFileName(NULL,"Save Lanmarks","./", "Test file (*.txt)");

	gcorr->saveLandmarks(filename);
}

void LandmarksDialog::loadLandmarks()
{
	QString filename = QFileDialog::getOpenFileName(NULL,"Load Lanmarks","./", "Test file (*.txt)");
	gcorr->loadLandmarks(filename);

	updateLandmarksTab();
}

void LandmarksDialog::updateCorrTableWidget()
{
	// Clear
	ui->corrTable->clearContents();

	// Add items
	int nCoor = gcorr->correspondences.size();
	int rowID = 0;
	for (int i = 0; i < nCoor; i++)
	{
		// Correspondences and scores
		VECTOR_PAIR vec2vec = gcorr->correspondences[i];
		std::vector<float> scores = gcorr->corrScores[i];

		// One to many
		if (vec2vec.first.size() == 1)
		{
			QString sID = vec2vec.first.front();
			for (int j = 0; j < (int)vec2vec.second.size(); j++, rowID++)
			{
				QString tID = vec2vec.second[j];
				QString score = QString::number(scores[j], 'f', 4);

				// Add row if need
				if (rowID >= ui->corrTable->rowCount())	ui->corrTable->insertRow(rowID);

				ui->corrTable->setItem(rowID, 0, new QTableWidgetItem(sID));
				ui->corrTable->setItem(rowID, 1, new QTableWidgetItem(tID));
				ui->corrTable->setItem(rowID, 2, new QTableWidgetItem(score));
			}
		}
		// Many to one
		else
		{
			QString tID = vec2vec.second.front();
			for (int j = 0; j < (int)vec2vec.first.size(); j++, rowID++)
			{
				QString sID = vec2vec.first[j];
				QString score = QString::number(scores[j], 'f', 4);

				// Add row if need
				if (rowID >= ui->corrTable->rowCount())	ui->corrTable->insertRow(rowID);
				ui->corrTable->setItem(rowID, 0, new QTableWidgetItem(sID));
				ui->corrTable->setItem(rowID, 1, new QTableWidgetItem(tID));
				ui->corrTable->setItem(rowID, 2, new QTableWidgetItem(score));
			}
		}
	}

	// Resize the table
	int nRows = ui->corrTable->rowCount();
	if (rowID < nRows)	{
		for (int i = nRows - 1; i > rowID - 1; i--)
			ui->corrTable->removeRow(i);
	}
}

void LandmarksDialog::updateCorrTab()
{
	updateCorrTableWidget();
}

void LandmarksDialog::updateAll()
{
	ui->sName->setText("Source: " + gcorr->sgName());
	ui->tName->setText("Target: " + gcorr->tgName());

	updateLandmarksTab();
	updateCorrTab();
}

void LandmarksDialog::reload()
{
	gcorr = tp->corresponder();
	if (!gcorr)
	{
		this->done(0);
		return;
	}

	// Update all
	updateAll();
}

void LandmarksDialog::visualizePart2PartDistance( int id)
{
	gcorr->visualizePart2PartDistance(id);

	tp->updateDrawArea();
}

void LandmarksDialog::computeCorrespondences()
{
	gcorr->computeCorrespondences();
	updateCorrTab();
}
