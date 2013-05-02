#include "landmarks_dialog.h"
#include "ui_landmarks_dialog.h"
#include "GraphCorresponder.h"

#include <QFileDialog>
 #include <QItemSelectionModel>

LandmarksDialog::LandmarksDialog(topoblend *topo_blender, QWidget *parent) :  QDialog(parent), ui(new Ui::LandmarksDialog)
{
	tb = topo_blender;
	gcorr = tb->corresponder();
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

	/// Point Landmarks
	this->connect(ui->addPointLandmarkButton, SIGNAL(clicked()), SLOT(addPointLandmark()));
	this->connect(ui->removePointLandmarkButton, SIGNAL(clicked()), SLOT(removePointLandmark()));
	this->connect(ui->pointLandmarksList, SIGNAL(itemSelectionChanged()), SLOT(visualizeCurrentPointLandmark()));
	this->connect(ui->visAllButton, SIGNAL(clicked()), SLOT(visualizeAllPointLandmarks()));


	/// Part Landmarks
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

	/// Alignment
	this->connect(ui->showAABB, SIGNAL(stateChanged(int)), SLOT(showAABB(int)));
	this->connect(ui->normalizeButton, SIGNAL(clicked()), SLOT(normalize()));
	this->connect(ui->rotateButton, SIGNAL(clicked()), SLOT(rotateGraph()));
	this->connect(ui->scaleTarget, SIGNAL(valueChanged(int)), SLOT(scaleGraph(int)));

	this->connect(ui->alignList1, SIGNAL(itemSelectionChanged()), SLOT(visualizeSelections()));
	this->connect(ui->alignList2, SIGNAL(itemSelectionChanged()), SLOT(visualizeSelections()));
	this->connect(ui->basicAlignButton, SIGNAL(clicked()), SLOT(basicAlign()));

	/// Correspondences
	this->connect(ui->prepareButton, SIGNAL(clicked()), SLOT(prepareMatrices()));

	this->connect(ui->spatialWeight, SIGNAL(valueChanged(double)), SLOT(setSpatialWeight(double)));
	this->connect(ui->structuralWeight, SIGNAL(valueChanged(double)), SLOT(setStructuralWeight(double)));
	this->connect(ui->sizeWeight, SIGNAL(valueChanged(double)), SLOT(setSizeWeight(double)));
	this->connect(ui->orientationWeight, SIGNAL(valueChanged(double)), SLOT(setOrientationWeight(double)));
	this->connect(ui->computeDisMButton, SIGNAL(clicked()), SLOT(computeDisM()));

	this->connect(ui->scoreThreshold, SIGNAL(valueChanged(double)), SLOT(setScoreThreshold(double)));
	this->connect(ui->computeCorrButton, SIGNAL(clicked()), SLOT(computePartToPartCorrespondences()));

	this->connect(ui->alignNodesButton, SIGNAL(clicked()), SLOT(alignAllNodes()));

	this->connect(ui->runButton, SIGNAL(clicked()), SLOT(computeCorrespondences()));
	this->connect(ui->corrTable, SIGNAL(cellClicked(int, int)), SLOT(visualizeCorr(int, int)));

	this->connect(ui->sourceID, SIGNAL(valueChanged(int)), SLOT(visualizePart2PartDistance(int)));
	tb->connect(ui->p2pButton, SIGNAL(clicked()), SLOT(testPoint2PointCorrespondences()));

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
		PART_LANDMARK set2set = gcorr->landmarks[i];
		
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
	updatePointLandmarksList();

	updateSList();
	updateTList();
	updateLandmarksTableWidget();
}


void LandmarksDialog::addLandmark()
{
	QList<QListWidgetItem *> sItems = ui->sList->selectedItems();
	QList<QListWidgetItem *> tItems = ui->tList->selectedItems();

	if (sItems.empty() || tItems.empty()) return;


	QVector<QString> sParts, tParts;

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

	foreach (QListWidgetItem* item, ui->alignList1->selectedItems())
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

	foreach (QListWidgetItem* item, ui->alignList2->selectedItems())
		tg()->getNode(item->text())->vis_property["color"] = Qt::yellow;

	tb->updateDrawArea();
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
		PART_LANDMARK set2set = gcorr->landmarks[id];

		// Set red for landmark
		foreach (QString sID, set2set.first)
			sg()->getNode(sID)->vis_property["color"] = Qt::red;

		foreach (QString tID, set2set.second)
			tg()->getNode(tID)->vis_property["color"] = Qt::red;

		tb->updateDrawArea();
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
		PART_LANDMARK vec2vec = gcorr->correspondences[i];
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

	this->ui->spatialWeight->setValue(gcorr->spactialW);
	this->ui->structuralWeight->setValue(gcorr->structuralW);
	this->ui->sizeWeight->setValue(gcorr->sizeW);
	this->ui->orientationWeight->setValue(gcorr->orientationW);

	this->ui->scoreThreshold->setValue(gcorr->scoreThreshold);

	this->ui->sourceID->setMaximum(gcorr->sg->nodes.size()-1);
}

void LandmarksDialog::updateAlignmentTab()
{
	// Clear
	ui->alignList1->clear();
	ui->alignList2->clear();

	// Add items
	for (int i = 0; i < sg()->nodes.size(); i++)
		ui->alignList1->addItem(new QListWidgetItem(sNode(i)->id));
	for (int i = 0; i < tg()->nodes.size(); i++)
		ui->alignList2->addItem(new QListWidgetItem(tNode(i)->id));
}

void LandmarksDialog::updateAll()
{
	//normalize();

	ui->sName->setText("Source: " + gcorr->sgName());
	ui->tName->setText("Target: " + gcorr->tgName());

	updateLandmarksTab();
	updateCorrTab();

	// Alignment tab
	this->ui->graphID->setMaximum(tb->graphs.size()-1);
	updateAlignmentTab();
}

void LandmarksDialog::reload()
{
	gcorr = tb->corresponder();
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

	tb->updateDrawArea();
}

void LandmarksDialog::computeCorrespondences()
{
	gcorr->computeCorrespondences();
	updateCorrTab();
}

void LandmarksDialog::rotateGraph()
{
	double angle = ui->angleAroundZ->value();

	Structure::Graph *g = tb->getGraph(ui->graphID->value());
	if (g)
	{
		// Rotate the target graph
		g->rotate(angle, Vector3(0, 0, 1));

		// Update the scene
		tb->updateDrawArea();
	}
}

void LandmarksDialog::showAABB(int state)
{
	foreach (Structure::Graph *g, tb->graphs)
		g->property["showAABB"] = (state == 2);

	tb->updateDrawArea();
}

void LandmarksDialog::scaleGraph(int slider)
{
	double scaleFactor = slider / 50.0;

	Structure::Graph *g = tb->graphs[ui->graphID->value()];
	g->scale(scaleFactor);

	tb->updateDrawArea();
}

void LandmarksDialog::normalize()
{
	foreach(Structure::Graph *g, tb->graphs)
	{
		g->moveBottomCenterToOrigin();
		g->normalize();
	}

	tb->setSceneBounds();
	tb->updateDrawArea();
}

void LandmarksDialog::addPointLandmark()
{
	gcorr->addPointLandmark();

	updatePointLandmarksList();

	tb->updateDrawArea();
}

void LandmarksDialog::updatePointLandmarksList()
{
	// Clear
	ui->pointLandmarksList->clear();

	// Add items
	foreach (POINT_LANDMARK landmark, gcorr->pointLandmarks)
	{
		 QVector<POINT_ID> sPoints = landmark.first;
		 QVector<POINT_ID> tPoints = landmark.second;

		 QString sIDString, tIDString;
		 foreach( POINT_ID sID, sPoints ) sIDString += "<" + QString::number(sID.first) + ", " + QString::number(sID.second) + ">";
		 foreach( POINT_ID tID, tPoints ) tIDString += "<" + QString::number(tID.first) + ", " + QString::number(tID.second) + ">";
		 ui->pointLandmarksList->addItem(new QListWidgetItem(sIDString + "\t" + tIDString));
	}
}

void LandmarksDialog::visualizeCurrentPointLandmark()
{
	int currRow = ui->pointLandmarksList->currentRow();
	if (currRow < 0 || currRow > gcorr->pointLandmarks.size() - 1)
	{
		qDebug() << "Current row is out of range.";
		return;
	}

	// Clear all selection
	gcorr->sg->clearSelections();
	gcorr->tg->clearSelections();

	// Add current point landmarks
	QColor color = Qt::yellow;
	POINT_LANDMARK landmark = gcorr->pointLandmarks[currRow];

	foreach (POINT_ID sID, landmark.first)
	{
		int nID = sID.first;
		int pID = sID.second;
		gcorr->sg->nodes[nID]->addSelectionWithColor(pID, color);
	}

	foreach (POINT_ID tID, landmark.second)
	{
		int nID = tID.first;
		int pID = tID.second;
		gcorr->tg->nodes[nID]->addSelectionWithColor(pID, color);
	}

	tb->updateDrawArea();
}

void LandmarksDialog::visualizeAllPointLandmarks()
{
	// Clear all selection
	gcorr->sg->clearSelections();
	gcorr->tg->clearSelections();

	// Add all point landmarks
	int num = gcorr->pointLandmarks.size();
	for (int id = 0; id < num; id++)
	{
		double t = (id+1) / (double) num;
		QColor color = qtJetColorMap(t);

		POINT_LANDMARK landmark = gcorr->pointLandmarks[id];

		foreach (POINT_ID sID, landmark.first)
		{
			int nID = sID.first;
			int pID = sID.second;
			gcorr->sg->nodes[nID]->addSelectionWithColor(pID, color);
		}

		foreach (POINT_ID tID, landmark.second)
		{
			int nID = tID.first;
			int pID = tID.second;
			gcorr->tg->nodes[nID]->addSelectionWithColor(pID, color);
		}
	}

	tb->updateDrawArea();
}


void LandmarksDialog::removePointLandmark()
{
	int currRow = ui->pointLandmarksList->currentRow();
	if (currRow >= 0 && currRow < gcorr->pointLandmarks.size())
		gcorr->pointLandmarks.remove(currRow);

	updatePointLandmarksList();

	tb->updateDrawArea();
}

void LandmarksDialog::setSpatialWeight( double w )
{
	gcorr->spactialW = w;
}

void LandmarksDialog::setStructuralWeight( double w )
{
	gcorr->structuralW = w;
}

void LandmarksDialog::setSizeWeight( double w )
{
	gcorr->sizeW = w;
}

void LandmarksDialog::setOrientationWeight( double w )
{
	gcorr->orientationW = w;
}

void LandmarksDialog::computeDisM()
{
	gcorr->computeFinalDistanceMatrix();
}

void LandmarksDialog::setScoreThreshold( double tau )
{
	gcorr->scoreThreshold = tau;
}

void LandmarksDialog::computePartToPartCorrespondences()
{
	gcorr->computePartToPartCorrespondences();
	updateCorrTab();
}

void LandmarksDialog::alignAllNodes()
{
	gcorr->correspondAllNodes();
}

void LandmarksDialog::prepareMatrices()
{
	gcorr->prepareAllMatrices();
}

void LandmarksDialog::basicAlign()
{
	if(ui->alignList1->selectedItems().empty()) return;
	if(ui->alignList2->selectedItems().empty()) return;

	Structure::Node * snode = sg()->getNode(ui->alignList1->selectedItems().front()->text());
	Structure::Node * tnode = tg()->getNode(ui->alignList2->selectedItems().front()->text());

	if(snode->type() != tnode->type()) return;

	if(snode->type() == Structure::CURVE)
	{
		gcorr->correspondTwoCurves((Structure::Curve*)snode,(Structure::Curve*)tnode,gcorr->tg);
	}
	if(snode->type() == Structure::SHEET)
	{
		gcorr->correspondTwoSheets((Structure::Sheet*)snode,(Structure::Sheet*)tnode,gcorr->tg);
	}

	tb->updateDrawArea();
}

void LandmarksDialog::visualizeCorr( int row, int column )
{
	if (row >= 0 && row < gcorr->correspondences.size())
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

		// Get the correspondence
		PART_LANDMARK set2set = gcorr->correspondences[row];

		// Set red for landmark
		foreach (QString sID, set2set.first)
			sg()->getNode(sID)->vis_property["color"] = Qt::red;

		foreach (QString tID, set2set.second)
			tg()->getNode(tID)->vis_property["color"] = Qt::red;

		tb->updateDrawArea();
	}
}
