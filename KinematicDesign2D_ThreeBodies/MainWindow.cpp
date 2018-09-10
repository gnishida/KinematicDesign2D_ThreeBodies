#include "MainWindow.h"
#include <QFileDialog>
#include "SynthesisSettingsDialog.h"
#include <vector>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);

	canvas = new canvas::Canvas(this);
	setCentralWidget(canvas);

	// setup the docking widgets
	weightWidget = new LinkageSynthesisWeightWidget(this, canvas->weights);
	weightWidget->show();
	addDockWidget(Qt::LeftDockWidgetArea, weightWidget);

	QActionGroup* groupMode = new QActionGroup(this);
	groupMode->addAction(ui.actionSelect);
	groupMode->addAction(ui.actionFixedRectangle);
	groupMode->addAction(ui.actionFixedCircle);
	groupMode->addAction(ui.actionFixedPolygon);
	groupMode->addAction(ui.actionMovingRectangle);
	groupMode->addAction(ui.actionMovingCircle);
	groupMode->addAction(ui.actionMovingPolygon);
	groupMode->addAction(ui.actionLinkageRegion);
	groupMode->addAction(ui.actionKinematics);
	ui.actionSelect->setChecked(true);
	
	groupLayer = new QActionGroup(this);
	initLayerMenu(2);

	ui.actionCollisionCheck->setChecked(canvas->collision_check);

	connect(ui.actionNew, SIGNAL(triggered()), this, SLOT(onNew()));
	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(onSave()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionUndo, SIGNAL(triggered()), this, SLOT(onUndo()));
	connect(ui.actionRedo, SIGNAL(triggered()), this, SLOT(onRedo()));
	connect(ui.actionCopy, SIGNAL(triggered()), this, SLOT(onCopy()));
	connect(ui.actionPaste, SIGNAL(triggered()), this, SLOT(onPaste()));
	connect(ui.actionDelete, SIGNAL(triggered()), this, SLOT(onDelete()));
	connect(ui.actionSelectAll, SIGNAL(triggered()), this, SLOT(onSelectAll()));
	connect(ui.actionSelect, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionFixedRectangle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionFixedCircle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionFixedPolygon, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionMovingRectangle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionMovingCircle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionMovingPolygon, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionLinkageRegion, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionKinematics, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionAddLayer, SIGNAL(triggered()), this, SLOT(onAddLayer()));
	connect(ui.actionInsertLayer, SIGNAL(triggered()), this, SLOT(onInsertLayer()));
	connect(ui.actionDeleteLayer, SIGNAL(triggered()), this, SLOT(onDeleteLayer()));
	connect(ui.actionGenerateLinkageWattI, SIGNAL(triggered()), this, SLOT(onGenerateLinkageWattI()));
	connect(ui.actionSynthesisSettings, SIGNAL(triggered()), this, SLOT(onSynthesisSettings()));
	connect(ui.actionWeightsWindow, SIGNAL(triggered()), this, SLOT(onWeightsWindow()));
	connect(ui.actionRun, SIGNAL(triggered()), this, SLOT(onRun()));
	connect(ui.actionRunBackward, SIGNAL(triggered()), this, SLOT(onRunBackward()));
	connect(ui.actionStop, SIGNAL(triggered()), this, SLOT(onStop()));
	connect(ui.actionStepForward, SIGNAL(triggered()), this, SLOT(onStepForward()));
	connect(ui.actionStepBackward, SIGNAL(triggered()), this, SLOT(onStepBackward()));
	connect(ui.actionCollisionCheck, SIGNAL(triggered()), this, SLOT(onCollisionCheck()));
	connect(ui.actionRestrictMotionRange, SIGNAL(triggered()), this, SLOT(onRestrictMotionRange()));
	connect(ui.actionShowSolutions, SIGNAL(triggered()), this, SLOT(onShowSolutions()));
	connect(ui.actionShowGridLines, SIGNAL(triggered()), this, SLOT(onShowGridLines()));
	connect(ui.actionShowInputPoses, SIGNAL(triggered()), this, SLOT(onShowInputPoses()));
	connect(ui.actionShowLinkage, SIGNAL(triggered()), this, SLOT(onShowLinkage()));

	// create tool bar for file menu
	ui.mainToolBar->addAction(ui.actionNew);
	ui.mainToolBar->addAction(ui.actionOpen);
	ui.mainToolBar->addAction(ui.actionSave);
	ui.mainToolBar->addSeparator();

	// create tool bar for edit menu
	ui.mainToolBar->addAction(ui.actionUndo);
	ui.mainToolBar->addAction(ui.actionRedo);
	ui.mainToolBar->addAction(ui.actionCopy);
	ui.mainToolBar->addAction(ui.actionPaste);
	ui.mainToolBar->addAction(ui.actionDelete);
	ui.mainToolBar->addSeparator();

	// create tool bar for modes
	ui.mainToolBar->addAction(ui.actionSelect);
	ui.mainToolBar->addAction(ui.actionFixedRectangle);
	ui.mainToolBar->addAction(ui.actionFixedCircle);
	ui.mainToolBar->addAction(ui.actionFixedPolygon);
	ui.mainToolBar->addAction(ui.actionMovingRectangle);
	ui.mainToolBar->addAction(ui.actionMovingCircle);
	ui.mainToolBar->addAction(ui.actionMovingPolygon);
	ui.mainToolBar->addAction(ui.actionLinkageRegion);
	ui.mainToolBar->addSeparator();

	// create tool bar for linkage generation
	ui.mainToolBar->addAction(ui.actionGenerateLinkageWattI);
	ui.mainToolBar->addSeparator();

	// create tool bar for kinematic simulation
	ui.mainToolBar->addAction(ui.actionStepBackward);
	ui.mainToolBar->addAction(ui.actionRun);
	ui.mainToolBar->addAction(ui.actionStop);
	ui.mainToolBar->addAction(ui.actionStepForward);
}

MainWindow::~MainWindow() {
}

void MainWindow::initLayerMenu(int num_layers) {
	for (int i = 0; i < menuLayers.size(); i++) {
		disconnect(menuLayers[i], SIGNAL(triggered()), this, SLOT(onLayerChanged()));
		ui.menuLayer->removeAction(menuLayers[i]);
		groupLayer->removeAction(menuLayers[i]);
		delete menuLayers[i];
	}
	menuLayers.clear();

	for (int i = 0; i < num_layers; i++) {
		menuLayers.push_back(ui.menuLayer->addAction(QString("Layer %1").arg(i + 1)));
		menuLayers[i]->setCheckable(true);
		groupLayer->addAction(menuLayers[i]);
		connect(menuLayers[i], SIGNAL(triggered()), this, SLOT(onLayerChanged()));
	}
	menuLayers[0]->setChecked(true);
}

void MainWindow::onNew() {
	canvas->clear();
	setWindowTitle("Dynamic Object Design");
}

void MainWindow::onOpen() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open design file..."), "", tr("Design files (*.xml)"));
	if (filename.isEmpty()) return;

	canvas->open(filename);
	setWindowTitle("Dynamic Object Design - " + QFileInfo(filename).fileName());
}

void MainWindow::onSave() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save design file..."), "", tr("Design files (*.xml)"));
	if (filename.isEmpty()) return;

	canvas->save(filename);
	setWindowTitle("Dynamic Object Design - " + QFileInfo(filename).fileName());
}

void MainWindow::onUndo() {
	canvas->undo();
}

void MainWindow::onRedo() {
	canvas->redo();
}

void MainWindow::onCopy() {
	canvas->copySelectedShapes();
}

void MainWindow::onPaste() {
	canvas->pasteCopiedShapes();
}

void MainWindow::onDelete() {
	canvas->deleteSelectedShapes();
}

void MainWindow::onSelectAll() {
	canvas->selectAll();
}

void MainWindow::onModeChanged() {
	if (ui.actionSelect->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_SELECT);
	}
	else if (ui.actionFixedRectangle->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_FIXED_RECTANGLE);
	}
	else if (ui.actionFixedCircle->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_FIXED_CIRCLE);
	}
	else if (ui.actionFixedPolygon->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_FIXED_POLYGON);
	}
	else if (ui.actionMovingRectangle->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_MOVING_RECTANGLE);
	}
	else if (ui.actionMovingCircle->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_MOVING_CIRCLE);
	}
	else if (ui.actionMovingPolygon->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_MOVING_POLYGON);
	}
	else if (ui.actionLinkageRegion->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_LINKAGE_REGION);
	}
	else if (ui.actionKinematics->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_KINEMATICS);
	}
	update();
}

void MainWindow::onAddLayer() {
	menuLayers.push_back(ui.menuLayer->addAction(QString("Layer %1").arg(menuLayers.size() + 1)));
	menuLayers.back()->setCheckable(true);
	groupLayer->addAction(menuLayers.back());
	menuLayers.back()->setChecked(true);
	connect(menuLayers.back(), SIGNAL(triggered()), this, SLOT(onLayerChanged()));

	canvas->addLayer();
}

void MainWindow::onInsertLayer() {
	menuLayers.push_back(ui.menuLayer->addAction(QString("Layer %1").arg(menuLayers.size() + 1)));
	menuLayers.back()->setCheckable(true);
	groupLayer->addAction(menuLayers.back());
	connect(menuLayers.back(), SIGNAL(triggered()), this, SLOT(onLayerChanged()));

	canvas->insertLayer();
}

void MainWindow::onDeleteLayer() {
	disconnect(menuLayers.back(), SIGNAL(triggered()), this, SLOT(onLayerChanged()));
	ui.menuLayer->removeAction(menuLayers.back());
	groupLayer->removeAction(menuLayers.back());
	delete menuLayers.back();
	menuLayers.resize(menuLayers.size() - 1);

	canvas->deleteLayer();
}

void MainWindow::onLayerChanged() {
	for (int i = 0; i < menuLayers.size(); i++) {
		if (menuLayers[i]->isChecked()) {
			canvas->setLayer(i);
			break;
		}
	}
}

void MainWindow::onGenerateLinkageWattI() {
	canvas->calculateSolutions(canvas::Canvas::LINKAGE_WATTI);
}

void MainWindow::onSynthesisSettings() {
	SynthesisSettingsDialog dlg;
	dlg.setNumSamples(canvas->num_samples);
	dlg.setStdDevPosition(canvas->stddev_position);
	dlg.setStdDevOrientation(canvas->stddev_orientation);
	dlg.setAvoidBranchDefect(canvas->avoid_branch_defect);
	dlg.setMinTransmissionAngle(canvas->min_transmission_angle);
	dlg.setNumParticles(canvas->num_particles);
	dlg.setNumIterations(canvas->num_pf_iterations);
	dlg.setRecordFile(canvas->record_pf);
	if (dlg.exec()) {
		canvas->num_samples = dlg.getNumSamples();
		canvas->stddev_position = dlg.getStdDevPosition();
		canvas->stddev_orientation = dlg.getStdDevOrientation();
		canvas->avoid_branch_defect = dlg.getAvoidBranchDefect();
		canvas->min_transmission_angle = dlg.getMinTransmissionAngle();
		canvas->num_particles = dlg.getNumParticles();
		canvas->num_pf_iterations = dlg.getNumIterations();
		canvas->record_pf = dlg.getRecordFile();
	}
}

void MainWindow::onWeightsWindow() {
	weightWidget->show();
}

void MainWindow::onRun() {
	canvas->run();
}

void MainWindow::onRunBackward() {
	canvas->invertSpeed();
	canvas->run();
}

void MainWindow::onStop() {
	canvas->stop();
}

void MainWindow::onStepForward() {
	canvas->stepForward();
}

void MainWindow::onStepBackward() {
	canvas->stepBackward();
}

void MainWindow::onCollisionCheck() {
	canvas->collision_check = ui.actionCollisionCheck->isChecked();
}

void MainWindow::onRestrictMotionRange() {
	canvas->restrict_motion_range = ui.actionRestrictMotionRange->isChecked();
}

void MainWindow::keyPressEvent(QKeyEvent* e) {
	canvas->keyPressEvent(e);
}

void MainWindow::keyReleaseEvent(QKeyEvent* e) {
	canvas->keyReleaseEvent(e);
}

void MainWindow::onShowSolutions() {
	canvas->show_solutions = ui.actionShowSolutions->isChecked();
	update();
}

void MainWindow::onShowGridLines() {
	canvas->show_grid_lines = ui.actionShowGridLines->isChecked();
	update();
}

void MainWindow::onShowInputPoses() {
	canvas->show_input_poses = ui.actionShowInputPoses->isChecked();
	update();
}

void MainWindow::onShowLinkage() {
	canvas->show_linkage = ui.actionShowLinkage->isChecked();
	update();
}
