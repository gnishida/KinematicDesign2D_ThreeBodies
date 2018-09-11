#include "LinkageSynthesisWeightWidget.h"
#include "MainWindow.h"
#include "Canvas.h"

LinkageSynthesisWeightWidget::LinkageSynthesisWeightWidget(MainWindow *parent, const std::vector<double>& weights) : QDockWidget((QWidget*)parent) {
	mainWin = parent;
	ui.setupUi(this);

	ui.horizontalSliderPositionError->setRange(0, 10);
	ui.horizontalSliderPositionError->setValue(weights[0]);
	ui.horizontalSliderLinkageLocation->setRange(0, 10);
	ui.horizontalSliderLinkageLocation->setValue(weights[1]);
	ui.horizontalSliderTrajectory->setRange(0, 10);
	ui.horizontalSliderTrajectory->setValue(weights[2]);
	ui.horizontalSliderSize->setRange(0, 10);
	ui.horizontalSliderSize->setValue(weights[3]);
	ui.lineEditPositionError->setText(QString::number(weights[0]));
	ui.lineEditLinkageLocation->setText(QString::number(weights[1]));
	ui.lineEditTrajectory->setText(QString::number(weights[2]));
	ui.lineEditSize->setText(QString::number(weights[3]));

	connect(ui.horizontalSliderPositionError, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
	connect(ui.horizontalSliderLinkageLocation, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
	connect(ui.horizontalSliderTrajectory, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
	connect(ui.horizontalSliderSize, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
	connect(ui.pushButtonApply, SIGNAL(clicked()), this, SLOT(onApply()));
}

LinkageSynthesisWeightWidget::~LinkageSynthesisWeightWidget() {
}

void LinkageSynthesisWeightWidget::onValueChanged(int) {
	ui.lineEditPositionError->setText(QString::number(ui.horizontalSliderPositionError->value()));
	ui.lineEditLinkageLocation->setText(QString::number(ui.horizontalSliderLinkageLocation->value()));
	ui.lineEditTrajectory->setText(QString::number(ui.horizontalSliderTrajectory->value()));
	ui.lineEditSize->setText(QString::number(ui.horizontalSliderSize->value()));

	mainWin->canvas->weights = {
		ui.lineEditPositionError->text().toDouble(),
		ui.lineEditLinkageLocation->text().toDouble(),
		ui.lineEditTrajectory->text().toDouble(),
		ui.lineEditSize->text().toDouble()
	};
}

void LinkageSynthesisWeightWidget::onApply() {
	mainWin->canvas->updateSolutions(canvas::Canvas::LINKAGE_WATTI);
	mainWin->canvas->setFocus();
}