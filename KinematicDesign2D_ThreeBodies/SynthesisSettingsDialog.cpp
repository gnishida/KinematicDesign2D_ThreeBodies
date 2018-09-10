#include "SynthesisSettingsDialog.h"

SynthesisSettingsDialog::SynthesisSettingsDialog(QWidget *parent) : QDialog(parent) {
	ui.setupUi(this);

	ui.lineEditNumSamples->setText("10000");
	ui.lineEditStdDevPosition->setText("0.8");
	ui.lineEditStdDevOrientation->setText("0.04");
	ui.checkBoxAvoidBranchDefect->setChecked(true);
	ui.lineEditMinTransmissionAngle->setText("0.15");
	ui.lineEditNumParticles->setText("100");
	ui.lineEditNumIterations->setText("20");
	ui.checkBoxRecordFile->setChecked(false);

	connect(ui.pushButtonOK, SIGNAL(clicked()), this, SLOT(onOK()));
	connect(ui.pushButtonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));
}

SynthesisSettingsDialog::~SynthesisSettingsDialog() {
}

void SynthesisSettingsDialog::setNumSamples(int num_samples) {
	ui.lineEditNumSamples->setText(QString::number(num_samples));
}

int SynthesisSettingsDialog::getNumSamples() {
	return ui.lineEditNumSamples->text().toInt();
}

void SynthesisSettingsDialog::setStdDevPosition(double stddev_position) {
	ui.lineEditStdDevPosition->setText(QString::number(stddev_position));
}

double SynthesisSettingsDialog::getStdDevPosition() {
	return ui.lineEditStdDevPosition->text().toDouble();
}

void SynthesisSettingsDialog::setStdDevOrientation(double stddev_orientation) {
	ui.lineEditStdDevOrientation->setText(QString::number(stddev_orientation));
}

double SynthesisSettingsDialog::getStdDevOrientation() {
	return ui.lineEditStdDevOrientation->text().toDouble();
}

void SynthesisSettingsDialog::setAvoidBranchDefect(bool avoid_branch_defect) {
	ui.checkBoxAvoidBranchDefect->setChecked(avoid_branch_defect);
}

bool SynthesisSettingsDialog::getAvoidBranchDefect() {
	return ui.checkBoxAvoidBranchDefect->isChecked();
}

void SynthesisSettingsDialog::setMinTransmissionAngle(double min_transmission_angle) {
	ui.lineEditMinTransmissionAngle->setText(QString::number(min_transmission_angle));
}

double SynthesisSettingsDialog::getMinTransmissionAngle() {
	return ui.lineEditMinTransmissionAngle->text().toDouble();
}

void SynthesisSettingsDialog::setNumParticles(int num_particles) {
	ui.lineEditNumParticles->setText(QString::number(num_particles));
}

int SynthesisSettingsDialog::getNumParticles() {
	return ui.lineEditNumParticles->text().toInt();
}

void SynthesisSettingsDialog::setNumIterations(int num_iterations) {
	ui.lineEditNumIterations->setText(QString::number(num_iterations));
}

int SynthesisSettingsDialog::getNumIterations() {
	return ui.lineEditNumIterations->text().toInt();
}

void SynthesisSettingsDialog::setRecordFile(bool record_file) {
	ui.checkBoxRecordFile->setChecked(record_file);
}

bool SynthesisSettingsDialog::getRecordFile() {
	return ui.checkBoxRecordFile->isChecked();
}

void SynthesisSettingsDialog::onOK() {
	accept();
}

void SynthesisSettingsDialog::onCancel() {
	reject();
}

