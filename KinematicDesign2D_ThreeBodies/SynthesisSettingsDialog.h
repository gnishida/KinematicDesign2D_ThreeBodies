#ifndef SYNTHESISSETTINGSDIALOG_H
#define SYNTHESISSETTINGSDIALOG_H

#include <QDialog>
#include "ui_SynthesisSettingsDialog.h"

class SynthesisSettingsDialog : public QDialog {
	Q_OBJECT

private:
	Ui::SynthesisSettingsDialog ui;

public:
	SynthesisSettingsDialog(QWidget *parent = 0);
	~SynthesisSettingsDialog();

	void setNumSamples(int num_samples);
	int getNumSamples();
	void setStdDevPosition(double stddev_position);
	double getStdDevPosition();
	void setStdDevOrientation(double stddev_orientation);
	double getStdDevOrientation();
	void setAvoidBranchDefect(bool avoid_branch_defect);
	bool getAvoidBranchDefect();
	void setMinTransmissionAngle(double min_transmission_angle);
	double getMinTransmissionAngle();
	void setNumParticles(int num_particles);
	int getNumParticles();
	void setNumIterations(int num_iterations);
	int getNumIterations();
	void setRecordFile(bool record_file);
	bool getRecordFile();

public slots:
	void onOK();
	void onCancel();
};

#endif // SYNTHESISSETTINGSDIALOG_H
