#ifndef LINKAGESYNTHESISWEIGHTWIDGET_H
#define LINKAGESYNTHESISWEIGHTWIDGET_H

#include <QDockWidget>
#include "ui_LinkageSynthesisWeightWidget.h"

class MainWindow;

class LinkageSynthesisWeightWidget : public QDockWidget {
	Q_OBJECT

private:
	MainWindow* mainWin;
	Ui::LinkageSynthesisWeightWidget ui;

public:
	LinkageSynthesisWeightWidget(MainWindow *parent, const std::vector<double>& weights);
	~LinkageSynthesisWeightWidget();

public slots:
	void onValueChanged(int);
	void onApply();
};

#endif // LINKAGESYNTHESISWEIGHTWIDGET_H
