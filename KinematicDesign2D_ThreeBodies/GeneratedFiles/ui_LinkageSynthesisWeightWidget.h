/********************************************************************************
** Form generated from reading UI file 'LinkageSynthesisWeightWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.6.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LINKAGESYNTHESISWEIGHTWIDGET_H
#define UI_LINKAGESYNTHESISWEIGHTWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LinkageSynthesisWeightWidget
{
public:
    QWidget *widget;
    QLabel *label_5;
    QLabel *label_15;
    QLabel *label_3;
    QLabel *label_4;
    QSlider *horizontalSliderPositionError;
    QLineEdit *lineEditLinkageLocation;
    QLineEdit *lineEditTrajectory;
    QLineEdit *lineEditPositionError;
    QLineEdit *lineEditSize;
    QLabel *labelPositionErrorMin;
    QPushButton *pushButtonApply;
    QSlider *horizontalSliderLinkageLocation;
    QSlider *horizontalSliderTrajectory;
    QSlider *horizontalSliderSize;
    QLabel *labelPositionErrorMax;
    QLabel *labelLinkageLocationMin;
    QLabel *labelLinkageLocationMax;
    QLabel *labelTrajectoryMin;
    QLabel *labelSizeMin;
    QLabel *labelTrajectoryMax;
    QLabel *labelSizeMax;

    void setupUi(QDockWidget *LinkageSynthesisWeightWidget)
    {
        if (LinkageSynthesisWeightWidget->objectName().isEmpty())
            LinkageSynthesisWeightWidget->setObjectName(QStringLiteral("LinkageSynthesisWeightWidget"));
        LinkageSynthesisWeightWidget->resize(251, 193);
        LinkageSynthesisWeightWidget->setMinimumSize(QSize(251, 38));
        widget = new QWidget();
        widget->setObjectName(QStringLiteral("widget"));
        label_5 = new QLabel(widget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(10, 100, 61, 21));
        label_15 = new QLabel(widget);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(10, 40, 51, 21));
        label_3 = new QLabel(widget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 10, 51, 21));
        label_4 = new QLabel(widget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(10, 70, 81, 21));
        horizontalSliderPositionError = new QSlider(widget);
        horizontalSliderPositionError->setObjectName(QStringLiteral("horizontalSliderPositionError"));
        horizontalSliderPositionError->setGeometry(QRect(130, 10, 91, 22));
        horizontalSliderPositionError->setOrientation(Qt::Horizontal);
        lineEditLinkageLocation = new QLineEdit(widget);
        lineEditLinkageLocation->setObjectName(QStringLiteral("lineEditLinkageLocation"));
        lineEditLinkageLocation->setGeometry(QRect(80, 40, 21, 20));
        lineEditTrajectory = new QLineEdit(widget);
        lineEditTrajectory->setObjectName(QStringLiteral("lineEditTrajectory"));
        lineEditTrajectory->setGeometry(QRect(80, 70, 21, 20));
        lineEditPositionError = new QLineEdit(widget);
        lineEditPositionError->setObjectName(QStringLiteral("lineEditPositionError"));
        lineEditPositionError->setGeometry(QRect(80, 10, 21, 20));
        lineEditSize = new QLineEdit(widget);
        lineEditSize->setObjectName(QStringLiteral("lineEditSize"));
        lineEditSize->setGeometry(QRect(80, 100, 21, 20));
        labelPositionErrorMin = new QLabel(widget);
        labelPositionErrorMin->setObjectName(QStringLiteral("labelPositionErrorMin"));
        labelPositionErrorMin->setGeometry(QRect(101, 10, 20, 21));
        labelPositionErrorMin->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        pushButtonApply = new QPushButton(widget);
        pushButtonApply->setObjectName(QStringLiteral("pushButtonApply"));
        pushButtonApply->setGeometry(QRect(40, 130, 171, 31));
        horizontalSliderLinkageLocation = new QSlider(widget);
        horizontalSliderLinkageLocation->setObjectName(QStringLiteral("horizontalSliderLinkageLocation"));
        horizontalSliderLinkageLocation->setGeometry(QRect(130, 40, 91, 22));
        horizontalSliderLinkageLocation->setOrientation(Qt::Horizontal);
        horizontalSliderTrajectory = new QSlider(widget);
        horizontalSliderTrajectory->setObjectName(QStringLiteral("horizontalSliderTrajectory"));
        horizontalSliderTrajectory->setGeometry(QRect(130, 70, 91, 22));
        horizontalSliderTrajectory->setOrientation(Qt::Horizontal);
        horizontalSliderSize = new QSlider(widget);
        horizontalSliderSize->setObjectName(QStringLiteral("horizontalSliderSize"));
        horizontalSliderSize->setGeometry(QRect(130, 100, 91, 22));
        horizontalSliderSize->setOrientation(Qt::Horizontal);
        labelPositionErrorMax = new QLabel(widget);
        labelPositionErrorMax->setObjectName(QStringLiteral("labelPositionErrorMax"));
        labelPositionErrorMax->setGeometry(QRect(230, 10, 21, 21));
        labelPositionErrorMax->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        labelLinkageLocationMin = new QLabel(widget);
        labelLinkageLocationMin->setObjectName(QStringLiteral("labelLinkageLocationMin"));
        labelLinkageLocationMin->setGeometry(QRect(101, 40, 20, 21));
        labelLinkageLocationMin->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        labelLinkageLocationMax = new QLabel(widget);
        labelLinkageLocationMax->setObjectName(QStringLiteral("labelLinkageLocationMax"));
        labelLinkageLocationMax->setGeometry(QRect(230, 40, 21, 21));
        labelLinkageLocationMax->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        labelTrajectoryMin = new QLabel(widget);
        labelTrajectoryMin->setObjectName(QStringLiteral("labelTrajectoryMin"));
        labelTrajectoryMin->setGeometry(QRect(101, 70, 20, 21));
        labelTrajectoryMin->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        labelSizeMin = new QLabel(widget);
        labelSizeMin->setObjectName(QStringLiteral("labelSizeMin"));
        labelSizeMin->setGeometry(QRect(101, 100, 20, 21));
        labelSizeMin->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        labelTrajectoryMax = new QLabel(widget);
        labelTrajectoryMax->setObjectName(QStringLiteral("labelTrajectoryMax"));
        labelTrajectoryMax->setGeometry(QRect(230, 70, 21, 21));
        labelTrajectoryMax->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        labelSizeMax = new QLabel(widget);
        labelSizeMax->setObjectName(QStringLiteral("labelSizeMax"));
        labelSizeMax->setGeometry(QRect(230, 100, 21, 21));
        labelSizeMax->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        LinkageSynthesisWeightWidget->setWidget(widget);

        retranslateUi(LinkageSynthesisWeightWidget);

        QMetaObject::connectSlotsByName(LinkageSynthesisWeightWidget);
    } // setupUi

    void retranslateUi(QDockWidget *LinkageSynthesisWeightWidget)
    {
        LinkageSynthesisWeightWidget->setWindowTitle(QApplication::translate("LinkageSynthesisWeightWidget", "Weights", 0));
        label_5->setText(QApplication::translate("LinkageSynthesisWeightWidget", "Size:", 0));
        label_15->setText(QApplication::translate("LinkageSynthesisWeightWidget", "Location:", 0));
        label_3->setText(QApplication::translate("LinkageSynthesisWeightWidget", "Accuracy:", 0));
        label_4->setText(QApplication::translate("LinkageSynthesisWeightWidget", "Smoothness:", 0));
        labelPositionErrorMin->setText(QApplication::translate("LinkageSynthesisWeightWidget", "0", 0));
        pushButtonApply->setText(QApplication::translate("LinkageSynthesisWeightWidget", "Apply", 0));
        labelPositionErrorMax->setText(QApplication::translate("LinkageSynthesisWeightWidget", "10", 0));
        labelLinkageLocationMin->setText(QApplication::translate("LinkageSynthesisWeightWidget", "0", 0));
        labelLinkageLocationMax->setText(QApplication::translate("LinkageSynthesisWeightWidget", "10", 0));
        labelTrajectoryMin->setText(QApplication::translate("LinkageSynthesisWeightWidget", "0", 0));
        labelSizeMin->setText(QApplication::translate("LinkageSynthesisWeightWidget", "0", 0));
        labelTrajectoryMax->setText(QApplication::translate("LinkageSynthesisWeightWidget", "10", 0));
        labelSizeMax->setText(QApplication::translate("LinkageSynthesisWeightWidget", "10", 0));
    } // retranslateUi

};

namespace Ui {
    class LinkageSynthesisWeightWidget: public Ui_LinkageSynthesisWeightWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LINKAGESYNTHESISWEIGHTWIDGET_H
