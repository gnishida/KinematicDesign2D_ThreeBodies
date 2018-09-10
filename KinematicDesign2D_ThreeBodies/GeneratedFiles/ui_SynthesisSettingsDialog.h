/********************************************************************************
** Form generated from reading UI file 'SynthesisSettingsDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.6.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SYNTHESISSETTINGSDIALOG_H
#define UI_SYNTHESISSETTINGSDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_SynthesisSettingsDialog
{
public:
    QGroupBox *groupBox_3;
    QLabel *label_16;
    QLabel *label_17;
    QLineEdit *lineEditNumParticles;
    QLineEdit *lineEditNumIterations;
    QCheckBox *checkBoxRecordFile;
    QPushButton *pushButtonCancel;
    QLineEdit *lineEditMinTransmissionAngle;
    QCheckBox *checkBoxAvoidBranchDefect;
    QPushButton *pushButtonOK;
    QLabel *label_18;
    QGroupBox *groupBox;
    QLabel *label_6;
    QLineEdit *lineEditStdDevPosition;
    QLabel *label_11;
    QLineEdit *lineEditStdDevOrientation;
    QLineEdit *lineEditNumSamples;
    QLabel *label;

    void setupUi(QDialog *SynthesisSettingsDialog)
    {
        if (SynthesisSettingsDialog->objectName().isEmpty())
            SynthesisSettingsDialog->setObjectName(QStringLiteral("SynthesisSettingsDialog"));
        SynthesisSettingsDialog->resize(331, 311);
        groupBox_3 = new QGroupBox(SynthesisSettingsDialog);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(10, 170, 311, 91));
        label_16 = new QLabel(groupBox_3);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(20, 20, 111, 21));
        label_17 = new QLabel(groupBox_3);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(20, 40, 111, 21));
        lineEditNumParticles = new QLineEdit(groupBox_3);
        lineEditNumParticles->setObjectName(QStringLiteral("lineEditNumParticles"));
        lineEditNumParticles->setGeometry(QRect(200, 20, 101, 20));
        lineEditNumIterations = new QLineEdit(groupBox_3);
        lineEditNumIterations->setObjectName(QStringLiteral("lineEditNumIterations"));
        lineEditNumIterations->setGeometry(QRect(200, 40, 101, 20));
        checkBoxRecordFile = new QCheckBox(groupBox_3);
        checkBoxRecordFile->setObjectName(QStringLiteral("checkBoxRecordFile"));
        checkBoxRecordFile->setGeometry(QRect(20, 60, 191, 21));
        pushButtonCancel = new QPushButton(SynthesisSettingsDialog);
        pushButtonCancel->setObjectName(QStringLiteral("pushButtonCancel"));
        pushButtonCancel->setGeometry(QRect(190, 270, 91, 31));
        lineEditMinTransmissionAngle = new QLineEdit(SynthesisSettingsDialog);
        lineEditMinTransmissionAngle->setObjectName(QStringLiteral("lineEditMinTransmissionAngle"));
        lineEditMinTransmissionAngle->setGeometry(QRect(210, 140, 101, 20));
        checkBoxAvoidBranchDefect = new QCheckBox(SynthesisSettingsDialog);
        checkBoxAvoidBranchDefect->setObjectName(QStringLiteral("checkBoxAvoidBranchDefect"));
        checkBoxAvoidBranchDefect->setGeometry(QRect(20, 120, 281, 17));
        pushButtonOK = new QPushButton(SynthesisSettingsDialog);
        pushButtonOK->setObjectName(QStringLiteral("pushButtonOK"));
        pushButtonOK->setGeometry(QRect(50, 270, 91, 31));
        label_18 = new QLabel(SynthesisSettingsDialog);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(60, 140, 141, 16));
        groupBox = new QGroupBox(SynthesisSettingsDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 40, 311, 71));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(50, 20, 101, 21));
        lineEditStdDevPosition = new QLineEdit(groupBox);
        lineEditStdDevPosition->setObjectName(QStringLiteral("lineEditStdDevPosition"));
        lineEditStdDevPosition->setGeometry(QRect(200, 20, 101, 20));
        label_11 = new QLabel(groupBox);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(50, 40, 121, 21));
        lineEditStdDevOrientation = new QLineEdit(groupBox);
        lineEditStdDevOrientation->setObjectName(QStringLiteral("lineEditStdDevOrientation"));
        lineEditStdDevOrientation->setGeometry(QRect(200, 40, 101, 20));
        lineEditNumSamples = new QLineEdit(SynthesisSettingsDialog);
        lineEditNumSamples->setObjectName(QStringLiteral("lineEditNumSamples"));
        lineEditNumSamples->setGeometry(QRect(210, 10, 101, 20));
        label = new QLabel(SynthesisSettingsDialog);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 10, 61, 21));

        retranslateUi(SynthesisSettingsDialog);

        QMetaObject::connectSlotsByName(SynthesisSettingsDialog);
    } // setupUi

    void retranslateUi(QDialog *SynthesisSettingsDialog)
    {
        SynthesisSettingsDialog->setWindowTitle(QApplication::translate("SynthesisSettingsDialog", "SynthesisSettingsDialog", 0));
        groupBox_3->setTitle(QApplication::translate("SynthesisSettingsDialog", "Particle filter:", 0));
        label_16->setText(QApplication::translate("SynthesisSettingsDialog", "#particles:", 0));
        label_17->setText(QApplication::translate("SynthesisSettingsDialog", "#iterations:", 0));
        checkBoxRecordFile->setText(QApplication::translate("SynthesisSettingsDialog", "Record cost evolution to a file", 0));
        pushButtonCancel->setText(QApplication::translate("SynthesisSettingsDialog", "Cancel", 0));
        checkBoxAvoidBranchDefect->setText(QApplication::translate("SynthesisSettingsDialog", "Avoid branch defect/poor transmission angle defect", 0));
        pushButtonOK->setText(QApplication::translate("SynthesisSettingsDialog", "OK", 0));
        label_18->setText(QApplication::translate("SynthesisSettingsDialog", "Min. transmission angle:", 0));
        groupBox->setTitle(QApplication::translate("SynthesisSettingsDialog", "Intermediate pose tolerance", 0));
        label_6->setText(QApplication::translate("SynthesisSettingsDialog", "Stdev for position:", 0));
        label_11->setText(QApplication::translate("SynthesisSettingsDialog", "Stdev for orientation:", 0));
        label->setText(QApplication::translate("SynthesisSettingsDialog", "# Samples:", 0));
    } // retranslateUi

};

namespace Ui {
    class SynthesisSettingsDialog: public Ui_SynthesisSettingsDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SYNTHESISSETTINGSDIALOG_H
