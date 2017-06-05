#ifndef FEATUREDIALOG_H
#define FEATUREDIALOG_H

#include <QObject>
#include <QWidget>
#include <QDialog>
#include <string>
#include <QString>
#include <QLabel>

using namespace std;
class FeatureDialog: public QDialog
{
    Q_OBJECT
public:
     FeatureDialog(QWidget *parent = 0, QString  filePath="/home", int detector_id=-1, int descriptor_id=-1, int numFeatures = 150);

public:
    QWidget *feature_gui;
    string file_path;

public slots:
    void onshow_results_clicked();
    void oncloseDialog_clicked();

};

#endif // FEATUREDIALOG_H



