#ifndef VISUALIZEDIALOG_H
#define VISUALIZEDIALOG_H

#define DESCRIPTOR_WIDTH 100
#define DESCRIPTOR_HEIGHT 100


#include <QObject>
#include <QWidget>
#include <QDialog>
#include <string>
#include <QString>
#include <QLabel>


#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"


#include <mrpt/vision/CFeatureExtraction.h>

using namespace cv;
using namespace std;
using namespace mrpt::vision;
using namespace mrpt::utils;

using namespace std;
class VisualizeDialog: public QDialog
{
Q_OBJECT
public:
    VisualizeDialog(QWidget *parent = 0, QString  filePath="/home", int detector_id=-1, int descriptor_id=-1);

public:
    QWidget *feature_gui;
    CFeatureList feats1, feats2;


public slots:
    void onNextClicked();
    void oncloseDialog_clicked();

};

#endif // VISUALIZEDIALOG_H



