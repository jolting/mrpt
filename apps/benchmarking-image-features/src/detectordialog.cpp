#include "detectordialog.h"
#include <QtGui>
#include <QDialog>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <QMessageBox>
#include <string>

#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/gui/CDisplayWindowPlots.h>


using namespace cv;
using namespace std;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt;
using namespace mrpt::gui;

void DetectorDialog::onshow_results_clicked()
{

}
void DetectorDialog::oncloseDialog_clicked()
{
    detector_gui->setVisible(false);
    detector_gui->close();
    //this->close();
    return;
}

DetectorDialog::DetectorDialog(QWidget *detector_gui, QString filePath, int detector_id) :QDialog(detector_gui)
{
    detector_gui = new QWidget;
    detector_gui->setWindowTitle("Detector Evaluation");
    QGridLayout *layout_grid = new QGridLayout;




    QPushButton *showResults = new QPushButton("Show Results");
    connect(showResults, SIGNAL(clicked(bool)), this, SLOT(onshow_results_clicked()) );
    QPushButton *closeDialog = new QPushButton("Close");
    connect(closeDialog, SIGNAL(clicked()), this, SLOT(oncloseDialog_clicked()) );


    cv::Mat img = imread(filePath.toStdString(), IMREAD_COLOR);

    if(img.empty() || detector_id == -1)
    {
        QMessageBox::information(this,"Image/Detector read error","Please specify a valid inputs for the image / detector..");
        return;
    }



    QLabel *label = new QLabel("Following are the Detector evaluation metrics\n"
                                "<b>Repeatability \n"
                               "Dispersion of the Image, \n"
                               "Computatioanl Cost\n"
                               "Number of found points </b>\n");

    layout_grid->addWidget(label,0,1,1,1);
    layout_grid->addWidget(showResults,2,1,1,1);
    layout_grid->addWidget(closeDialog,3,1,1,1);




    detector_gui->setLayout(layout_grid);
    detector_gui->show();

    //cv::imshow("sample image",img);
    //cv::waitKey(0);
}
