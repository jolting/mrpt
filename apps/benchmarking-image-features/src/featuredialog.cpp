#include "featuredialog.h"

#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <math.h>

#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindowPlots.h>

using namespace cv;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt;


void FeatureDialog::onshow_results_clicked()
{

}
void FeatureDialog::oncloseDialog_clicked()
{
    feature_gui->setVisible(false);
    feature_gui->close();
    //this->close();
    return;
}

FeatureDialog::FeatureDialog(QWidget *parent, QString  filePath, int detector_id, int descriptor_id, int numFeatures) :QDialog(feature_gui)
{
    feature_gui = new QWidget;
    feature_gui->setWindowTitle("Feature (Detector /Descriptor) Evaluation");
    QGridLayout *layout_grid = new QGridLayout;


    QString details("You have selected the DETECTOR" + QString::number(detector_id) + "and Descriptor "+ QString::number(descriptor_id) +" ,The selected image "+ filePath + " has the following characteristics");


    QLabel *label = new QLabel(details);
    QLabel *label2 = new QLabel;
    label2->setText(filePath);// + "I am a label");

    QPushButton *showResults = new QPushButton("Show Results");
    connect(showResults, SIGNAL(clicked(bool)), this, SLOT(onshow_results_clicked()) );
    QPushButton *closeDialog = new QPushButton("Close");
    connect(closeDialog, SIGNAL(clicked()), this, SLOT(oncloseDialog_clicked()) );


    cv::Mat img = imread(filePath.toStdString(), IMREAD_COLOR);
    if(img.empty() || detector_id == -1 || descriptor_id == -1)
    {
        QMessageBox::information(this,"Image/Detector/Descriptor read error","Please specify a valid inputs for the image / detector..");
        return;
    }


    // Feature Extraction Starts here

    mrpt::vision::CFeatureExtraction fext;
    //string sel_method;
    mrpt::vision::TDescriptorType desc_to_compute;
    int sel_num_feats = 100; // CHANGE THIS to Take as input from user

    // Max # of features
    const size_t  nFeats =  sel_num_feats ==0 ? int(150) : sel_num_feats;


    // Reading the files below
    CImage img1,img2;
    string file1 = filePath.toStdString();
    string file2 = filePath.toStdString();

    if (!file1.empty())
    {
        if (!img1.loadFromFile(file1))
            THROW_EXCEPTION_CUSTOM_MSG1("Error loading file: %s",file1.c_str())
    }
    else
    {
        file1 = "/home/raghavender/Downloads/image1.jpg";
        img1.loadFromFile(file1);
    }

    /*cv::Mat cvImg = cv::cvarrToMat( img1.getAs<IplImage>() );
    cv::imshow("sample pic",cvImg);
    cv::waitKey(1);
    QImage win3("Image1");
*/



    fext.options.featsType = TFeatureType(detector_id);

    // this does not handle descriptors beyond index 5 (ORB) need to implement other descriptors from OpenCV.
    desc_to_compute = TDescriptorType(pow(2,(descriptor_id+1)));


    CFeatureList feats1, feats2;
    CTicTac tictac;

    cout << "Detecting features in image1..."; tictac.Tic();
    fext.detectFeatures(img1,feats1, 0,nFeats );
    cout << tictac.Tac() * 1000 << " ms (" << feats1.size() << " features)\n";



    const size_t N_TIMES = 1;
    //const size_t N_TIMES = 10;

    cout << "Extracting descriptors from image 1..."; tictac.Tic();
    for (size_t timer_loop=0;timer_loop<N_TIMES;timer_loop++)
        fext.computeDescriptors(img1,feats1, desc_to_compute);
    cout << tictac.Tac() * 1000.0 / N_TIMES << " ms" << endl;


    cv::Mat cvImg = cv::cvarrToMat( img1.getAs<IplImage>() );
    //cv::imshow("image", cvImg);
    //cv::waitKey(1);


    CDisplayWindow win1 ;

    win1.setPos(10,10);
    win1.showImageAndPoints(img1,feats1, TColor::blue);
    win1.waitForKey();

    //display the detected images here

    /*CDisplayWindow	win1("Image1"), win2("Image2");

    win1.setPos(10,10);
    win1.showImageAndPoints(img1,feats1, TColor::blue);


    win1.waitForKey();
    */



    /*layout_grid->addWidget(label,0,1,1,1);
    layout_grid->addWidget(label2,1,1,1,1);
    layout_grid->addWidget(showResults,2,1,1,1);
    layout_grid->addWidget(closeDialog,3,1,1,1);




    feature_gui->setLayout(layout_grid);
    feature_gui->show();
*/
    //cv::imshow("sample image",img);
    //cv::waitKey(0);

}




