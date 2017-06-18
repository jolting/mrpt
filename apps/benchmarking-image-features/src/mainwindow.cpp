#include "mainwindow.h"
#include <vector>
#include <string>
#include <string.h>
#include <iostream>

#include <QButtonGroup>
#include <QLineEdit>
#include <QtGui>
#include <QPixmap>

#include "detectordialog.h"
#include "descriptordialog.h"
#include "featuredialog.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>

#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"


#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindowPlots.h>

#include <opencv2/line_descriptor.hpp>
#include "opencv2/core/utility.hpp"

#include "CFeatureExtraction_LSD.hpp"



using namespace cv::line_descriptor;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt;

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

void MainWindow::on_button_generate_clicked()
{
    ReadInputFormat();
    //FeatureDialog feature_dialog (this, inputFilePath->text(),detector_selected, descriptor_selected, numFeatures);
    qimage1.load("/home/raghavender/Downloads/images.jpeg");
    image1->setPixmap(QPixmap::fromImage(my_image));
}

void MainWindow::button_close_clicked()
{
    window_gui->close();
    this->close();
    return;
}
void MainWindow::on_descriptor_choose(int choice)
{

    if(choice == 7)
    {
        computeBSD();
    }
    cout << " Hello world !! " <<endl;
    //int choice = descriptors_select->currentIndex();
    //if (choice == 8)
    {
        param1->setText("Enter the scale below:");
        param2->setText("Enter the number of Octaves here:");
        //param3->setText(choice);
        //param3->setVisible(false);
        param4->setVisible(false);


    }
}

void MainWindow::makeAllParamsVisible(bool flag)
{
    param1->setVisible(flag);
    param2->setVisible(flag);
    param3->setVisible(flag);
    param4->setVisible(flag);
    param5->setVisible(flag);
    param1_edit->setVisible(flag);
    param2_edit->setVisible(flag);
    param3_edit->setVisible(flag);
    param4_edit->setVisible(flag);
    param5_edit->setVisible(flag);
}
void MainWindow::on_detector_choose(int choice)
{

    makeAllParamsVisible(true);
    // for Harris Features
    if (choice ==1) {
        param1->setText("Enter threshold : ");
        param2->setText("Enter sensitivity, k : ");
        param3->setText("Enter smoothing, sigma : ");
        param4->setText("Enter block size, radius : ");
        param5->setText("Enter tile_image (true/false) : ");
        param1_edit->setText("0.005");
        param2_edit->setText("0.04");
        param3_edit->setText("1.5");
        param4_edit->setText("3");
        param5_edit->setText("true");
    }
    //for SIFT Features
    else if (choice == 3)
    {
        param1->setText("Enter Threshold: ");
        param2->setText("Enter Edge Threshold: ");
        param1_edit->setText("0.04");
        param2_edit->setText("10");
        param3->setVisible(false);
        param4->setVisible(false);
        param5->setVisible(false);
        param3_edit->setVisible(false);
        param4_edit->setVisible(false);
        param5_edit->setVisible(false);

    }
    // for SURF Features
    else if(choice == 4)
    {
        param1->setText("Enter hessianThreshold: ");
        param2->setText("Enter nLayersPerOctave: ");
        param3->setText("Enter nOctaves: ");
        param4->setText("Enter if rotation invariant: ");
        param1_edit->setText("600");
        param2_edit->setText("4");
        param3_edit->setText("2");
        param4_edit->setText("true");

        param5->setVisible(false);
        param5_edit->setVisible(false);


    }
    if (choice ==8)
    {
        param1->setText("Enter the scale below");
        param2->setText("Enter the number of Octaves here:");
        param1_edit->setText("2");
        param2_edit->setText("1");
        param3->setVisible(false);
        param4->setVisible(false);
        param3_edit->setVisible(false);
        param4_edit->setVisible(false);
    }
}
void MainWindow::on_file_input_choose(int choice)
{
    if (choice == 0 || choice == 2)
    {
        inputFilePath2->setVisible(false);
        browse_button2->setVisible(false);
        image2->setVisible(false);

    }
    else
    {
        inputFilePath2->setVisible(true);
        browse_button2->setVisible(true);

        image2->setVisible(true);
    }
}



/*
 *
 * this function is called to show the performance of the selected detector on the input selected by the user
 * the performance metric displayed is in terms of repeatability, dispersion of image, computational cost, number of found points, etc.
 * */
void MainWindow::on_detector_button_clicked()
{
    ReadInputFormat();

    // Feature Extraction Starts here
    CFeatureExtraction fext;
    CFeatureList featsHarris1,featsHarris2;
    CFeatureList featsSIFT1, featsSIFT2;
    CFeatureList featsSURF1, featsSURF2;

    CImage img1, img2;

    img1.loadFromFile(file_path1);
    img2.loadFromFile(file_path2);

    if(detector_selected == 0)
    {

    }
    else if(detector_selected == 1)
    {

        harris_opts.threshold = param1_edit->text().toFloat();
        harris_opts.k = param2_edit->text().toFloat();
        harris_opts.sigma = param3_edit->text().toFloat();
        harris_opts.radius = param4_edit->text().toFloat();
        //harris_opts.tile_image =  true;

        string temp_str = param5_edit->text().toStdString();
        bool temp_bool = temp_str.compare("true") ? false : true;
        harris_opts.tile_image = temp_bool;

        cout <<  temp_bool << endl;

        fext.options.featsType = featHarris;

        fext.options.harrisOptions.threshold = harris_opts.threshold;//0.005;
        fext.options.harrisOptions.k =  harris_opts.k;  // default sensitivity
        fext.options.harrisOptions.sigma = harris_opts.sigma;  // default from matlab smoothing filter
        fext.options.harrisOptions.radius = harris_opts.radius;  // default block size
        //fext.options.harrisOptions.min_distance = 100;
        fext.options.harrisOptions.tile_image = harris_opts.tile_image;

        fext.detectFeatures(img1, featsHarris1);
        fext.detectFeatures(img2, featsHarris2);
        cout << "Harris Threshold:  " << harris_opts.threshold << endl;
        cout << "Fext Harris Threshold:  " << fext.options.harrisOptions.threshold << endl;
        cout << "Number of Harris Features: " << featsHarris1.size() << endl;

        cv::Mat cvImg1 = cv::cvarrToMat(img1.getAs<IplImage>());
        cv::Mat cvImg2 = cv::cvarrToMat(img2.getAs<IplImage>());


        // Drawing a circle around corners for image 1
        //C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
        for(int i=0 ; i<featsHarris1.size() ; i++)
        {
            int temp_x = (int) featsHarris1.getFeatureX(i);
            int temp_y = (int) featsHarris1.getFeatureY(i);
            circle(cvImg1, Point(temp_x, temp_y), 5, Scalar(1), 2, 8, 0);
        }
        // Drawing a circle around corners for image 2
        for(int i=0 ; i<featsHarris2.size() ; i++)
        {
            int temp_x = (int) featsHarris2.getFeatureX(i);
            int temp_y = (int) featsHarris2.getFeatureY(i);
            circle(cvImg2, Point(temp_x, temp_y), 5, Scalar(1), 2, 8, 0);
        }

        // converting the cv::Mat to a QImage and changing the resolution of the output images
        cv::Mat temp1 (cvImg1.cols,cvImg1.rows,cvImg1.type());
        cvtColor(cvImg1, temp1, CV_BGR2RGB);
        QImage dest1 = QImage((uchar*) temp1.data, temp1.cols, temp1.rows, temp1.step, QImage::Format_RGB888);
        QImage qscaled1 = dest1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        image1->setPixmap(QPixmap::fromImage(qscaled1));

        cv::Mat temp2(cvImg2.cols, cvImg2.rows, cvImg2.type());
        cvtColor(cvImg2, temp2, CV_BGR2RGB);
        QImage dest2 = QImage((uchar*) temp2.data, temp2.cols, temp2.rows, temp2.step, QImage::Format_RGB888);
        QImage qscaled2 = dest2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        image2->setPixmap(QPixmap::fromImage(qscaled2));

    }
    else if(detector_selected == 3)
    {
        fext.options.featsType = featSIFT;
        SIFT_opts.threshold = param1_edit->text().toFloat();
        SIFT_opts.edge_threshold = param2_edit->text().toFloat();


        fext.options.SIFTOptions.threshold = SIFT_opts.threshold;
        fext.options.SIFTOptions.edgeThreshold = SIFT_opts.edge_threshold;

        cout << "detecting SIFT Features " << endl ;

        fext.detectFeatures(img1, featsSIFT1);
        fext.detectFeatures(img2, featsSIFT2);
        //featsSIFT1.saveToTextFile("BCDFeatures.txt");

        cout << "Number of Harris Features: " << featsSIFT1.size() << endl;

        cv::Mat cvImg1 = cv::cvarrToMat(img1.getAs<IplImage>());
        cv::Mat cvImg2 = cv::cvarrToMat(img2.getAs<IplImage>());


        // Drawing a circle around corners for image 1
        //C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
        for(int i=0 ; i<featsSIFT1.size() ; i++)
        {
            int temp_x = (int) featsSIFT1.getFeatureX(i);
            int temp_y = (int) featsSIFT1.getFeatureY(i);
            circle(cvImg1, Point(temp_x, temp_y), 5, Scalar(1), 2, 8, 0);
        }
        // Drawing a circle around corners for image 2
        for(int i=0 ; i<featsSIFT2.size() ; i++)
        {
            int temp_x = (int) featsSIFT2.getFeatureX(i);
            int temp_y = (int) featsSIFT2.getFeatureY(i);
            circle(cvImg2, Point(temp_x, temp_y), 5, Scalar(1), 2, 8, 0);
        }

        // converting the cv::Mat to a QImage and changing the resolution of the output images
        cv::Mat temp1 (cvImg1.cols,cvImg1.rows,cvImg1.type());
        cvtColor(cvImg1, temp1, CV_BGR2RGB);
        QImage dest1 = QImage((uchar*) temp1.data, temp1.cols, temp1.rows, temp1.step, QImage::Format_RGB888);
        QImage qscaled1 = dest1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        image1->setPixmap(QPixmap::fromImage(qscaled1));

        cv::Mat temp2(cvImg2.cols, cvImg2.rows, cvImg2.type());
        cvtColor(cvImg2, temp2, CV_BGR2RGB);
        QImage dest2 = QImage((uchar*) temp2.data, temp2.cols, temp2.rows, temp2.step, QImage::Format_RGB888);
        QImage qscaled2 = dest2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        image2->setPixmap(QPixmap::fromImage(qscaled2));

    }
    else if (detector_selected == 4)
    {
        fext.options.featsType = featSURF;
        SURF_opts.hessianThreshold = param1_edit->text().toInt();
        SURF_opts.nLayersPerOctave = param2_edit->text().toInt();
        SURF_opts.nOctaves = param3_edit->text().toInt();
        string temp_str = param4_edit->text().toStdString();
        bool temp_bool = temp_str.compare("true") ? false : true;
        SURF_opts.rotation_invariant = temp_bool;
        cout <<  temp_bool << endl;

        fext.options.SURFOptions.hessianThreshold = SURF_opts.hessianThreshold;
        fext.options.SURFOptions.nLayersPerOctave = SURF_opts.nLayersPerOctave;
        fext.options.SURFOptions.nOctaves = SURF_opts.nOctaves;
        fext.options.SURFOptions.rotation_invariant = SURF_opts.rotation_invariant;

        cout << "detecting SURF Features " << endl ;

        fext.detectFeatures(img1, featsSURF1);
        fext.detectFeatures(img2, featsSURF2);

        cout << "Number of SURF Features: " << featsSURF1.size() << endl;

        cv::Mat cvImg1 = cv::cvarrToMat(img1.getAs<IplImage>());
        cv::Mat cvImg2 = cv::cvarrToMat(img2.getAs<IplImage>());


        // Drawing a circle around corners for image 1
        //C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
        for(int i=0 ; i<featsSURF1.size() ; i++)
        {
            int temp_x = (int) featsSURF1.getFeatureX(i);
            int temp_y = (int) featsSURF1.getFeatureY(i);
            circle(cvImg1, Point(temp_x, temp_y), 5, Scalar(1), 2, 8, 0);
        }
        // Drawing a circle around corners for image 2
        for(int i=0 ; i<featsSURF2.size() ; i++)
        {
            int temp_x = (int) featsSURF2.getFeatureX(i);
            int temp_y = (int) featsSURF2.getFeatureY(i);
            circle(cvImg2, Point(temp_x, temp_y), 5, Scalar(1), 2, 8, 0);
        }

        // converting the cv::Mat to a QImage and changing the resolution of the output images
        cv::Mat temp1 (cvImg1.cols,cvImg1.rows,cvImg1.type());
        cvtColor(cvImg1, temp1, CV_BGR2RGB);
        QImage dest1 = QImage((uchar*) temp1.data, temp1.cols, temp1.rows, temp1.step, QImage::Format_RGB888);
        QImage qscaled1 = dest1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        image1->setPixmap(QPixmap::fromImage(qscaled1));

        cv::Mat temp2(cvImg2.cols, cvImg2.rows, cvImg2.type());
        cvtColor(cvImg2, temp2, CV_BGR2RGB);
        QImage dest2 = QImage((uchar*) temp2.data, temp2.cols, temp2.rows, temp2.step, QImage::Format_RGB888);
        QImage qscaled2 = dest2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        image2->setPixmap(QPixmap::fromImage(qscaled2));
    }
    if(detector_selected == 7)
    {

    }

    if(detector_selected == 8)
    {
        cv::Mat img = imread(file_path1);
        cv::Mat img2 = imread(file_path1);


        int scale = param1_edit->text().toInt();
        int numOctaves = param2_edit->text().toInt();

        cv::Mat ff = computeLSD(img,scale,numOctaves);
        cv::Mat ff2 = computeLSD(img2,scale,numOctaves);




        qimage1 = QImage((uchar*) ff.data, ff.cols, ff.rows, ff.step, QImage::Format_RGB888);
        //image1->setPixmap(QPixmap::fromImage(qimage1));

        QImage qsmall1 = qimage1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        image1->setPixmap(QPixmap::fromImage(qsmall1));


        qimage2 = QImage((uchar*) ff2.data, ff2.cols, ff2.rows, ff2.step, QImage::Format_RGB888);
        QImage qsmall2 = qimage2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        //image2->setPixmap(QPixmap::fromImage(qimage2));

        image2->setPixmap(QPixmap::fromImage(qsmall2));


    }
    if(detector_selected == 6)
    {
        computeBSD();
    }
    if(detector_selected == 7)
    {
        cv::Mat imageMat = imread("/home/raghavender/Downloads/robot2.png");
        /* create a random binary mask */
              cv::Mat mask = Mat::ones( imageMat.size(), CV_8UC1 );

              /* create a pointer to a BinaryDescriptor object with deafult parameters */
              Ptr<LSDDetector> bd = LSDDetector::createLSDDetector();

              /* create a structure to store extracted lines */
              vector<KeyLine> lines;

              /* extract lines */
              cv::Mat output = imageMat.clone();
              bd->detect( imageMat, lines, 2, 1, mask );

              /* draw lines extracted from octave 0 */
              if( output.channels() == 1 )
                cvtColor( output, output, COLOR_GRAY2BGR );
             for ( size_t i = 0; i < lines.size(); i++ )
             {
               KeyLine kl = lines[i];
               if( kl.octave == 0)
               {
                 /* get a random color */
                 int R = ( rand() % (int) ( 255 + 1 ) );
                 int G = ( rand() % (int) ( 255 + 1 ) );
                 int B = ( rand() % (int) ( 255 + 1 ) );

                 /* get extremes of line */
                 Point pt1 = Point2f( kl.startPointX, kl.startPointY );
                 Point pt2 = Point2f( kl.endPointX, kl.endPointY );

                 /* draw line */
                 line( output, pt1, pt2, Scalar( B, G, R ), 3 );
               }

             }

             /* show lines on image */
             imshow( "LSD lines", output );
             waitKey();

    }
    //DetectorDialog detect_dialog(this, inputFilePath->text(), detector_selected);
}

/*
 *
 * this function is called to show the performance of the selected descriptor on the input selected by the user
 * the performance metric displayed is in terms of percentage of patches matched, descriptor distance between close matches, false positives/negatives, computational cost, etc.
 * */
void MainWindow::on_descriptor_button_clicked()
{
    ReadInputFormat();
    //DescriptorDialog descriptor_dialog(this, inputFilePath->text(), descriptor_selected);
}

/*
 *
 * this function browses for the image files that the user can select or the directory that the user can select
 * */
void MainWindow::on_browse_button_clicked()
{
    ReadInputFormat();

    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::AnyFile);

    //0 = single image; 1 = stereo image; 2 = rawlog file ; 3 = image dataset folder
    if(currentInputIndex == 0 || currentInputIndex == 1)
    {
        dialog.setNameFilter(tr("Images (*.png *.xpm *.jpg *.tiff *.gif)"));
    }
    else if(currentInputIndex == 2 || currentInputIndex == 3)
    {
        dialog.setFileMode(QFileDialog::Directory);
    }

    //dialog.setNameFilter(tr("Images (*.png *.xpm *.jpg)"));
    dialog.setViewMode(QFileDialog::Detail);
    QStringList fileNames;
    if(dialog.exec())
        fileNames = dialog.selectedFiles();
    inputFilePath->setText(fileNames.at(0));



}
void MainWindow::on_browse_button_clicked2()
{
    ReadInputFormat();

    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::AnyFile);

    //0 = single image; 1 = stereo image; 2 = rawlog file ; 3 = image dataset folder
    if(currentInputIndex == 0 || currentInputIndex == 1)
    {
        dialog.setNameFilter(tr("Images (*.png *.xpm *.jpg *.tiff *.gif)"));
    }
    else if(currentInputIndex == 2 || currentInputIndex == 3)
    {
        dialog.setFileMode(QFileDialog::Directory);
    }

    //dialog.setNameFilter(tr("Images (*.png *.xpm *.jpg)"));
    dialog.setViewMode(QFileDialog::Detail);
    QStringList fileNames;
    if(dialog.exec())
        fileNames = dialog.selectedFiles();
    inputFilePath2->setText(fileNames.at(0));



}

/*
 * This function reads and stores the states of the user selections and can be used by other functions when required
 *
 **/
void MainWindow::ReadInputFormat()
{
    // store the input type here
    currentInputIndex = inputs->currentIndex();

    //store the detector chosen here
    detector_selected = detectors_select->currentIndex();

    //store the descriptor here
    descriptor_selected = descriptors_select->currentIndex();

    //numFeatures = numFeaturesLineEdit->text().toInt();

    file_path1 = inputFilePath->text().toStdString();
    file_path2 = inputFilePath2->text().toStdString();
}

MainWindow::MainWindow(QWidget *window_gui) : QMainWindow(window_gui)
{
    inputFormat = 0;
    currentInputIndex = 0;
    detector_selected = -1;
    descriptor_selected = -1;

    window_gui = new QWidget;
    window_gui->setWindowTitle("GUI app for benchmarking image detectors and descriptord");

    //Initialize the detectors here
    groupBox1 = new QGroupBox(tr("Select your detector"));
    string detector_names[] = {"KLT Detector", "Harris Corner Detector",
                     "BCD (Binary Corner Detector)", "SIFT",
                     "SURF", "FAST Detector",
                     "FASTER Detector", "AKAZE Detector",
                     "LSD Detector"};
    detectors_select = new QComboBox;

    for(int i=0 ; i<NUM_DETECTORS ; i++)
        detectors_select->addItem(detector_names[i].c_str());
    connect(detectors_select, SIGNAL(currentIndexChanged(int)),this,SLOT(on_detector_choose(int)) );


    QPushButton *detector_button = new QPushButton;
    detector_button->setText("Evaluate Detector");
    detector_button->setFixedSize(150,40);
    connect(detector_button, SIGNAL(clicked(bool)),this, SLOT(on_detector_button_clicked()));
    QVBoxLayout *vbox = new QVBoxLayout;

    vbox->addWidget(detectors_select);
    vbox->addWidget(detector_button);
    groupBox1->setLayout(vbox);



    //Initialize the descriptors here
    groupBox2 = new QGroupBox(tr("Select your descriptor"));
    string descriptor_names[] = {"SIFT Descriptor", "SURF Descriptor",
                                 "Intensity-domain spin image descriptor", "Polar Images descriptor",
                                 "Log-polar image descriptor", "ORB Descriptors",
                                 "LATCH Descriptor", "BLD Descriptor"};
                                  //"BRIEF Descriptors"};

    descriptors_select = new QComboBox;
    for(int i=0 ; i<NUM_DESCRIPTORS ; i++)
        descriptors_select->addItem(descriptor_names[i].c_str());

    connect(descriptors_select, SIGNAL(currentIndexChanged(int)),this,SLOT(on_descriptor_choose(int)) );


    QPushButton *descriptor_button = new QPushButton;
    descriptor_button->setText("Evaluate Descriptor");
    descriptor_button->setFixedSize(150,40);
    connect(descriptor_button, SIGNAL(clicked(bool)),this, SLOT(on_descriptor_button_clicked()));
    QVBoxLayout *vbox2 = new QVBoxLayout;

    vbox2->addWidget(descriptors_select);
    vbox2->addWidget(descriptor_button);
    groupBox2->setLayout(vbox2);



    //Displaying the pair of images here
    groupBox_images = new QGroupBox ("Stereo Image Pair");
    image1 = new QLabel;
    qimage1.load("/home/raghavender/Downloads/image1.jpg"); // replace this with initial image of select an image by specifying path
    QImage qscaled1 = qimage1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    image1->setPixmap(QPixmap::fromImage(qscaled1));

    image2 = new QLabel;
    qimage2.load("/home/raghavender/Downloads/image1.jpg"); // replace this with initial image of select an image by specifying path
    QImage qscaled2 = qimage2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    image2->setPixmap(QPixmap::fromImage(qscaled2));

    QHBoxLayout *hbox_images = new QHBoxLayout;
    hbox_images->addWidget(image1);
    hbox_images->addWidget(image2);
    groupBox_images->setLayout(hbox_images);


    //provide user input image options
    QGroupBox *inputGroupBox = new QGroupBox;
    QLabel *inputLabel = new QLabel("<b>Specify the input data format</b>");
    inputs = new QComboBox;
    inputs->addItem("Single Image");
    inputs->addItem("Stereo Image");
    inputs->addItem("Image Rawlog file");
    inputs->addItem("Image Dataset");
    inputs->addItem("Stereo Image Dataset");

    connect(inputs, SIGNAL(currentIndexChanged(int)), this, SLOT(on_file_input_choose(int)));

    inputFilePath = new QLineEdit;
    browse_button = new QPushButton("Browse1");
    connect(browse_button, SIGNAL(clicked()), this, SLOT(on_browse_button_clicked()));

    inputFilePath2 = new QLineEdit;
    browse_button2 = new QPushButton("Browse2");
    connect(browse_button2, SIGNAL(clicked()), this, SLOT(on_browse_button_clicked2()));
    //initially have the buttons hidden as single image selected by default
    inputFilePath2->setVisible(false);
    browse_button2->setVisible(false);



    //ask user for the number of feature
    QLabel *numFeaturesLabel = new QLabel("Enter the number of features to be detected");
    numFeaturesLineEdit = new QLineEdit;
    numFeaturesLineEdit->setText("enter a number here");

    QVBoxLayout *inputVbox = new QVBoxLayout;
    inputVbox->addWidget(numFeaturesLabel);
    inputVbox->addWidget(numFeaturesLineEdit);
    inputVbox->addWidget(inputLabel);
    inputVbox->addWidget(inputs);
    inputVbox->addWidget(inputFilePath);
    inputVbox->addWidget(browse_button);

    inputVbox->addWidget(inputFilePath2);
    inputVbox->addWidget(browse_button2);

    inputGroupBox->setLayout(inputVbox);


    QGroupBox *paramsGroupBox = new QGroupBox;

    param1 = new QLabel("Enter Parameter 1 value for the detector: ");
    param1_edit = new QLineEdit;
    param1_edit->setText("enter param value");
    param2 = new QLabel("Enter Parameter 2 value for the detector: ");
    param2_edit = new QLineEdit;
    param2_edit->setText("enter param value");
    param3 = new QLabel("Enter Parameter 3 value for the detector: ");
    param3_edit = new QLineEdit;
    param3_edit->setText("enter param value");
    param4 = new QLabel("Enter Parameter 4 value for the detector: ");
    param4_edit = new QLineEdit;
    param4_edit->setText("enter param value");
    param5 = new QLabel("Enter Parameter 5 value for the detector: ");
    param5_edit = new QLineEdit;
    param5_edit->setText("enter param value");


    output1 = new QLabel("Sample output goes here");
    output1->setVisible(false);

    QVBoxLayout *paramVBox = new QVBoxLayout;
    paramVBox->addWidget(param1);
    paramVBox->addWidget(param1_edit);
    paramVBox->addWidget(param2);
    paramVBox->addWidget(param2_edit);
    paramVBox->addWidget(param3);
    paramVBox->addWidget(param3_edit);
    paramVBox->addWidget(param4);
    paramVBox->addWidget(param4_edit);
    paramVBox->addWidget(param5);
    paramVBox->addWidget(param5_edit);

    paramVBox->addWidget(output1);
    paramsGroupBox->setLayout(paramVBox);

    // initializing the buttons here
    QGroupBox *groupBox_buttons = new QGroupBox;
    button_generate = new QPushButton("Generate Detectors / Descriptors");
    button_generate->setFixedSize(200,40);
    connect(button_generate, SIGNAL(clicked(bool)), this,SLOT(on_button_generate_clicked()));
    button_close = new QPushButton("Close");
    button_close->setFixedSize(200,40);

    connect(button_close,SIGNAL(clicked(bool)),this,SLOT(button_close_clicked()));
    QHBoxLayout *hbox1 = new QHBoxLayout;
    hbox1->addWidget(button_close);
    hbox1->addWidget(button_generate);    
    groupBox_buttons->setLayout(hbox1);


    QGroupBox *groupBox_buttons2 = new QGroupBox;
    QHBoxLayout *hbox2 = new QHBoxLayout;
    next_button = new QPushButton("Next");
    prev_button = new QPushButton("Previous");
    next_button->setFixedSize(100,20);
    prev_button->setFixedSize(100,20);
    hbox2->addWidget(prev_button);
    hbox2->addWidget(next_button);
    groupBox_buttons2->setLayout(hbox2);



    layout_grid = new QGridLayout;
    layout_grid->addWidget(groupBox1,1,0,1,1);
    layout_grid->addWidget(groupBox2,1,1,1,1);
    layout_grid->addWidget(groupBox_images,2,0,2,2);
    layout_grid->addWidget(inputGroupBox,1,2,1,1);
    layout_grid->addWidget(groupBox_buttons2,4,0,1,2);
    layout_grid->addWidget(groupBox_buttons,5,0,2,2);
    layout_grid->addWidget(paramsGroupBox,3,2,1,1);


    window_gui->setLayout(layout_grid);
    window_gui->show();
}
