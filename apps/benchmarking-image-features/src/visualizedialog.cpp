#include "visualizedialog.h"

#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <math.h>

#include <QGroupBox>
#include <QImage>

#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindowPlots.h>

#include <mrpt/utils/metaprogramming.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/system/threads.h>


using namespace std;

using namespace cv;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt;


void VisualizeDialog::onNextClicked()
{

}
void VisualizeDialog::oncloseDialog_clicked()
{
    feature_gui->setVisible(false);
    feature_gui->close();
    //this->close();
    return;
}

VisualizeDialog::VisualizeDialog(QWidget *parent, QString  filePath, int detector_id, int descriptor_id) :QDialog(feature_gui)
{
    feature_gui = new QWidget;
    feature_gui->setWindowTitle("Visualize Detector /Descriptor");
    QGridLayout *layout_grid = new QGridLayout;



    QString details("You have selected the DETECTOR" + QString::number(detector_id) + "and Descriptor "+ QString::number(descriptor_id) +" ,The selected image has the following characteristics");


    QLabel *label = new QLabel(details);

    QPushButton *next_button = new QPushButton("Next Descriptor");
    connect(next_button, SIGNAL(clicked(bool)), this, SLOT(onNextClicked()) );
    QPushButton *closeDialog = new QPushButton("Close");
    connect(closeDialog, SIGNAL(clicked()), this, SLOT(oncloseDialog_clicked()) );


    //cv::Mat img = imread(filePath.toStdString(), IMREAD_COLOR);
    if( detector_id == -1 || descriptor_id == -1)
    {
        QMessageBox::information(this,"Image/Detector/Descriptor read error","Please specify a valid inputs for the image / detector..");
        return;
    }



    // START showing the images here

    //Displaying the pair of images here
    QGroupBox *groupBox_images;
    QImage qimage1, qimage2;
    QLabel *image1, *image2;

    int len = 10;
    QImage qimages[len];
    QLabel *images[len];

    // testing displaying images in a grid, use this to display descriptors as a QImage array later
     for(int i=0 ; i<len ; i++)
     {
         images[i] = new QLabel;
         qimages[i].load("/home/raghavender/Downloads/image1.jpg");
         QImage qscaled = qimages[i].scaled(DESCRIPTOR_WIDTH, DESCRIPTOR_HEIGHT, Qt::KeepAspectRatio);
         images[i]->setPixmap(QPixmap::fromImage(qscaled));
     }

    QGroupBox *groupBox_images_tiled = new QGroupBox ("Single Image");
    QVBoxLayout *vbox_images = new QVBoxLayout;
    for(int i=0 ; i< len ; i++)
    {
        vbox_images->addWidget(images[i]);
    }
    groupBox_images_tiled->setLayout(vbox_images);



    groupBox_images = new QGroupBox ("Single Image");
    image1 = new QLabel;
    qimage1.load("/home/raghavender/Downloads/image1.jpg"); // replace this with initial image of select an image by specifying path
    QImage qscaled1 = qimage1.scaled(DESCRIPTOR_WIDTH, DESCRIPTOR_HEIGHT, Qt::KeepAspectRatio);
    image1->setPixmap(QPixmap::fromImage(qscaled1));

    image2 = new QLabel;
    qimage2.load("/home/raghavender/Downloads/image1.jpg"); // replace this with initial image of select an image by specifying path
    QImage qscaled2 = qimage2.scaled(DESCRIPTOR_WIDTH, DESCRIPTOR_HEIGHT, Qt::KeepAspectRatio);
    image2->setPixmap(QPixmap::fromImage(qscaled2));

    QHBoxLayout *hbox_images = new QHBoxLayout;
    hbox_images->addWidget(image1);
    hbox_images->addWidget(image2);
    groupBox_images->setLayout(hbox_images);


    // end showing the images here



    cout << "I reached before the reading features part " << endl;


    // Maybe need to find a SMARTER WAY to load the extracted features and descriptors by a passing them in a constructor or using SIGNALS and SLOTS TO pass the CFeatureList variables
    feats1.loadFromTextFile("/home/raghavender/Features1.txt");
    feats2.loadFromTextFile("/home/raghavender/Features2.txt");

    //MAIN Descriptor description starts here
    CTicTac tictac;


    CImage img1_show;

    img1_show.selectTextFont("6x13");

    cout << "I reached before the for loop " << endl;

    // Show features distances: // feats1.size()
    for (unsigned int i1 = 0; i1< 1 ;i1++)
    {
        tictac.Tic();

        // Display the current descriptor in its window and the best descriptor from the other image:
        switch (descriptor_id) {
            case descAny: // Patch
            case descPolarImages:
            case descLogPolarImages:
            case descSpinImages: {
                CImage auxImg1, auxImg2;
                if (descriptor_id == -1) // descAny
                {
                    auxImg1 = feats1[i1]->patch;

                } else if (descriptor_id == 3) // descPolarImages
                {
                    auxImg1.setFromMatrix(feats1[i1]->descriptors.PolarImg);

                } else if (descriptor_id == 4)  // descLogPolarImages
                {
                    auxImg1.setFromMatrix(feats1[i1]->descriptors.LogPolarImg);

                } else if (descriptor_id == 2) // descSpinImages
                {

                    const size_t nR = feats1[i1]->descriptors.SpinImg_range_rows;
                    const size_t nC =
                            feats1[i1]->descriptors.SpinImg.size() / feats1[i1]->descriptors.SpinImg_range_rows;
                    CMatrixFloat M1(nR, nC);
                    for (size_t r = 0; r < nR; r++)
                        for (size_t c = 0; c < nC; c++)
                            M1(r, c) = feats1[i1]->descriptors.SpinImg[c + r * nC];
                    auxImg1.setFromMatrix(M1);

                }

                while (auxImg1.getWidth() < 100 && auxImg1.getHeight() < 100)
                    auxImg1.scaleImage(auxImg1.getWidth() * 2, auxImg1.getHeight() * 2, IMG_INTERP_NN);


                cv::Mat cvImg1 = cv::cvarrToMat(auxImg1.getAs<IplImage>());


                cv::Mat temp1 (cvImg1.cols,cvImg1.rows,cvImg1.type());
                cvtColor(cvImg1, temp1, CV_BGR2RGB);
                QImage dest1 = QImage((uchar*) temp1.data, temp1.cols, temp1.rows, temp1.step, QImage::Format_RGB888);
                QImage qscaled1 = dest1.scaled(DESCRIPTOR_WIDTH, DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                image1->setPixmap(QPixmap::fromImage(qscaled1));
            }
                break;
            case descSIFT: {
                vector<float> v1, v2;
                mrpt::utils::metaprogramming::copy_container_typecasting(feats1[i1]->descriptors.SIFT, v1);

                //winptrPlot_descr1->plot( v1 );
                //winptrPlot_descr2->plot( v2 );

            }
                break;
            case descSURF: {
                //winptrPlot_descr1->plot( feats1[i1]->descriptors.SURF );
                //winptrPlot_descr1->axis_fit();
            }
                break;
            default: {
                cerr << "Descriptor specified is not handled yet" << endl;
            }
                break;
        }
    }

        /// MAIN Descriptor description ends here


    layout_grid->addWidget(label,0,0,1,1);
    layout_grid->addWidget(closeDialog,1,0,1,1);
    layout_grid->addWidget(next_button,1,1,1,1);
    layout_grid->addWidget(groupBox_images,2,0,2,2);
    layout_grid->addWidget(groupBox_images_tiled,3,0,1,1);

    feature_gui->setLayout(layout_grid);
    feature_gui->show();

}
