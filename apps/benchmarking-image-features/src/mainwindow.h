#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#define NUM_DETECTORS 9
#define NUM_DESCRIPTORS 8
#define IMAGE_WIDTH 500
#define IMAGE_HEIGHT 500


#include <QMainWindow>
#include <QObject>
#include <QWidget>
#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QRadioButton>
#include <QLineEdit>
#include <QMessageBox>
#include <QGroupBox>
#include <QListWidget>
#include <QComboBox>
#include <QFileDialog>
#include <QMap>
#include <QtGui>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent=0);
    void ReadInputFormat();

public:



    QWidget *window_gui;
    QLabel *detector_label;

    QLabel *sample_image;
    QImage my_image;

    QPushButton *button_generate;
    QPushButton *button_close;

    QPushButton *prev_button;
    QPushButton *next_button;

    QPushButton *browse_button;
    QPushButton *browse_button2;

    QGridLayout *layout_grid;

    int inputFormat;
    int currentInputIndex;
    int detector_selected;
    int descriptor_selected;

    QGroupBox *groupBox1;
    QGroupBox *groupBox2;
    QGroupBox *groupBox_images;




    QComboBox *inputs;
    QLineEdit *inputFilePath;
    QLineEdit *inputFilePath2;
    string file_path1;
    string file_path2;

    QLineEdit *numFeaturesLineEdit;
    int numFeatures;


    QComboBox *detectors_select;
    QComboBox *descriptors_select;

    QLabel *image1;
    QLabel *image2;
    QImage qimage1;
    QImage qimage2;

    QLabel *param1;
    QLabel *param2;
    QLabel *param3;
    QLabel *param4;
    QLabel *param5;

    QLineEdit *param1_edit;
    QLineEdit *param2_edit;
    QLineEdit *param3_edit;
    QLineEdit *param4_edit;
    QLineEdit *param5_edit;

    QLabel *output1;

    struct HarrisOptions
    {
        float threshold;
        float k;
        float sigma;
        float radius;
        float min_distance;
        bool tile_image;
    }harris_opts;

    struct SIFTOptions
    {
        float edge_threshold;
        float threshold;
    }SIFT_opts;

    struct SURFOptions
    {
        int hessianThreshold;
        int nLayersPerOctave;
        int nOctaves;
        bool rotation_invariant;
    }SURF_opts;

signals:
    //void currentIndexChanged(int index);

public:
    void makeAllParamsVisible(bool flag);

public slots:
    void on_button_generate_clicked();
    void button_close_clicked();
    void on_detector_button_clicked();
    void on_descriptor_button_clicked();
    void on_browse_button_clicked();
    void on_browse_button_clicked2();
    void on_descriptor_choose(int choice);

    void on_detector_choose(int choice);
    void on_file_input_choose(int choice);

};

#endif // MAINWINDOW_H
