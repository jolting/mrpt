#include "descriptordialog.h"
#include <QDialog>
#include <QLabel>
#include <QGridLayout>

DescriptorDialog::DescriptorDialog(QWidget *descriptor_gui, QString filePath, int descriptor_id):QDialog(descriptor_gui)
{
    descriptor_gui = new QWidget;
    descriptor_gui->setWindowTitle("Descriptor Evaluation");
    QGridLayout *layout_grid = new QGridLayout;

    QLabel *label = new QLabel("Following are the Descriptor Evaluation metrics:"
                               "% of mathces in the given image pair\n"
                               "distance between close matches\n"
                               "computational cost\n");

    //PROVIDE THE USER WITH DESCRIPTOR SPECIFIC PARAMETER OPTIONS HERE LIKE activate/deactivate robust stereo matching,
    //forward and backward step by step play of features and images in case of an image sequence, etc..

    layout_grid->addWidget(label,1,1,1,1);

    descriptor_gui->setLayout(layout_grid);
    descriptor_gui->show();

}




