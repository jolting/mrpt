#ifndef DESCRIPTORDIALOG_H
#define DESCRIPTORDIALOG_H
#include <QObject>
#include <QDialog>
#include <string>

using namespace std;

class DescriptorDialog :public QDialog
{
    Q_OBJECT
public:
    explicit DescriptorDialog(QWidget *parent = 0, QString  filePath="/home", int descriptor_id=-1);
public:
    QWidget *descriptor_gui;
    string file_path;
};

#endif // DESCRIPTORDIALOG_H


