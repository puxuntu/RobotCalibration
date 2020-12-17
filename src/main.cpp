#include "RobotCalibration.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    RobotCalibration w;
    w.show();
    return a.exec();
}
