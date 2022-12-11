#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    w.resize(800,600); w.setWindowTitle("Aplikacja monitorująco-sterująca");
    w.setWindowIcon(QIcon(":/new/prefix1/solar.png"));
    return a.exec();
}
