#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QMainWindow>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void add(QString messsage);

public slots:
    void find_devices();
    void connect_to_device();
    void left();
    void right();
    void up();
    void down();

    void stop_1();
    void stop_2();

private:
    Ui::MainWindow *ui;
    QSerialPort *device;

};
#endif // MAINWINDOW_H
