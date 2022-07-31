#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>

using namespace std;

QByteArray servo_ctrl;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->device = new QSerialPort;

    servo_ctrl.resize(2);
    servo_ctrl[0]=0;
    servo_ctrl[1]=0;


    connect(ui->pushButton,SIGNAL(clicked()), this, SLOT(find_devices()));
    connect(ui->pushButton_2,SIGNAL(clicked()), this, SLOT(connect_to_device()));

    connect(ui->pushButton_4,SIGNAL(pressed()), this, SLOT(left()));
    connect(ui->pushButton_4,SIGNAL(released()), this, SLOT(stop_1()));

    connect(ui->pushButton_5,SIGNAL(pressed()), this, SLOT(right()));
    connect(ui->pushButton_5,SIGNAL(released()), this, SLOT(stop_1()));

    connect(ui->pushButton_6,SIGNAL(pressed()), this, SLOT(up()));
    connect(ui->pushButton_6,SIGNAL(released()), this, SLOT(stop_2()));

    connect(ui->pushButton_7,SIGNAL(pressed()), this, SLOT(down()));
    connect(ui->pushButton_7,SIGNAL(released()), this, SLOT(stop_2()));

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::find_devices()
{
    QList<QSerialPortInfo> devices;
    devices = QSerialPortInfo::availablePorts();
    for(int i = 0; i < devices.count(); i++)
    {
        ui->comboBox->addItem(devices.at(i).portName() + " " + devices.at(i).description());
    }

}

void MainWindow::connect_to_device()
{

    if(ui->comboBox->count() == 0)
    {
        return;
    }

    QString portName = ui->comboBox->currentText().split("\t").first();
    string tmp;
    tmp = portName.toStdString();
    int k = tmp.find(' ',0);
    string tmp1 =tmp.substr(0,k);

    portName = QString::fromStdString(tmp1);
    this->device->setPortName(portName);

    if(!device->isOpen())
    {

        if(device->open(QSerialPort::ReadWrite))
        {
            this->device->setBaudRate(QSerialPort::Baud9600);
            this->device->setDataBits(QSerialPort::Data8);
            this->device->setParity(QSerialPort::NoParity);
            this->device->setStopBits(QSerialPort::OneStop);

            add("Otwarto port szeregowy.");

        }

        else
        {
            add("Otwarcie portu szeregowego się nie powiodło!");
        }
    }
}


void MainWindow::add(QString message)
{
    ui->textEdit->append(message);
}

void MainWindow::left()
{
    add("ruch w lewo");
    servo_ctrl[0]=1;
    this->device->write(servo_ctrl, 2);
}


void MainWindow::right()
{
    add("ruch w prawo");
    servo_ctrl[0]=2;
    this->device->write(servo_ctrl, 2);
}

void MainWindow::stop_1()
{

    add("serwo 1 zatrzymane");
    servo_ctrl[0]=0;
    this->device->write(servo_ctrl, 2);
}


void MainWindow::up()
{
    add("ruch w górę");
    servo_ctrl[1]=1;
    this->device->write(servo_ctrl, 2);
}


void MainWindow::down()
{
    add("ruch w dół");
    servo_ctrl[1]=2;
    this->device->write(servo_ctrl, 2);
}


void MainWindow::stop_2()
{

    add("serwo 2 zatrzymane");
    servo_ctrl[1]=0;
    this->device->write(servo_ctrl, 2);
}

