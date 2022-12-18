#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include<fstream>
#include<iostream>
#include <QDateTime>
using namespace std;

int isOpen=0;
int isConnected=0;
FILE *file;

QByteArray servo_ctrl;

float voltage;
float current;
float power;
float illuminance;

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

    connect(ui->pushButton_3,SIGNAL(clicked()), this, SLOT(create_file()));
    connect(ui->pushButton_8,SIGNAL(clicked()), this, SLOT(close_file()));

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::find_devices()
{
    ui->comboBox->clear();
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
            connect(this->device, SIGNAL(readyRead()), this, SLOT(readFromPort()));
            isConnected=1;
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
    if(isConnected==1){
        servo_ctrl[0]=1;
        this->device->write(servo_ctrl, 2);
    }
    else
        add("Brak połączenia z urządzeniem.");
}


void MainWindow::right()
{
    if(isConnected==1){
        servo_ctrl[0]=2;
        this->device->write(servo_ctrl, 2);
    }
    else
        add("Brak połączenia z urządzeniem.");

}

void MainWindow::stop_1()
{
    if(isConnected==1){
    servo_ctrl[0]=0;
    this->device->write(servo_ctrl, 2);
    }

    else
        add("Brak połączenia z urządzeniem.");


}


void MainWindow::up()
{
    if(isConnected==1){
        servo_ctrl[1]=1;
        this->device->write(servo_ctrl, 2);
    }
    else
        add("Brak połączenia z urządzeniem.");

}


void MainWindow::down()
{
    if(isConnected==1){
        servo_ctrl[1]=2;
        this->device->write(servo_ctrl, 2);
    }
    else
        add("Brak połączenia z urządzeniem.");

}


void MainWindow::stop_2()
{
    if(isConnected==1)
    servo_ctrl[1]=0;
    this->device->write(servo_ctrl, 2);
}

void MainWindow::readFromPort()
{
    char voltage_buff[5];
    char current_buff[6];
    char illuminance_buff[8];

    while(this->device->canReadLine()) {

        this->device->readLine(voltage_buff,5);

        float voltage_float;
        sscanf(voltage_buff, "%f", &voltage_float);
        QString voltage_string = QString::number(voltage_float, 'f', 2);

        this->device->readLine(current_buff,6);

        float current_float;
        sscanf(current_buff, "%f", &current_float);
        QString current_string = QString::number(current_float, 'f', 2);

        this->device->readLine(illuminance_buff,8);
        float illuminance;
        sscanf(illuminance_buff, "%f", &illuminance);
        qDebug() << illuminance;
        QString illuminance_string = QString::number(illuminance);


        float power = voltage_float*current_float;
        QString power_string = QString::number(power, 'f', 2);
        ui->label_13->setText(current_string);
        ui->label_11->setText(power_string);
        ui->label_12->setText(voltage_string);
        ui->label_20->setText(illuminance_string);
        if(isOpen==1){
            QDateTime date = QDateTime::currentDateTime();
            QString formattedTime1 = date.toString("dd.MM.yyyy hh:mm:ss");
            string formattedTime = formattedTime1.toStdString();
            fprintf(file, "%s; ", formattedTime.c_str());
            fprintf(file, "%.2f; ", voltage_float);
            fprintf(file, "%.1f; ", current_float);
            fprintf(file, "%d\n", (int)illuminance);


        }

    }
}

void MainWindow::create_file()
{

    string str = ui->lineEdit->text().toStdString() + ".txt";
    if(str == ".txt")
        add("Podaj właściwą nazwę pliku!");
    else
    {
        const char * name = str.c_str();
        file = fopen(name , "a");
        fprintf(file, "Dane są zapisane w formacie: 'data'; 'napięcie'; 'prąd'; 'natężenie światła'.\n");
        isOpen =1;
        add("Rozpoczęto zapis do pliku.");
    }
}

void MainWindow::close_file()
{
    fclose(file);
    isOpen = 0;
    add("Zakończono zapis do pliku.");
}

