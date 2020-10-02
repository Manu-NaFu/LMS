#include "parameters.h"
#include "ui_parameters.h"
#include <iostream>
#include <QMessageBox>


parameters::parameters(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::parameters)
{
    ui->setupUi(this);

    //Default values for the parameters.
    p_.setX(400);
    p_.setY(400);
    p_.setAngle(0);
    x_var_= 2;
    y_var_= 2;
    angle_var_ = 4;
    range_x_ = 20;
    range_y_ = 20;
    range_angle_ = 80;
    width_= 800;
    height_ = 800;
    baudrate_ = 115200;
    port_ = (char*) std::malloc(sizeof(char)*256);
    strcpy(port_,"/dev/ttyUSB0");
    w_ = 0.7;
    levmarq_ = false;
    ignore_human_ = true;

    //Initial algorithm: Brute force. Levenberg Marquardt not visible.
    ui->algorithm->setCurrentIndex(0);
    ui->l_levmarq->setVisible(false);
   /* ui->x_lev->setVisible(false);
    ui->y_lev->setVisible(false);
    ui->a_lev->setVisible(false);
    ui->text_a_var->setVisible(false);
    ui->text_x_var->setVisible(false);
    ui->text_y_var->setVisible(false);*/

    ui->text_x->setText("");
    ui->text_y->setText("");
    ui->text_x_var->setText("");
    ui->text_y_var->setText("");
    ui->text_wi->setText("");
    ui->text_h->setText("");
    ui->text_baud->setText("");
    ui->text_port->setText("");
    ui->text_we->setText("");
    ui->text_a_var->setText("");
    ui->range_Y->setText("");
    ui->range_x->setText("");
    ui->range_angle->setText("");


}

parameters::~parameters()
{
    delete ui;
}


void parameters::on_b_accept_clicked()
{
    //If the users accepts, the parameters are set according to some requirements.
    //If one or many of those parameters are not in the allowed range, it will show a warning message to the user to change them.
    bool error = false;
    if (ui->text_x_var->text().length() > 0)
    {
        if (ui->text_x_var->text().toInt() > 0)
            x_var_= ui->text_x_var->text().toInt();
        else
            error = true;
    }

    if (ui->text_y_var->text().length() > 0)
    {
        if (ui->text_y_var->text().toInt() > 0)
            y_var_= ui->text_y_var->text().toInt();
        else
            error = true;
    }

    if (ui->text_wi->text().length() > 0)
    {
        if (ui->text_wi->text().toInt() > 0)
            width_= ui->text_wi->text().toInt();
        else
            error = true;
    }

    if (ui->text_h->text().length() > 0)
    {
        if (ui->text_h->text().toInt())
            height_ = ui->text_h->text().toInt();
        else
            error = true;
    }

    if (ui->text_x->text().length() > 0)
    {
        if (ui->text_x->text().toInt() > 0 && ui->text_x->text().toInt() < width_)
            p_.setX(ui->text_x->text().toInt());
        else
            error = true;
    }
    else
    {
        p_.setX(width_/2);
    }

    if (ui->text_y->text().length() > 0)
    {
        if (ui->text_y->text().toInt() > 0 && ui->text_y->text().toInt() < height_)
            p_.setY(ui->text_y->text().toInt());
        else
            error = true;
    }
    else
    {
        p_.setY(height_/2);
    }

    if (ui->text_baud->text().length() > 0)
    {
        if (ui->text_baud->text().toInt() > 0)
            baudrate_ = ui->text_baud->text().toInt();
        else
            error = true;
    }

    if (ui->text_port->text().length() > 0)
        strcpy(port_,ui->text_port->text().toStdString().c_str());

    if (ui->text_we->text().length() > 0)
    {
        if (ui->text_we->text().toDouble() >= (0-0.00001) && ui->text_we->text().toDouble() <= (1+0.00001))
            w_ = ui->text_we->text().toDouble();
        else
            error = true;
    }

    if (ui->text_a_var->text().length() > 0)
    {
        if (ui->text_a_var->text().toDouble() > 0)
            angle_var_ = ui->text_a_var->text().toDouble();
        else
            error = true;
    }

    if (ui->range_Y->text().length() > 0)
    {
        if (ui->range_Y->text().toInt() > 0)
            range_y_ = ui->range_Y->text().toInt();
        else
            error = true;
    }

    if (ui->range_x->text().length() > 0)
    {
        if (ui->range_x->text().toInt() > 0)
            range_x_ = ui->range_x->text().toInt();
        else
            error = true;
    }

    if (ui->range_angle->text().length() > 0)
    {
        if (ui->range_angle->text().toInt() > 0)
            range_angle_ = ui->range_angle->text().toInt();
        else
            error = true;
    }

    if (error)
    {
        QMessageBox msg;
        msg.setText("A parameter or many of them have an invalid value. Check the parameters in order to correct them.");
        msg.exec();
    }
    else
        this->close();
}


void parameters::clear()
{
    //Default values for parameters.
    p_.setX(400);
    p_.setY(400);
    p_.setAngle(0);
    x_var_= 3;
    y_var_= 3;
    width_= 800;
    height_ = 800;
    baudrate_ = 115200;
    range_x_ = 20;
    range_angle_ = 60;
    range_y_ = 20;
    ignore_human_ = true;
    strcpy(port_,"/dev/ttyUSB0");
    w_ = 0.7;
    angle_var_ = 4;
    levmarq_ = false;


    ui->text_x->setText("");
    ui->text_y->setText("");
    ui->text_x_var->setText("");
    ui->text_y_var->setText("");
    ui->text_a_var->setText("");
    ui->text_wi->setText("");
    ui->text_h->setText("");
    ui->text_baud->setText("");
    ui->text_port->setText("");
    ui->text_we->setText("");
    ui->algorithm->setCurrentIndex(0);
    ui->ignore_human->setChecked(true);
    ui->ignore_human->setText("Yes");

}


void parameters::on_algorithm_currentIndexChanged(int index)
{
    //When the users chooses another algorithm, the information shown changes in order to
    //let the user change the parameters for the chosen algorithm.
    if (index == 0)
    {
        levmarq_ = false;
        ui->l_levmarq->setVisible(false);
       /* ui->text_x_var->setVisible(false);
        ui->text_y_var->setVisible(false);
        ui->text_a_var->setVisible(false);
        ui->a_lev->setVisible(false);
        ui->x_lev->setVisible(false);
        ui->y_lev->setVisible(false);*/

        ui->l_brute->setVisible(true);
        ui->x_brute->setVisible(true);
        ui->y_brute->setVisible(true);
        ui->angle_brute->setVisible(true);
        ui->range_Y->setVisible(true);
        ui->range_x->setVisible(true);
        ui->range_angle->setVisible(true);

        ui->text_x_var->setText("");
        ui->text_y_var->setText("");
        ui->text_a_var->setText("");
        ui->range_x->setText("");
        ui->range_Y->setText("");
        ui->range_angle->setText("");
        x_var_ = 2;
        y_var_ = 2;
        angle_var_ = 4;
    }
    else
    {
        levmarq_ = true;
        ui->l_levmarq->setVisible(true);
        ui->text_x_var->setVisible(true);
        ui->text_y_var->setVisible(true);
        ui->text_a_var->setVisible(true);
        ui->a_lev->setVisible(true);
        ui->x_lev->setVisible(true);
        ui->y_lev->setVisible(true);

        ui->l_brute->setVisible(false);
        ui->x_brute->setVisible(false);
        ui->y_brute->setVisible(false);
        ui->angle_brute->setVisible(false);
        ui->range_Y->setVisible(false);
        ui->range_x->setVisible(false);
        ui->range_angle->setVisible(false);

        ui->text_x_var->setText("");
        ui->text_y_var->setText("");
        ui->text_a_var->setText("");
        x_var_ = 1;
        y_var_ = 1;
        angle_var_ = 1.5;
    }
}

void parameters::on_b_cancel_clicked()
{
    //If the users cancels, every textBox is set to no text.
    ui->algorithm->setCurrentIndex(0);
    ui->text_x->setText("");
    ui->text_y->setText("");
    ui->text_x_var->setText("");
    ui->text_y_var->setText("");
    ui->text_wi->setText("");
    ui->text_h->setText("");
    ui->text_baud->setText("");
    ui->text_port->setText("");
    ui->text_we->setText("");
    ui->text_a_var->setText("");
    ui->range_Y->setText("");
    ui->range_x->setText("");
    ui->range_angle->setText("");
    ui->ignore_human->setChecked(true);
    ui->ignore_human->setText("Yes");

    this->close();
}

void parameters::on_ignore_human_stateChanged(int arg1)
{
    if (arg1 == 0)
    {
        ui->ignore_human->setText("No");
        ignore_human_ = false;
    }
    else if (arg1 == 2)
    {
        ignore_human_ = true;
        ui->ignore_human->setText("Yes");
    }
}
