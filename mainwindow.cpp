#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "ui_parameters.h"
#include "parameters.h"
#include "work.h"
#include <simulation.h>

#include <cstdio>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include "slam.hpp"
#include "levmarq.h"
#include "sdk/include/rplidar.h"

#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include <QRunnable>
#include <QObject>
#include <QFileDialog>
#include <QThread>
#include <QProcess>
#include <QThreadPool>
#include <QMessageBox>
#include <QMouseEvent>
#include <QString>
#include <QDebug>


Q_DECLARE_METATYPE(rplidar_response_measurement_node_hq_t)
Q_DECLARE_METATYPE(rplidar_response_measurement_node_hq_t*)
Q_DECLARE_OPAQUE_POINTER(rplidar_response_measurement_node_hq_t)

Q_DECLARE_METATYPE(cv::Mat)
Q_DECLARE_METATYPE(cv::Mat*)
Q_DECLARE_OPAQUE_POINTER(cv::Mat)

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //ui->label_auto->setVisible(false);
    //this->setStyleSheet("background-color: rgb(30,50,56);");
    connect(ui->b_set_parameters,SIGNAL(clicked()),this,SLOT(btnparameters())); // creating connections to activate the window parameters once the user clicks the button.
//    connect(ui->joyPad, &JoyPad::xChanged, this, &MainWindow::joypad_x_changed);
//    connect(ui->joyPad, &JoyPad::yChanged, this, &MainWindow::joypad_y_changed);

    param = new parameters; //Assigning a new parameters to the class param.

    pool = QThreadPool::globalInstance(); //Creating a new pool for the trheads.

    points = 0; //Initialising the number of points clicked.

    work.setAutoDelete(false); //The thread is not deleted after calling run();
    connect(&work, &Work::print_img,  this, &MainWindow::print_img); //Connecting the slot print_img of the mainwindow from the signal print_img of the work class.
    connect(&work, &Work::error_port, this, &MainWindow::error_port); //Connecting the slot error_port of the mainwindow from the signal error_port of the work class.

    simulation.setAutoDelete(false); //The thread is not deleted after calling run();
    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(&simulation, &Simulation::print_img,         this,     &MainWindow::print_img); //Connecting the slot print_img of the mainwindow from the signal print_img of the simulation class.
    connect(&simulation, &Simulation::sim_finished,      this,     &MainWindow::sim_finished); //Connecting the slot sim_finished of the mainwindow from the signal sim_finished of the simulation class.
    connect(&simulation, &Simulation::path_not_found,    this,     &MainWindow::path_not_found);
    connect(&simulation, &Simulation::wait_for_readings, &sim_env, &Sim_env::create_readings);
    connect(&simulation, &Simulation::move_robot,        &sim_env, &Sim_env::move_robot);

    sim_env.setAutoDelete(false); //The thread is not deleted after calling run();
    qRegisterMetaType<rplidar_response_measurement_node_hq_t*>("rplidar_response_measurement_node_hq_t*");
    connect(&sim_env,    &Sim_env::emit_readings,       &simulation, &Simulation::update_readings);
    connect(ui->joyPad,  &JoyPad::xChanged,             &sim_env,    &Sim_env::update_x_speed);
    connect(ui->joyPad,  &JoyPad::yChanged,             &sim_env,    &Sim_env::update_y_speed);
    connect(&sim_env,    &Sim_env::moved,               &simulation, &Simulation::moved_robot);
    connect(&sim_env,    &Sim_env::finished_navigation, &simulation, &Simulation::finished_navigation);

    setWindowIcon(QIcon(":./pixil-layer-Background.png"));

    work.init(); //Initialising work.
    simulation.init(); //Initialising simulation.
    sim_env.init();
}

MainWindow::~MainWindow()
{
    //Setting running conditions to false, freeing memory and waiting for threads.
    pool->waitForDone();
    work.clear();
    simulation.clear();
    sim_env.clear();
    delete ui;
}

void MainWindow::on_b_new_map_clicked()
{
    //When there is no simulation nor real map running and there is no map alredy created, the user can start a new map.
    if (!simulation.running() && !work.running() && !work.finished() && !work.started())
    {
        //The real map state is set as started, the parameters are set and the thread is launched.
        work.setStarted(true);
        work.set_parameters(param->getPoint(), param->getWidth(), param->getHeight(), param->getXVariation(), param->getYVariation(), param->getAngleVariation(), param->getWeight(), param->getBaudrate(), param->getPort(), param->getLevmarq(), param->getRangeX(), param->getRangeY(), param->getRangeAngle(), param->getIgnoreHuman());
        pool->start(&work);
    }
    else
    {
        //If the state of the software does not mach the conditions, a warning message is shown.
        QMessageBox msg;
        msg.setText("There is a simulation running or a map beeing created. If the map has finished, reset and start maping again");
        msg.exec();
    }
}

void MainWindow::path_not_found()
{
    QMessageBox msg;
    msg.setText("The path is blocked.\n The robot cannot get there.");
    msg.exec();
}

void MainWindow::on_b_save_map_clicked()
{
    if (work.running()) //If a real map is beeing created, then the image of that map can be saved.
    {
        //The state of save is set as true in order to block the thread while the user saves the image.
        work.setSave(true);

        //A file dialog to ask the user for the name of the file is shown and the image is saved.
        QString fileName = QFileDialog::getSaveFileName(this, tr("Save Map"), ".", tr("Image Files (*.png)"));
        cv::imwrite(fileName.toStdString()+".png",work.getImgMap());

        //The state of save is now set to false again and the thread stops running.
        work.setSave(false);
        work.setRunning(false);

        //The real map state is now set to finished.
        work.setFinished(true);
    }
    else if (simulation.started()) //If there is a simulaion started, a warning message is shown.
    {
        QMessageBox msg;
        msg.setText("There is a simulation running. To save a simulation map, click stop or let the simulation finish.");
        msg.exec();
    }
    else if (work.finished()) //If the real map has been created, a warning message is shown.
    {
        QMessageBox msg;
        msg.setText("A map has just finished and it has been saved. Reset and then start a new map");
        msg.exec();
    }
    else //If there is no simulation nor real map beeing created, a warning message is shown.
    {
        QMessageBox msg;
        msg.setText("There is no map beeing created.");
        msg.exec();
    }
}


void MainWindow::on_b_save_sim_clicked()
{
    //The user will be able to save the readings only if a real map has finished, the real map is not running and no simulation has started.
    if (work.finished() && !work.running() && !simulation.started())
    {
        //A file dialog will be shown to choose the name and route of the file to save to.
        QString fileName = QFileDialog::getSaveFileName(this, tr("Save simulation"), ".", tr("Text Files (*.txt)"));
        std::ofstream f_w;
            //The file is opened.
            f_w.open(fileName.toStdString()+".txt");

            if (f_w.is_open())
            {
                //If the file is opened, the readings from the work class are written into the file.
                for (int i = 0; i < (int)work.getCountReadings(); i++)
                {
                    //Angle from where the ray was captured.
                    int write = work.readings_[i].angle_z_q14;
                    f_w << write << "\n";
                    //Distance the ray traveled.
                    write = work.readings_[i].dist_mm_q2;
                    f_w << write << "\n";
                    //Quality of the reading.
                    write = work.readings_[i].quality;
                    f_w << write << "\n";
                    //Number of the ray.
                    write = work.readings_[i].flag;
                    f_w << write << "\n";
                }
                //f_w << "\n";
            }

            //Once the file is written, it is closed.
            f_w.close();
    }
    else if(simulation.running()) //If there is a simulation running, a warning message is shown
    {
        QMessageBox msg;
        msg.setText("You cannot save data from simulation, you should already have the file.");
        msg.exec();
    }
    else //If none of the conditions are met, it means there is a real map beeing created and the user needs to save it to proceed.
    {
        QMessageBox msg;
        msg.setText("Save the map and then save the data for simulation.");
        msg.exec();
    }


}


void MainWindow::on_b_restart_clicked()
{
    //If a simulation has started and is not running, it resets the program.
    if (simulation.started() && !simulation.running())
    {
        //The elements of the UI are reset to their initial state, as well as the param class and the simulation class.
        ui->sim_mode_label->setText("Simulation mode: live");
        ui->map->setText("MAP");
        ui->simu_finished->setText("");
        ui->text_spd->setText("100");
        ui->l_point1->setStyleSheet("");
        ui->t_point1->setStyleSheet("");
        ui->l_point1->setText("");
        ui->t_point1->setText("");
        ui->l_point2->setStyleSheet("");
        ui->t_point2->setStyleSheet("");
        ui->l_point2->setText("");
        ui->t_point2->setText("");
        ui->lab_distance->setStyleSheet("");
        ui->lab_distance->setText("");
        simulation.clear();
        sim_env.clear();
        param->clear();
    }
    //If there is a simulation started and running, a warning message is shown.
    else if (simulation.started() && simulation.running())
    {
        QMessageBox msg;
        msg.setText("You cannot reset when a simulation is running.\nStop the simulation and try again");
        msg.exec();
    }
    //If no simulation or map has started or finished, there is nothing to reset.
    else if (!simulation.started() && !work.finished() && !work.running())
    {
        QMessageBox msg;
        msg.setText("There is nothing to reset.");
        msg.exec();
    }
    else if(work.running()) //If there is a real map running, it does not reset.
    {
        QMessageBox msg;
        msg.setText("You cannot reset when a map is beeing created.\nSave the map and try again");
        msg.exec();
    }
    //If the simulation has not started and a real map is finished, it resets the program.
    else if(!simulation.running() && work.finished())
    {
        //The elementos of the UI are reset to their initial state, as well as the param class and the work class.
        ui->map->setText("MAP");
        ui->simu_finished->setText("");
        ui->text_spd->setText("100");
        ui->l_point1->setStyleSheet("");
        ui->t_point1->setStyleSheet("");
        ui->l_point1->setText("");
        ui->t_point1->setText("");
        ui->l_point2->setStyleSheet("");
        ui->t_point2->setStyleSheet("");
        ui->l_point2->setText("");
        ui->t_point2->setText("");
        ui->lab_distance->setStyleSheet("");
        ui->lab_distance->setText("");
        work.clear();
        param->clear();
    }

}




void MainWindow::on_b_load_clicked()
{
    //If there is no real map beeing created or finished and no simulation finished or started
     if (!work.running() && !work.finished() && !simulation.running() && !simulation.finished() && !simulation.started())
     {
         //A file dialog is opened to ask the user for the path and file name to load.
        QString fileName = QFileDialog::getOpenFileName(this, tr("Open simulation"), ".", tr("Text Files (*.txt)"));

         std::ifstream f_r;
         std::string line;

         f_r.open(fileName.toStdString());

         //If the file is opened, the readings are saved into a buffer and the number of readings is updated.
         if (f_r.is_open())
         {
             simulation.readings_ = (rplidar_response_measurement_node_hq_t*) std::malloc(sizeof(rplidar_response_measurement_node_hq_t)*200000);
             count = 0;
             while(getline(f_r, line))
             {
                 if (count < 200000)
                 {
                     simulation.readings_[count].angle_z_q14 = stoi(line);
                     getline(f_r, line);
                     simulation.readings_[count].dist_mm_q2 = stoi(line);
                     getline(f_r, line);
                     simulation.readings_[count].quality = stoi(line);
                     getline(f_r, line);
                     simulation.readings_[count].flag = stoi(line);
                 }
                 count++;
             }
             simulation.setCount(count);
             simulation.setLoaded(true);
             simulation.setLive(false);
             simulation.setReadingsReady(true);
             ui->sim_mode_label->setText("Simulation mode: loaded data");
             ui->joyPad->setVisible(false);
             ui->label_robot_c->setVisible(false);
             ui->label_auto->setVisible(false);
             QMessageBox msg;
             msg.setText("Data loaded!");
             msg.exec();
         }
         f_r.close();

     }
     else if (work.running())
     {
         QMessageBox msg;
         msg.setText("A real map is beeing created. You cannot load a simulation.");
         msg.exec();
     }
     else
     {
         QMessageBox msg;
         msg.setText("A simulation is active. You cannot load a new simulation.\n\n Reset and then load.");
         msg.exec();
     }
}

void MainWindow::on_b_start_stop_clicked()
{
    if(!simulation.finished()) //If simulation is not finished, some other requirements are checked.
    {
        //If there is a simulation running and no map beeing created, the simulation stops.
        if (simulation.running() && !work.running())
        {
            simulation.setRunning(false);
        }
        //If the simulation has started but is not running, it means that the simulation is stopped. If no map is beeing created, the simulation continues.
        else if (!simulation.running() && !work.running() && simulation.started())
        {
            simulation.setRunning(true);
        }
        //If the simulation has been loaded or is live, there is no map beeing created and simulation is not started or running, then it starts.
        else if (!simulation.running() && !work.running() && !simulation.started() && (simulation.loaded() or simulation.live()))
        {
            if (simulation.live())
            {
                QMessageBox msg;
                msg.setText("Choose map file to run simulation.");
                msg.exec();
                QString fileName = QFileDialog::getOpenFileName(this, tr("Open map"), ".", tr("Image Files (*.png)"));
                if (fileName.size() > 0)
                {
                    simulation.readings_ = (rplidar_response_measurement_node_hq_t*) std::malloc(sizeof(rplidar_response_measurement_node_hq_t)*200000);
                    sim_env.set_parameters(param->getPoint(), fileName, ui->text_spd->text().toDouble());
                    sim_env.setFinished(false);
                    pool->start(&sim_env);
                    while(!sim_env.started());
                    timer_.start();
                }
            }
            if (sim_env.started() or simulation.loaded())
            {
                simulation.setRunning(true);
                simulation.setStarted(true);
                param->setWidth(sim_env.width());
                param->setHeight(sim_env.height());
                simulation.set_parameters(param->getPoint(), param->getWidth(), param->getHeight(), param->getXVariation(), param->getYVariation(), param->getAngleVariation(), param->getWeight(), ui->text_spd->text().toDouble(), param->getLevmarq(), param->getRangeX(), param->getRangeY(), param->getRangeAngle(), param->getIgnoreHuman(), param->getSafetyDistance(), param->getPercOccupancy(), param->getHeuristic());
                pool->start(&simulation);
            }
        }
        //If there is a map beeing created, a warning message is shown.
        else if (work.running() || work.finished())
        {
            QMessageBox msg;
            msg.setText("You cannot start or stop the simulation because a real map is beeing created or just finished.");
            msg.exec();
        }
        //If there is no simulation loaded or is not live, a warning message is shown.
        else if (!simulation.loaded() or !simulation.live())
        {
            QMessageBox msg;
            msg.setText("There is no simulation data loaded and simulation is not live.");
            msg.exec();
        }

    }
    else
    {
        QMessageBox msg;
        msg.setText("The simulation has finished.");
        msg.exec();
    }
}

void MainWindow::btnparameters()
{

    //The parameters window will only be available when there is no map beeing created or simulation running.
    if (!work.running() && !simulation.running() && !simulation.finished() && !work.finished())
    {
        param->show();
        param->setAttribute( Qt::WA_QuitOnClose, false ); //When the main window is closed, the window param will also close.
    }
    else
    {
        QMessageBox msg;
        msg.setText("You cannot modify the parameters after starting a map (real or simulated).");
        msg.exec();
    }
}

void MainWindow::mousePressEvent(QMouseEvent* event)
{
    if(event->button() == Qt::RightButton)
    {
        QPoint aux_point = ui->map_button->mapFromGlobal(ui->map_button->cursor().pos());
        if (simulation.started() && aux_point.x() >= 0 && aux_point.x() <= ui->map_button->size().width() && aux_point.y() >= 0 && aux_point.y() <= ui->map_button->size().height())
        {
            qDebug() << "Navigating to: " << aux_point;
            simulation.setTargetP(cv::Point(aux_point.x()/(800.0/param->getWidth()), aux_point.y()/(800.0/param->getHeight())));
            simulation.setStartedNewPath(true);
            simulation.setNavigating(true);
            //simulation.start_navigation(aux_point);
        }
    }
}


void MainWindow::on_map_button_clicked()
{
    //If there are no points clicked, then it counts as the first point, and the visual elements are set to visualize the first point.
    if (points%2 == 0)
    {
        Q1 = ui->map_button->mapFromGlobal(ui->map_button->cursor().pos());//local coordinates
        ui->l_point1->setStyleSheet("background-color: rgb(54, 104, 117); color: rgb(238, 238, 236);");
        ui->t_point1->setStyleSheet("background-color: rgb(54, 104, 117); color: rgb(238, 238, 236);");
        ui->l_point1->setText(QString("(%1,%2)").arg(Q1.x()/(800.0/param->getWidth())).arg(Q1.y()/(800.0/param->getHeight())));
        ui->t_point1->setText("Point 1");
        points=1;
        ui->l_point2->setStyleSheet("");
        ui->t_point2->setStyleSheet("");
        ui->l_point2->setText("");
        ui->t_point2->setText("");
    }
    //If there is one point clicked, the visual elements are set to visualize the second point.
    else if (points%2 == 1)
    {
        Q2 = ui->map_button->mapFromGlobal(ui->map_button->cursor().pos());//local coordinates
        ui->l_point2->setStyleSheet("background-color: rgb(54, 104, 117); color: rgb(238, 238, 236);");
        ui->t_point2->setStyleSheet("background-color: rgb(54, 104, 117); color: rgb(238, 238, 236);");
        ui->l_point2->setText(QString("(%1,%2)").arg(Q2.x()/(800.0/param->getWidth())).arg(Q2.y()/(800.0/param->getHeight())));
        ui->t_point2->setText("Point 2");
        points=2;
    }
}


void MainWindow::on_b_distance_clicked()
{
    //If there are only 0 or 1 points clicked, a warning message is shown.
    if (points == 0)
    {
        QMessageBox msg;
        msg.setText("Click two points on the map and then click this button to get the distance between them.\n");
        msg.exec();
    }
    else if (points == 1)
    {
        QMessageBox msg;
        msg.setText("You need to click another point before calculating\n");
        msg.exec();
    }
    //If two points are clicked, the distance is calculated and shown on its appropiate visual element.
    else if (points == 2)
    {
        points = 0;
        double distance = sqrt(pow(Q1.x()/(800.0/param->getWidth())-Q2.x()/(800.0/param->getWidth()),2)+pow(Q1.y()/(800.0/param->getHeight())-Q2.y()/(800.0/param->getHeight()),2));
        ui->lab_distance->setStyleSheet("background-color: rgb(54, 104, 117);color: rgb(238, 238, 236);");
        ui->lab_distance->setText(QString("Distance: %1 cm").arg(distance));
    }
}




void MainWindow::on_text_spd_textChanged(const QString &arg1)
{
    simulation.setSpeed(arg1.toDouble());
    sim_env.setSpeed(arg1.toDouble());
}

void MainWindow::sim_finished()
{
   sim_env.clear();
   ui->simu_finished->setText("Simulation finished");
   QMessageBox msg;
   msg.setText("Simulation finished.\n");
   msg.exec();
}


void MainWindow::error_port()
{
    QMessageBox msg;
    msg.setText("Error, cannot bind to the specified serial port.\n");
    msg.exec();
    pool->waitForDone();
}

void MainWindow::print_img(const QImage &img)
{
    ui->map->setPixmap(QPixmap::fromImage(img.rgbSwapped()));
}

//void MainWindow::on_manual_rb_clicked()
//{
//    ui->joyPad->setVisible(true);
//    ui->label_auto->setVisible(false);
//    auto_mode = false;
//}


//void MainWindow::on_auto_rb_clicked()
//{
//    ui->joyPad->setVisible(false);
//    ui->label_auto->setVisible(true);
//    auto_mode = true;
//}

void MainWindow::joypad_y_changed(float value)
{
    qDebug() << value;
}
void MainWindow::joypad_x_changed(float value)
{
    qDebug() << value;
}

void MainWindow::on_b_end_sim_clicked()
{
    simulation.setFinished(true);
    simulation.setStarted(false);
    simulation.setRunning(false);
    sim_env.setFinished(true);
    sim_env.setStarted(false);
    sim_env.setRunning(false);
    ui->simu_finished->setText("Simulation finished");
    QMessageBox msg;
    msg.setText("Simulation finished.\n");
    msg.exec();
    qDebug() << "Elapsed time (s): " << timer_.elapsed()/1000.0;
}
