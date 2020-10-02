#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "parameters.h"
#include "simulation.h"
#include "work.h"

#include "slam.hpp"
#include "sdk/include/rplidar.h"

#include <QThreadPool>
#include <QPoint>
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



private:
    parameters *param; //Parameters window to ask and store parameters needed by the software.
    QThreadPool *pool; //Pool to launch and manage threads.
    Work work; //Class work for the real sensor scenario.
    Simulation simulation; //Class simulation for simulated scenarios.
    QPoint Q1; //First point to calculate distance between two points of the map.
    QPoint Q2; //Second point to calculate distance between two points of the map.
    int points; //Number of points clicked (0, 1 or 2).



private slots:
    //This slot launches the work thread in order to begin creating a new real map.
    void on_b_new_map_clicked(); //Slot executed when the button to create a new map is clicked.

    //This slot saves an image of the real map that is beeing created.
    void on_b_save_map_clicked(); //Slot executed when the button to save the map is clicked.

    //This slot saves the readings of the LiDAR in a text file for simulate it later.
    void on_b_save_sim_clicked(); //Slot executed when the button to save the readings for simulation is clicked.

    //This slot resets the software, finishing all threads and setting default values to the parameters.
    void on_b_restart_clicked(); //Slot executed when the button reset is clicked.

    //This slot loads an specified by the user simulation.
    void on_b_load_clicked(); //Slot executed when the button to load simulation is clicked.

    //This slot stops or starts or continues the simulation, depending on the current state of the simulation.
    void on_b_start_stop_clicked(); //Slot executed when the button to start or stop the simulation is clicked.

    //This slot show the parameters window in order to let the user set the parameters.
    void btnaction(); //Slot executed when the button to set the parameters is clicked.

    //This slot calculates and shows the distance between two points of the map previously clicked by the user.
    void on_b_distance_clicked(); //Slot executed when the button to calculate distance between two points is clicked.

    //This slot saves the coordinates of a point clicked by the user and makes sure there is only a maximum of 2 points active.
    void on_map_button_clicked(); //Slot executed when the users clicks a point in the map to calculate distance.

    //This slot sets the speed for the simulation process.
    void on_text_spd_textChanged(const QString &arg1); //Slot executed when the text which sets the speed is changed.

    //This slot sets the pixmap of a Qlabel in order to show the map in real time.
    void print_img(const QImage &img); //Slot executed when the threads (work or simulation) need to print the image of the map.

    //This slot shows a message to make the user know that the simulation has finished.
    void sim_finished(); //Slot executed when a simulation finishes.

    //This slot shows an error message to the user when the driver of the LiDAR cannot connect to the specified serial port.
    void error_port(); //Slot executed when the driver of the LiDAR cannot connect to the specified serial port.


private:
    Ui::MainWindow *ui; //Graphic user interface.
    int count; //Variable used to count the number of readings once a simulation is loaded.
};
#endif // MAINWINDOW_H
