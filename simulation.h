#ifndef SIMULATION_H
#define SIMULATION_H

#include<QObject>
#include<QRunnable>
#include<unistd.h>
#include<QTimer>
#include<QThread>
#include <QImage>
#include "sdk/include/rplidar.h"
#include "slam.hpp"
#include "levmarq.h"

class Simulation : public QObject, public QRunnable
{
    Q_OBJECT

private:
    bool running_; //Represents whether the simulation is running or not.
    bool started_; //Represents whether the simulation has started or not.
    bool finished_; //Represents whether the simulation has finished or not.
    bool loaded_; //Represents whether the data of a simulation has been loaded or not.
    bool levmarq_; //Represents whether to use Levenberg Marquardt algorithm or brute force.
    bool ignore_human_; //Represents wheter to ignore the human or not while mapping.
    int start_; //Represents whether it is the first frame or not.
    slam::Position P_;//Position of the system.

    //Variables that store the parameters needed by the SLAM algorithms.
        //range_... is used by brute force and means the range where to try the position of the system.
        //..._var_ is used by Levenberg Marquardt and means the variation of the variable between two estimations.
        //The speed of the simulation will be proportional to the variable speed_.
        //w_ represents the weight of the previous value of a pixel.
    int width_,height_, count_, range_x_, range_y_, range_angle_;
    double x_var_, y_var_, angle_var_,w_, speed_;
    cv::Mat mat_; //Used for sending the visual map as image to the main window.


public:
    rplidar_response_measurement_node_hq_t * readings_; //Readings from the LiDAR read from file.

    void init(); //Initialises bool variables.
    void run(); //Function that will run as a thread.


    //Observers

    //Returns whether the simulation is running or not.
    inline bool running() const {return this->running_;}

    //Returns whether the creation of a map has started or not.
    inline bool started() const {return this->started_;}

    //Returns whether the creation of a map has finished or not.
    inline bool finished() const {return this->finished_;}

    //Returns whether the data for simulation has been loaded or not.
    inline bool loaded() const {return this->loaded_;}

    //Returns whether the algorithm used is Levenberg Marquart or Brute force.
        // 1 for levmarq, 0 for brute force.
    inline bool levmarq() const {return this->levmarq_;}

    //Returns whether it is the first frame or not.
    inline int start() const {return this->start_;}

    //Returns the total number of readings read from the simulation file.
    inline int getCount() const {return this->count_;}

    //Returns the speed at which the simulation is run.
    inline double getSpeed() const {return this->speed_;}


    //Modifiers

    //Allows to set the state of the simulation to running or not running.
    inline void setRunning(const bool &running) {this->running_ = running;}

    //Allows to set the state of the simulation to started or not started.
    inline void setStarted(const bool &started) {this->started_ = started;}

    //Allows to set the state of the simulationcreation of a map to finished or unfinished.
    inline void setFinished(const bool &finished) {this->finished_ = finished;}

    //Allows to set the state of the simulation to loaded or not loaded.
    inline void setLoaded(const bool &loaded) {this->loaded_ = loaded;}

    //Sets the algorithm as levmarq or brute force.
    inline void setLevmarq(const bool &levmarq) {this->levmarq_ = levmarq;}

    //Allows to set the variable start_ to check if it is the first frame or not.
    inline void setStart(const int &start) {this->start_ = start;}

    //Sets the number of readings from the LiDAR read from the simulation file.
    inline void setCount(const int &count) {this->count_ = count;}

    //Sets the speed of the simulation.
    inline void setSpeed(const double &speed) {this->speed_ = speed;}

    //Sets the parameters needed by the SLAM algorithms.
    void set_parameters(slam::Position p, int width, int height, int x_var, int y_var, double angle_var, double w, double speed, bool levmarq, int range_x, int range_y, int range_angle, bool ignore_human);

    void clear(); //Frees memory and sets default values to parameters.


signals:
    void print_img(const QImage &img); //This signal is sent every time the map is updated and ready to be printed.
    void sim_finished(); //Signal sent to make the main window aware that the simulations has finished.

};

#endif // SIMULATION_H
