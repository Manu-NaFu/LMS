#ifndef WORK_H
#define WORK_H

#include<QObject>
#include<QRunnable>
#include<unistd.h>
#include<QTimer>
#include <QThread>
#include "slam.hpp"
#include "sdk/include/rplidar.h"


class Work : public QObject, public QRunnable
{
    Q_OBJECT

private:
    bool running_; //Represents whether the real map is beeing created or not.
    bool error_; //Used to specify if an error has occurred during the connection to the LiDAR.
    bool started_; //Represents whether the thread has started creating the real map or not.
    bool finished_; //Represents whether the real map is finished or not.
    bool save_; //Represents whether the thread has to be blocked while the map is saved by the main window.
    bool levmarq_; //Represents whether to use Levenberg Marquardt algorithm or brute force.
    bool ignore_human_;
    int start_; //Represents whether it is the first frame or not.
    int count_readings_; //Number of total readings from the start.
    slam::Position P_; //Position of the system.

    //Variables that store the parameters needed by the SLAM algorithms.
        //range_... is used by brute force and means the range where to try the position of the system.
        //..._var_ is used by Levenberg Marquardt and means the variation of the variable between two estimations.
        //w_ represents the weight of the previous value of a pixel.
    int width_,height_, baudrate_, range_x_, range_y_, range_angle_;
    double x_var_, y_var_, angle_var_,w_;
    char *port_;//The port where the LiDAR will be connected.
    cv::Mat mat_; //Matrix to store the visual map while it is saved as an image.


public:
    rplidar_response_measurement_node_hq_t * readings_; //Readings from the LiDAR to save to file.

    void init(); //Initialises bool variables.
    void run(); //Function that will run as a thread.


    //Observers

    //Returns whether the real map is beeing created or not.
    inline bool running() const {return this->running_;}

    //Returns whether the creation of a map has started or not.
    inline bool started() const {return this->started_;}

    //Returns whether an error has occurred during the connection to the LiDAR or not.
    inline bool error() const {return this->error_;}

    //Returns whether the thread should be stopped in order ot let the main window save the map.
    inline bool save() const {return this->save_;}

    //Returns whether the creation of a map has finished or not.
    inline bool finished() const {return this->finished_;}

    //Returns whether the algorithm used is Levenberg Marquart or Brute force.
        // 1 for levmarq, 0 for brute force.
    inline bool levmarq() const {return this->levmarq_;}

    //Returns whether it is the first frame or not.
    inline int start() const {return this->start_;}

    //Returns the number of total readings from the start.
    inline int getCountReadings() const {return this->count_readings_;}

    //Returns the visual map as a cv::Mat.
    inline cv::Mat getImgMap() const {return this->mat_;}

    //Modifiers

    //Allows to set the state of the creation of the map to running or not running.
    inline void setRunning(const bool &running) {this->running_ = running;}

    //Allows to set the state of the creation of a map to started or not started.
    inline void setStarted(const bool &started) {this->started_ = started;}

    //Allows to set the state of the creation of a map to finished or unfinished.
    inline void setFinished(const bool &finished) {this->finished_ = finished;}

    //Allows to set the state of the creation of a map to saving or not saving.
    inline void setSave(const bool &save) {this->save_ = save;}

    //Sets the algorithm as levmarq or brute force.
    inline void setLevmarq(const bool &levmarq) {this->levmarq_ = levmarq;}

    //Allows to set the variable start_ to check if it is the first frame or not.
    inline void setStart(const int &start) {this->start_ = start;}

    //Sets the number of readings from the LiDAR from the start of the creation of the map.
    inline void setCountReadings(const int &count) {this->count_readings_ = count;}

    //Sets the parameters needed by the SLAM algorithms.
    void set_parameters(slam::Position p, int width, int height, int x_var, int y_var, double angle_var, double w, int baudrate, char *port, bool levmarq, int range_x, int range_y, int range_angle, bool ignore_human);

    void clear(); //Frees memory and sets default values to parameters.

signals:
    void print_img(const QImage &img); //This signal is sent every time the map is updated and ready to be printed.
    void error_port(); //This signal is sent when the driver of the LiDAR could not connect to the specified serial port.

};

#endif // WORK_H
