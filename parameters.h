#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <QWidget>
#include "slam.hpp"
#include <string>

namespace Ui {
class parameters;
}

class parameters : public QWidget
{
    Q_OBJECT

private:
    Ui::parameters *ui; //Graphic user interface.

    //Variables that store the parameters needed by the SLAM algorithms.
        //range_... is used by brute force and means the range where to try the position of the system.
        //..._var_ is used by Levenberg Marquardt and means the variation of the variable between two estimations.
        //w_ represents the weight of the previous value of a pixel.
    int width_,height_, baudrate_, range_x_, range_y_, range_angle_;
    double angle_var_, x_var_, y_var_, w_;
    slam::Position p_; //Initial position of the system.
    char *port_; //The port where the LiDAR will be connected.
    bool levmarq_; //Represent whether to use Levenberg Marquardt or brute force.
    bool ignore_human_;

public:
    explicit parameters(QWidget *parent = nullptr);
    ~parameters();

    //Observers

    //Returns the width of the map.
    inline int getWidth() const {return this->width_;}

    //Returns the height of the map.
    inline int getHeight() const {return this->height_;}

    //Returns the baudrate that is going to be used to connect to the LiDAR.
    inline int getBaudrate() const {return this->baudrate_;}

    //Returns the range of X to optimize the position with the SLAM brute force algorithm.
    inline int getRangeX() const {return this->range_x_;}

    //Returns the range of Y to optimize the position with the SLAM brute force algorithm.
    inline int getRangeY() const {return this->range_y_;}

    //Returns the range of the angle to optimize the position with the SLAM brute force algorithm.
    inline int getRangeAngle() const {return this->range_angle_;}

    //Returns the initial point of the system in the map.
    inline slam::Position getPoint() const {return this->p_;}

    //Returns the variation of the angle between two estimations used by the SLAM levmarq algorithm.
    inline double getAngleVariation() const {return this->angle_var_;}

    //Returns the variation of X between two estimations used by the SLAM levmarq algorithm.
    inline double getXVariation() const {return this->x_var_;}

    //Returns the variation of Y between two estimations used by the SLAM levmarq algorithm.
    inline double getYVariation() const {return this->y_var_;}

    //Returns the weight of the previous value value of a pixel
    inline double getWeight() const {return this->w_;}

    //Returns the port which the LiDAR driver will try to connect to.
    inline char * getPort() const {return this->port_;}

    //Returns whether the levmar algorithm or the brute force algorithm will be used.
        // 1 for levmarq, 0 for brute force.
    inline bool getLevmarq() const {return this->levmarq_;}

    //Returns whether the human should be ignored or not while mapping.
    inline bool getIgnoreHuman() const {return this->ignore_human_;}


    //Modifiers

    //Sets the width of the map.
    inline void setWidth(const int &width) {this->width_ = width;}

    //Sets the height of the map.
    inline void setHeight(const int &height) {this->height_ = height;}

    //Sets the baudrate to connect to the LiDAR.
    inline void setBaudrate(const int baudrate) {this->baudrate_ = baudrate;}

    //Sets the range of X that the brute force algorithm will use in order to estimate the position.
    inline void setRangeX(const int range_x) {this->range_x_ = range_x;}

    //Sets the range of Y that the brute force algorithm will use in order to estimate the position.
    inline void setRangeY(const int range_y) {this->range_y_ = range_y;}

    //Sets the range of the angle that the brute force algorithm will use in order to estimate the position.
    inline void setRangeAngle(const int range_angle) {this->range_angle_ = range_angle;}

    //Sets the initial position of the system.
    inline void setPoint(const slam::Position P) {this->p_ = P;}

    //Sets the variation of the angle between two estimations that the Levenberg Marquardt algorithm will use.
    inline void setAngleVariation(const double angle_var) {this->angle_var_ = angle_var;}

    //Sets the variation of X between two estimations that the Levenberg Marquardt algorithm will use.
    inline void setXVariation(const double &x_var) {this->x_var_ = x_var;}

    //Sets the variation of Y between two estimations that the Levenberg Marquardt algorithm will use.
    inline void setYVariation(const double &y_var) {this->y_var_ = y_var;}

    //Sets the weight of the preivous value of a pixel.
    inline void setWeight(const double &w) {this->w_ = w;}

    //Sets the port which the driver of the LiDAR will use to connect to.
    inline void setPort(const char* &port) {strcpy(this->port_,port);}

    //Sets the algorithm that will be used:
        // 1 for levmarq, 0 for brute force.
    inline void setLevmarq(const bool &levmarq) {this->levmarq_ = levmarq;}

    void clear();//Frees memory and sets default values to parameters.

private slots:
    void on_b_accept_clicked(); //Slot to accept the modification of the parameters.

    //Slot that executes when the algorithm is changed, and it loads the default values for each algorithm.
    void on_algorithm_currentIndexChanged(int index);

    void on_b_cancel_clicked(); //Slot to decline the modification of the parameters.

    //Slot that executes when the user changes the parameter of ignoring human or not and it changes the state of that variable.
    void on_ignore_human_stateChanged(int arg1);
};

#endif // PARAMETERS_H
