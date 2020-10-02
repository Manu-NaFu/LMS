#include "simulation.h"
#include "slam.hpp"
#include "levmarq.h"
#include<QThread>
#include<QMessageBox>


//Error function to optimize using Levenberg Marquardt algorithm.
void error(const aruco::LevMarq<double>::eVector &sol, aruco::LevMarq<double>::eVector &err, const slam::Map &map, const double *dist, const double *theta, const int count)
{
    err.resize(1);
    slam::Position P(sol(0), sol(1), sol(2));
    err(0) = map.fit(P, dist, theta, count);
}


void Simulation::init()
{
    running_ = false;
    started_ = false;
    finished_ = false;
    loaded_ = false;
    start_ = 0;
    ignore_human_ = true;
}


void Simulation::run()
{

    slam::Map map(width_,height_); //Map that will be created.
    int   count = count_; //Count of reading stages.
    int pos = 0, prev_pos = 0; //Counters for managing readings.
    int x = 0, y = 0; //Variables to calculate where to print a line in the map.
    double *dist_ =(double*) std::malloc(sizeof(double)*8192); //Vector of distances from the laser ray.
    double *theta_ =(double*) std::malloc(sizeof(double)*8192); //Vector of angles from the laser ray.



    //While the map is not finished and there are still readings (pos < count).
    while(pos < count && !finished_ && started_)
    {
        //while the simulation is stopped, save the map.
        while(!running_ && !map.getMap().empty() && !finished_ && started_)
        {
            if(!started_)
                break;
            else
                cv::imwrite("map_sim_stop.png",map.getMap());
        }
        //If the user has reset while the simulation was stopped, leave.
        if (!started_){break;}

        //Undraw the system from the map in order to clear that space.
        map.undrawSystem(P_);

        //Manage readings.
        prev_pos = pos;
        //Total readings in this stage.
        int t_stg_readings = 0;

        //While the angle is not superior to 358, get the angle and the distance and save it into a buffer.
        for (t_stg_readings = 0; readings_[pos].angle_z_q14 * 90.f / (1 << 14) < 358.0 ; t_stg_readings++)
        {
            //Angle in degrees
            theta_[t_stg_readings] = readings_[pos].angle_z_q14 * 90.f / (1 << 14);
            //Distance in cm
            dist_[t_stg_readings] = readings_[pos].dist_mm_q2 / 10.f / (1 << 2);
            pos++;
        }
        //If it is not the first reading and there has been readings (pos-prev_pos != 0), the position is estimated.
        if (start_ == 1 && pos-prev_pos != 0)
        {
            //If the selected algorithm is Levenberg Marquardt, then a solver is created.
            //The params for the solver would be to stop after 2000 iterations or after the error is under 0.000001.
            //The variation of each variable (x,y,angle) is algo set.
            if (levmarq_)
            {
                aruco::LevMarq<double> Solver;
                aruco::LevMarq<double>::eVector sol(3);
                Solver.setParams(2000,0.000001);
                sol(0) = P_.getX();
                sol(1) = P_.getY();
                sol(2) = P_.getAngle();
                Solver.setDervEpsilon({x_var_, y_var_, angle_var_});
                Solver.solve(sol, bind(error, std::placeholders::_1, std::placeholders::_2, map, dist_, theta_, pos-prev_pos));
                P_.setX(sol(0));
                P_.setY(sol(1));
                P_.setAngle(sol(2));
            }
            else
            //If brute force is the chosen algorithm, the software will execute this code.
            {
                map.bruteForce(P_, dist_, theta_, pos-prev_pos, range_x_, range_y_, range_angle_, x_var_, y_var_, angle_var_);
            }
        }

        //Once the position is estimated, this code calculates where to print lines using trigonometry to obtain the x and y coordinates in the map and prints the lines.
        for (int i = 0; i < t_stg_readings ; i++)
        {
            //If the software should ignore the human, then ignores some angles of the readings.
            if (ignore_human_)
            {
                if(theta_[i] >125 || theta_[i] < 55)
                {
                    x = (cos( (theta_[i]+P_.getAngle()) * PI / 180.0 ) * dist_[i]) + P_.getX();
                    y = (sin( (theta_[i]+P_.getAngle()) * PI / 180.0 ) * dist_[i]) + P_.getY();


                    if (x <= map.getCols() && y <= map.getRows() && dist_[i]>0.1)
                        map.lineToObject(P_.getX(), P_.getY(), x, y, w_);
                }
            }
            else
            {
                x = (cos( (theta_[i]+P_.getAngle()) * PI / 180.0 ) * dist_[i]) + P_.getX();
                y = (sin( (theta_[i]+P_.getAngle()) * PI / 180.0 ) * dist_[i]) + P_.getY();
                //printf("theta (degrees): %03.2f Dist (cm): %08.2f Pos: %d X: %d Y: %d  \n",  theta, dist[pos], pos,points[pos].x,points[pos].y);

                if (x <= map.getCols() && y <= map.getRows() && dist_[i]>0.1)
                    map.lineToObject(P_.getX(), P_.getY(), x, y, w_);
            }
        }

        //The map is updated with the new position of the system and the new lines printed.
        map.update(P_);

        //The image is sent to the main window as a pixmap to be printed in the Qlabel.
        mat_ = map.getMap();
        cv::resize(mat_,mat_,cv::Size(800,800));

        QImage img((const uchar*)mat_.data, mat_.cols, mat_.rows, mat_.step, QImage::Format_RGB888);
        emit(print_img(img.copy()));

        //The time this process is asleep is proportional to the speed that the users sets.
        usleep((10000/speed_)*1000);

        //From now on, the software will know it is not the first frame.
        start_ = 1;

        //If there were some readings regarding, this code ignores them.
        while (readings_[pos].angle_z_q14 * 90.f / (1 << 14) > 358.0)
            pos++;

        //If pos >= count it means the simulation should finish as there are no more readings regarding.
        //The image of the map is saved.
        if (pos >= count)
        {
            running_ = false;
            finished_ = true;
            emit sim_finished();
            cv::imwrite("map_sim_finished.png",map.getMap());
        }
    }

}



void Simulation::set_parameters(slam::Position p, int width, int height, int x_var, int y_var, double angle_var, double w, double speed, bool levmarq, int range_x, int range_y, int range_angle, bool ignore_human)
{
    P_.setX(p.getX());
    P_.setY(p.getY());
    P_.setAngle(p.getAngle());
    x_var_= x_var;
    y_var_= y_var;
    width_= width;
    height_ =height;
    w_ = w;
    angle_var_ = angle_var;
    speed_ = speed;
    range_x_ = range_x;
    range_y_ = range_y;
    range_angle_ = range_angle;
    levmarq_ = levmarq;
    ignore_human_ = ignore_human;
}

void Simulation::clear()
{
    //Default values for parameters and free memory.
    set_parameters(slam::Position(400,400,0), 800, 800, 1, 1, 1.5, 0.7, 100, true, 20, 20, 60, true);
    finished_ = true;
    started_ = false;
    cv::waitKey(2000);
    running_ = false;
    finished_ = false;
    loaded_ = false;
    start_ = 0;
    count_ = 0;

    for (int i = 0; i < 200000; i++)
    {
        readings_[i].angle_z_q14 = 0;
        readings_[i].dist_mm_q2 = 0;
        readings_[i].quality = 0;
        readings_[i].flag = 0;
    }

    delete[] readings_;
}





