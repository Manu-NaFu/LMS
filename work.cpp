#include "work.h"

#include "sdk/include/rplidar.h"
#include "levmarq.h"
#include "slam.hpp"

#include<QThread>
#include <QImage>


//Error function to optimize using Levenberg Marquardt algorithm.
void error_2(const aruco::LevMarq<double>::eVector &sol, aruco::LevMarq<double>::eVector &err, const slam::Map &map, const double *dist, const double *theta, const int count)
{
    err.resize(1);
    slam::Position P(sol(0), sol(1), sol(2));
    err(0) = map.fit(P, dist, theta, count);
}



//Function to check on RPLIDAR health status
bool checkRPLIDARHealth(rp::standalone::rplidar::RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
             drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

void Work::init()
{
    running_ = false;
    started_ = false;
    finished_ = false;
    error_ = false;
    save_ = false;
    levmarq_ = true;
    start_ = 0;
    ignore_human_ = true;
}


void Work::run()
{
    u_result op_result; //Variable to check the result of a complete reading.
    count_readings_ = 0; //Number of total readings from the start of the creation of the map.
    double dist[8192]; //Buffer to store the distance of rays in a reading.
    double theta[8192]; //Buffer to store the angle from where a ray was taken in a reading.
    slam::Map map(width_, height_); //The map that will be created.

    // create the driver instance
   rp::standalone::rplidar::RPlidarDriver * drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
   if (!drv) {
       fprintf(stderr, "insufficent memory, exit\n");
       exit(-2);
   }

   rplidar_response_device_info_t devinfo;
   bool connectSuccess = false;
   // make connection...
       if(!drv)
           drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
       else
           drv->reset();
       if (IS_OK(drv->connect(port_, baudrate_)))
       {
           op_result = drv->getDeviceInfo(devinfo);

           if (IS_OK(op_result))
           {
               connectSuccess = true;
           }
           else
           {
               delete drv;
               drv = NULL;
           }
       }


   //If it was not able to connect, there was a problem with the port.
   if (!connectSuccess) {
       emit error_port();
       error_ = true;
   }
   else
   {
       // print out the device serial number, firmware and hardware version number..
       printf("RPLIDAR S/N: ");
       for (int pos = 0; pos < 16 ;++pos) {
           printf("%02X", devinfo.serialnum[pos]);
       }

       printf("\n"
               "Firmware Ver: %d.%02d\n"
               "Hardware Rev: %d\n"
               , devinfo.firmware_version>>8
               , devinfo.firmware_version & 0xFF
               , (int)devinfo.hardware_version);
   }


   // check health...
   if (!error_)
   {
       if (!checkRPLIDARHealth(drv)) {
           error_ = true;
       }
   }




   //If there were not errors, the creating of the map starts.
   if(!error_)
   {
       drv->startMotor();
       // start scan...
       drv->startScan(0,1);
       running_ = true;
       readings_ =(rplidar_response_measurement_node_hq_t*) std::malloc(sizeof(rplidar_response_measurement_node_hq_t)*200000);
   }
   else
   {
       started_ = false;
       running_ = false;
       error_ = false;
   }

    while(running_)
    {

        rplidar_response_measurement_node_hq_t nodes[8192]; //Readings from a complete spin.
        size_t   count = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t); //Number of readings per spin.
        int x=0,y=0; //Variables to calculate where to print a line in the map.



        op_result = drv->grabScanDataHq(nodes, count);

        if (IS_OK(op_result)) {

            drv->ascendScanData(nodes, count); //Sorts the measures by angle


            map.undrawSystem(P_);

            //Calculates the distance in cm and the angle in degrees.
            //It also saves the readings in the buffer for saving them later.
            for (int pos = 0; pos < (int)count ; ++pos) {
                dist[pos] = nodes[pos].dist_mm_q2 / 10.f / (1 << 2);
                theta[pos] = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
                count_readings_++;
                if (count_readings_ < 200000)
                {
                    readings_[count_readings_].angle_z_q14 = nodes[pos].angle_z_q14;
                    readings_[count_readings_].dist_mm_q2 = nodes[pos].dist_mm_q2;
                    readings_[count_readings_].quality = nodes[pos].quality;
                    readings_[count_readings_].flag = nodes[pos].flag;
                }
            }


            //If is not the first frame, the algorithm begins.
            if (start_ == 1)
            {
                //If the selected algorithm is Levenberg Marquardt, then a solver is created.
                //The params for the solver would be to stop after 2000 iterations or after the error is under 0.000001.
                //The variation of each variable (x,y,angle) is algo set.
                if (levmarq_)
                {
                    aruco::LevMarq<double> Solver;
                    aruco::LevMarq<double>::eVector sol(3);
                    Solver.setParams(500,0.000001);
                    sol(0) = P_.getX();
                    sol(1) = P_.getY();
                    sol(2) = P_.getAngle();
                    Solver.setDervEpsilon({x_var_, y_var_, angle_var_});
                    Solver.solve(sol, bind(error_2, std::placeholders::_1, std::placeholders::_2, map, dist, theta, count));
                    P_.setX(sol(0));
                    P_.setY(sol(1));
                    P_.setAngle(sol(2));
                }
                //If brute force is the chosen algorithm, the software will execute this code.
                else
                {
                    map.bruteForce(P_, dist, theta, count, range_x_, range_y_, range_angle_, x_var_, y_var_, angle_var_);
                }


            }

            //Once the position is estimated, this code calculates where to print lines using trigonometry to obtain the x and y coordinates in the map and prints the lines.
            for (int pos = 0; pos < (int)count ; ++pos)
            {
                //If the software should ignore the human, then ignores some angles of the readings.
                if (ignore_human_)
                {
                    if(theta[pos] >125 || theta[pos] < 55)
                    {
                        x = (cos( (theta[pos]+P_.getAngle()) * PI / 180.0 ) * dist[pos]) + P_.getX();
                        y = (sin( (theta[pos]+P_.getAngle()) * PI / 180.0 ) * dist[pos]) + P_.getY();

                        if (x <= width_ && y <= height_ && dist[pos]>0.1)
                            map.lineToObject(P_.getX(), P_.getY(), x, y, w_);
                    }
                }
                else
                {
                    x = (cos( (theta[pos]+P_.getAngle()) * PI / 180.0 ) * dist[pos]) + P_.getX();
                    y = (sin( (theta[pos]+P_.getAngle()) * PI / 180.0 ) * dist[pos]) + P_.getY();

                    if (x <= width_ && y <= height_ && dist[pos]>0.1)
                        map.lineToObject(P_.getX(), P_.getY(), x, y, w_);
                }
            }


            //The map is updated with the new position of the system and the new lines printed.
            map.update(P_);

            //From now on, the software will know that it is not the first frame.
            start_ = 1;

            //The image is sent to the main window as a pixmap to be printed in the Qlabel.
            mat_ = map.getMap();
            cv::resize(mat_,mat_,cv::Size(800,800));
            QImage img((const uchar*)mat_.data, mat_.cols, mat_.rows, mat_.step, QImage::Format_RGB888);
            emit(print_img(img.copy()));


            //The image is saved into the mat_ buffer in order to save it from the main window while save_ is active
            mat_ = map.getMap();
            while(save_){}
        }
    }

    drv->stop();
    drv->stopMotor();
    rp::standalone::rplidar::RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return;
}


void Work::set_parameters(slam::Position p, int width, int height, int x_var, int y_var, double angle_var, double w, int baudrate, char *port, bool levmarq, int range_x, int range_y, int range_angle, bool ignore_human)
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
    baudrate_ = baudrate;
    levmarq_ = levmarq;
    range_x_ = range_x;
    range_y_ = range_y;
    range_angle_ = range_angle;
    ignore_human_ = ignore_human;
    port_ = (char*) std::malloc(sizeof(char)*256);
    strcpy(port_, port);
}


void Work::clear()
{
    //Default values for the parameters and free memory.
    set_parameters(slam::Position(400,400,0), 800, 800, 1, 1, 1.5, 0.7, 115200, "/dev/ttyUSB0", true, 20, 20, 60, true);
    running_ = false;
    start_ = 0;
    error_ = false;
    started_ = false;
    finished_ = false;
    count_readings_ = 0;


    for (int i = 0; i < 200000; i++)
    {
        readings_[i].angle_z_q14 = 0;
        readings_[i].dist_mm_q2 = 0;
        readings_[i].quality = 0;
        readings_[i].flag = 0;
    }

    delete[] readings_;
}







