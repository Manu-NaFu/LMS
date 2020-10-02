#ifndef _SLAMHPP_
#define _SLAMHPP_

#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdlib>


#define PI 3.14159265


namespace slam
{
    //Clas to store and manage the position of a system in order to implement SLAM.
    class Position
    {
        private:
            int x_, y_; //Stores X and Y variables of the system.
            double angle_; //Stores the angle of the system.

        public: 

        //Constructors
            Position(int x=0, int y=0, double angle=0) //Default contrstuctor. Accepts the three components of the position.
            {
                this->setX(x);
                this->setY(y);
                this->setAngle(angle);
            }

            Position(const Position & p) //Overloaded constructor, accepts another position as parameter.
            {
                this->setX(p.getX());
                this->setY(p.getY());
                this->setAngle(p.getAngle());
            }

        //Observers
            inline const int & getX() const {return this-> x_;} //Returns the component X of the system.
            inline const int & getY() const {return this-> y_;} //Returns the component Y of the system.
            inline const double & getAngle() const {return this-> angle_;} //Returns the component angle of the system.

        //Modifiers
            //Sets the component X of the system. Recieves a const int by reference.
            inline void setX(const int & x) {this->x_ = x;}
            //Sets the component Y of the system. Recieves a const int by reference.
            inline void setY(const int & y) {this->y_ = y;}
            //Sets the component angle of the system. Recieves a const int by reference.
            inline void setAngle(const double & angle) {this->angle_ = angle;}

            //Operator to allow assignment using '='.
            slam::Position &operator = (const slam::Position p)
            {
                this->setX(p.getX());
                this->setY(p.getY());
                this->setAngle(p.getAngle());
                return *this;
            }
    };

    //Class to store and perform SLAM using a occupation matrix (oc_) and a visual matrix (map_).
    //Needs a slam::Position in order to locate the system.
    //The occupation work as follows:
        // 0 for unknown space.
        // 1 to 255 for % of occupation.  1 means low occupation and 255 means high occupation.
    class Map
    {
        private:
            cv::Mat map_;
            cv::Mat oc_;

        public: 

        //Constructors
            //Default constructor given X (width) and Y(height) components.
            //Initiates every pixel to 0;
            Map(const int &x = 800, const int &y = 800)
            {
                cv::Mat map(x, y, CV_8UC3);
                this->setMap(map);
                cv::Mat oc(x, y, CV_8UC1);
                this->oc_ = oc;

                for(int i=0;i<this->getRows();i++)
                    for(int j=0;j<this->getCols();j++)
                    {
                        this->setValues(i,j,0,0,0);
                        this->setValuesOc(i,j,0);
                    }
            }

            Map(cv::Mat map) {this->setMap(map);} //Overloaded constructor given a map

        //Observers
            inline cv::Mat getMap() const {return this-> map_;} //Returns the visual map as cv::Mat.
            inline cv::Mat getOc() const {return this-> oc_;} //Returns the occupation map as cv::Mat.
            inline int getRows() const {return this->map_.rows;} //Returns the rows (height) as int.
            inline int getCols() const {return this->map_.cols;} //Returns the columns (width) as int.

            //Returns the values for a pixel from the visual map using the coordinates.
            //If the map is bigger than the point, it returns the point. If not, it returns the first point of the map.
            inline const cv::Vec3b & getPixel(const int x, const int y) const 
            {
                if (this->getRows() >= y && this->getCols() >= x) 
                    return map_.at<cv::Vec3b>(y,x);
                else
                    return map_.at<cv::Vec3b>(0,0);
                    
            }

            //Overloaded function to obtain the values of a pixel using a Points instead of the coordinates.
            //If the map is bigger than the point, it returns the point. If not, it returns the first point of the map.
            inline const cv::Vec3b & getPixel(const cv::Point & pos) const 
            {
                if (this->getRows() >= pos.y && this->getCols() >= pos.x) 
                    return map_.at<cv::Vec3b>(pos);
                else
                    return map_.at<cv::Vec3b>(0,0);

            }

            //Returns the value of a channel of a pixel using the coordinates and the channel defined by an int.
            //If the map is bigger than the point, it returns the point. If not, it returns -1.
            int getPixel_channel(const int x, const int y, const int channel) const
            {
                if (this->getRows() >= y && this->getCols() >= x) 
                    return this->map_.at<cv::Vec3b>(y,x)[channel];
                else 
                    return -1;
            }

            //Returns the value of a channel of a pixel using the coordinates and the channel defined by a char.
            //If the map is bigger than the point, it returns the point. If not, it returns -1.
            int getPixel_channel(const int x, const int y, const char channel) const
            {
                if (channel == 'b' && this->getRows() >= y && this->getCols() >= x)
                    return this->map_.at<cv::Vec3b>(y,x)[0];
                else if (channel == 'g' && this->getRows() >= y && this->getCols() >= x)
                    return this->map_.at<cv::Vec3b>(y,x)[1];
                else if (channel == 'r' && this->getRows() >= y && this->getCols() >= x)
                    return this->map_.at<cv::Vec3b>(y,x)[2];
                else
                    return -1;
            }


            //Returns the values for a pixel from the visual map using the coordinates.
            //If the map is bigger than the point, it returns the point. If not, it returns the first point of the map.
            inline const uchar & getPixelOc(const int x, const int y) const
            {
                if (this->getRows() >= y && this->getCols() >= x && y>=0 && x>=0)
                    return oc_.at<uchar>(y,x);
                else
                    return oc_.at<uchar>(0,0);
                    
            }

            //Overloaded function to obtain the values of a pixel using a Points instead of the coordinates.
            //If the map is bigger than the point, it returns the point. If not, it returns the first point of the map.
            inline const uchar & getPixelOc(const cv::Point & pos) const
            {
                if (this->getRows() >= pos.y && this->getCols() >= pos.x && pos.y>=0 && pos.x>=0)
                    return oc_.at<uchar>(pos);
                else
                    return oc_.at<uchar>(0,0);
                    
            }

        //Modifiers
            inline void setMap(const cv::Mat & map) {this->map_ = map;} //Assigns the visual map to the specified cv::Mat.
            inline void setOc(const cv::Mat & map) {this->oc_ = map;} //Assigns the occupation map to the specified cv::Mat.

            //Frees the memory of the two maps.
            inline void release(){
                map_.release();
                oc_.release();
            }

            //Resizes both maps according to the specified width and height.
            inline void resize(const int &width, const int &height){
                map_.create(width, height, CV_8UC3);
                oc_.create(width, height, CV_8UC1);
            }

            //Sets the rgb values of a pixel in the visual map given the values and coordinates.
            inline void setValues(const int & x, const int & y, const int & r, const int & g, const int & b)
            {
                if (this->getRows() >= y && this->getCols() >= x && y>=0 && x>=0)
                {
                    map_.at<cv::Vec3b>(y,x)[0] = b;
                    map_.at<cv::Vec3b>(y,x)[1] = g;
                    map_.at<cv::Vec3b>(y,x)[2] = r;
                }
                
            }

            //Sets the rgb values of a pixel in the visual map given the values and the cv::Point.
            inline void setValues(const cv::Point & pos, const int & r, const int & g, const int & b)
            {
                if (this->getRows() >= pos.y && this->getCols() >= pos.x && pos.y>=0 && pos.x>=0)
                {
                    map_.at<cv::Vec3b>(pos)[0] = b;
                    map_.at<cv::Vec3b>(pos)[1] = g;
                    map_.at<cv::Vec3b>(pos)[2] = r;
                }
            }

            //Sets the occupation value of a pixel in the occuation map given the value and coordinates.
            inline void setValuesOc(const int & x, const int & y, const int & value)
            {
                if (this->getRows() >= y && this->getCols() >= x && y>=0 && x>=0)
                {
                    oc_.at<uchar>(y,x) = value;
                }
                
            }

            //Sets the occupation value of a pixel in the occuation map given the value and the cv::Point.
            inline void setValuesOc(const cv::Point & pos, const int & value)
            {
                if (this->getRows() >= pos.y && this->getCols() >= pos.x && pos.y>=0 && pos.x>=0)
                {
                    oc_.at<uchar>(pos) = value;
                }
            }

            //Sets all the values for all the pixels on both matrixes to 0.
            inline void set_zeros()
            {
                for(int i=0;i<this->getRows();i++)
                    for(int j=0;j<this->getCols();j++)
                    {
                        this->setValues(i,j,0,0,0);
                        this->setValuesOc(i,j,0);
                    }
            }



            //Function to draw a line in the map from the position of the system to an objectgiven two points and the weight of last value.
            void lineToObject(const int & x0, const int & y0, const int & x1, const int & y1, const double & w)
            {
                if(x0 > 0 && x0 < getCols() && y0 > 0 && y0 < getRows() && x1 > 0 && x1 < getCols() && y1 > 0 && y1 < getRows() && w >= 0 && w <= 1)
                {
                    cv::Point p0(x0,y0), p1(x1,y1);

                    //Line iterator from opencv that implements the Bresenham algorithm.
                    cv::LineIterator it(oc_, p0, p1, 8);

                    int value = 0;

                    //The iterator is iterated to take into account all the pixels in the line between the system and the object.
                    for(int i = 0; i < it.count - 3; i++, ++it)
                    {
                        value = rint(getPixelOc(it.pos())*w + 127*(1.0-w));
                        setValuesOc(it.pos(), value);
                    }

                    value = rint(getPixelOc(it.pos())*w + 191*(1.0-w));
                    setValuesOc(it.pos(), value);
                    it++;
                    value = rint(getPixelOc(it.pos())*w + 223*(1.0-w));
                    setValuesOc(it.pos(), value);
                    it++;
                    value = rint(getPixelOc(it.pos())*w + 255*(1.0-w));
                    setValuesOc(it.pos(), value);
                    it++;
                    value = rint(getPixelOc(it.pos())*w + 223*(1.0-w));
                    setValuesOc(it.pos(), value);
                    it++;
                    value = rint(getPixelOc(it.pos())*w + 191*(1.0-w));
                    setValuesOc(it.pos(), value);
                }

            }

            //Function to draw a line with a desired colour from one point to another
            void line(const int x0, const int y0, const int x1, const int y1, const int r, const int g, const int b)
            {
                cv::Point p0(x0,y0), p1(x1,y1);

                cv::LineIterator it(this->map_, p0, p1, 8);

                for(int i = 0; i < it.count; i++, ++it)
                    this->setValues(it.pos(), r, g, b);
            }


            //Function that calculates the fitness for a specified position according to the occupation map.
            //Recieves the data necesary to compare the point with the map, which are:
                //try_P: the point to evaluate.
                //Current_dist: the distance readings obtained from the LiDAR.
                //theta: the angle from which the distances were taken.
                //count: the number of distance readings.
            //Returns the fitness from 0 to 1 where 0 means the best fitness and 1 means the worst fitness.
            double fit(const slam::Position try_P, const double *current_dist, const double *theta,  int count) const
            {
                double fitness = 0.0;
                int x = 0, y = 0;
                
                for (int pos = 0; pos < count; pos++)
                {
                    //Si la medición fue válida (la distancia medida fue mayor a 0.1), se tiene en cuenta.
                    if (current_dist[pos] > 0.1)
                    {
                        x = (cos( (theta[pos]+try_P.getAngle()) * PI / 180.0 ) * current_dist[pos]) + try_P.getX();
                        y = (sin( (theta[pos]+try_P.getAngle()) * PI / 180.0 ) * current_dist[pos]) + try_P.getY();
                        if(x < getCols() && y < getRows())
                            fitness+= ((int)getPixelOc(x,y));
                    }
                }
                
                return (255.0*count - fitness)/(255.0*count);
            }

            //Function that applies brute force to estimate the position and orientation of the system in the map.
                //P is the points to be calculated
                //dist is a buffer with the distances from the LiDAR
                //theta is a buffer with the angle from where the distances were taken.
                //count is the number of rays or measures.
                //range_x is the range where to try with the X component.
                //range_y is the range where to try with the Y component.
                //range_angle is the range where to try with the angle component.
            void bruteForce(slam::Position &P, const double *dist, const double *theta, const int &count, const int &range_x, const int &range_y, const int &range_angle, const int & x_var, const int &y_var, const double &angle_var)
            {
                //Variables to try and optimize the position of the system in the map.
                slam::Position try_P(P.getX()-20, P.getY()-20, P.getAngle()-30);
                double opt = 0.0;
                double opt_try = 0.0;
                slam::Position prev_P(P);
                //First value of optimization.
                opt = fit(prev_P, dist, theta, count);

                //The following code will test the different positions of de system in a given range.
                //From (current position - range_y_/2) to (current position + range_y/2)
                for (int i = 0; i < range_y; i=i+y_var){
                    try_P.setY(prev_P.getY() - range_y/2 + i);

                    //From (current position - range_x_/2) to (current position + range_x/2)
                    for (int j = 0; j < range_x; j=j+x_var){
                        try_P.setX(prev_P.getX() - range_x/2 + j);

                        //From (current angle - range_angle_/2) to (current angle + range_angle/2)
                        for (int k = 0; k < range_angle; k=k+angle_var){
                            try_P.setAngle(prev_P.getAngle()-range_angle/2+k);
                            opt_try = fit(try_P, dist, theta, count);

                            //If the new value is better than the previous one, the position is updated.
                            if (opt_try < opt)
                            {
                                P = try_P;
                                opt = opt_try;
                            }
                        }
                    }
                }
            }



            //Draws the system in the visual map.
            void drawSystem(slam::Position P, const int r, const int g, const int b)
            {
                for (int i = 0; i < 5; i++)
                    for (int j = 0; j < 5; j++)
                        setValues(P.getX()-2+i,P.getY()-2+j, r, g, b);
                
                int x = (cos( (P.getAngle()+270) * PI / 180.0 ) * 10) + P.getX();
                int y = (sin( (P.getAngle()+270) * PI / 180.0 ) * 10) + P.getY();
                line(P.getX(), P.getY(), x, y, r, g, b);
            }

            //Undraws the system from the visual map.
            void undrawSystem(slam::Position P)
            {
                for (int i = 0; i < 5; i++)
                    for (int j = 0; j < 5; j++)
                        setValues(P.getX()-2+i,P.getY()-2+j, 60, 60, 60);
                
                int x = (cos( (P.getAngle()+270) * PI / 180.0 ) * 10) + P.getX();
                int y = (sin( (P.getAngle()+270) * PI / 180.0 ) * 10) + P.getY();
                line(P.getX(), P.getY(), x, y, 60, 60, 60);
            }

            //Updates the visual map according to the occupation map. It also draws the system in the visual map.
            void update(slam::Position P)
            {
                for(int i=0;i<this->getRows();i++)
                    for(int j=0;j<this->getCols();j++)
                    {
                        if (this->getPixelOc(i,j) != 0)
                        {
                            //If the value of occupation is under 161, we take it as unoccupied.
                            if (this->getPixelOc(i,j) < 161)
                                this->setValues(i,j,60,60,60);
                            //If the value is under 30, we take it as unknown.
                            else if (this->getPixelOc(i,j) < 30)
                                this->setValues(i,j,0,0,0);
                            //If the value is over 222, we take it as occupied.
                            else if (this->getPixelOc(i,j) > 222)
                                this->setValues(i,j,255,0,0);
                            //If the value is between 161 and 222, we take it as parcially occupied.
                            else if (this->getPixelOc(i,j) > 160)
                                this->setValues(i,j,80,0,0);
                        }
                    }
                this->drawSystem(P, 0, 0, 255);
            }

            //Operator to allow assignment using '='.
            slam::Map &operator = (const slam::Map m)
            {
                this->setMap(m.getMap());
                this->setOc(m.getOc());
                return *this;
            }
    };


}

#endif
