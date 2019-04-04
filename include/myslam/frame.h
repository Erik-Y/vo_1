/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"
#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{
    
// forward declare 
class MapPoint;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    cv::Ptr<cv::ORB>               orb_;
    
    std::vector<cv::KeyPoint>      keypoints_;
    vector<double>                 depths_;
    cv::Mat                        descriptors_;
    vector<cv::Point3f>            points_3d_;
    SE3                            T_c_w_;
    
    
    
    unsigned long                  id_;         // id of this frame
    double                         time_stamp_; // when it is recorded
 
    Mat                            color_, depth_; // color and depth image 
    
    int num_of_features_;   // number of features
    double scale_factor_;   // scale in image pyramid
    int level_pyramid_;     // number of pyramid levels
    double depth_scale_ ;
    
public: // data members 
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Mat color=Mat(), Mat depth=Mat() );
    
    ~Frame();
    
    // factory function
    static Frame::Ptr createFrame(); 
    
    void keypoints_detect();
    void descriptor_compute();
    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp);
    //void get_3d_points();

};

}

#endif // FRAME_H
