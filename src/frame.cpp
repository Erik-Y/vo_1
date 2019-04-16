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

#include "myslam/frame.h"
#include "myslam/config.h"


namespace myslam
{
Frame::Frame()
: id_(-1), time_stamp_(-1)
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    
    orb_=cv::ORB::create( num_of_features_, scale_factor_, level_pyramid_ );
    depth_scale_ = Config::get<float>("camera.depth_scale");
}
Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Mat color, Mat depth )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), color_(color), depth_(depth)
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    
    orb_=cv::ORB::create( num_of_features_, scale_factor_, level_pyramid_ );
    depth_scale_ = Config::get<float>("camera.depth_scale");
}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return  make_shared<myslam::Frame>(Frame(factory_id++)) ;
}

void Frame::keypoints_detect()
{
    vector<cv::KeyPoint> keypoints_raw;
    orb_->detect(color_, keypoints_raw );
    
    for( auto keypoint: keypoints_raw )
    {
        
        double d = findDepth( keypoint );
        if( d >= 0 )
        {
            
            keypoints_.push_back(keypoint);
            depths_.push_back(d);
        }
    }
    
}
void Frame::descriptor_compute()

{   //orb_->compute(color_, keypoints_raw, descriptors_raw );
    orb_->compute(color_, keypoints_, descriptors_ );
}

double Frame::findDepth ( const cv::KeyPoint& kp )//接受一个特征点，返回深度信息
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];
    if ( d!=0 )
    {
        return double(d)/depth_scale_;
    }
    else 
    {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/depth_scale_;
            }
           
        }
    }
    return -1.0;
}



}
