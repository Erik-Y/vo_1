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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

namespace myslam
{

VisualOdometry::VisualOdometry(): 
     state_ ( INITIALIZING ), frame_refe_( nullptr ), frame_curr_( nullptr ), num_lost_ ( 0 ), num_inliers_ ( 0 ),camera_(nullptr)
{
   
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
}

VisualOdometry::~VisualOdometry()
{

}

bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case INITIALIZING:
    {
        //将当前帧和参考帧都置为刚输入进来的帧，把当前帧插入到关键帧容器中，将3d特征点以及对应的描述子放到参考pts_3d_ref_，descriptors_ref_
        state_ = OK;
        frame_curr_ = frame;//至当前帧为该帧
         
        frame_curr_->keypoints_detect();//将特征点提取到
        
        
        frame_curr_ ->descriptor_compute();//将描述子保存到
        
        get_3d_points();
        frame_refe_ = frame_curr_;
        
        
       
        break;
    }
    case OK:
    {
        static int flag = 0;
        
        frame_curr_ = frame;//至当前帧为该帧
        frame_curr_->keypoints_detect();//将特征点提取到
        frame_curr_ ->descriptor_compute();//将描述子保存到
        get_3d_points();
        features_match();//匹配当前描述子descriptors_和参考描述子descriptors_ref_，匹配结果保存在curr_feature_matches_.
        
        poseEstimationPnP();//计算SE3形式的变换矩阵，保存于T_c_r_estimated_，正确点个数保存于num_inliers
        frame_curr_->T_c_w_ = T_c_r_estimated_ * frame_refe_->T_c_w_;  // T_c_w = T_c_r*T_r_w 
        frame_refe_ = frame_curr_;
        
        num_lost_ = 0;
        break;
    }


    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
}


void VisualOdometry::features_match()
{
    //use OpenCV's brute force match 
    vector<cv::DMatch> matches_raw;
    cv::BFMatcher matcher ( cv::NORM_HAMMING );
    matcher.match ( frame_refe_->descriptors_, frame_curr_->descriptors_, matches_raw );
    // select the best matches
    float min_dis = std::min_element (
                        matches_raw.begin(), matches_raw.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    matches_.clear();
    for ( unsigned i = 0; i != matches_raw.size(); ++i )
    {
        if ( matches_raw[i].distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            matches_.push_back(matches_raw[i]);
        }
    }
    cout<<"good matches: "<<matches_.size()<<endl;
}



void VisualOdometry::poseEstimationPnP()//计算SE3形式的变换矩阵，保存于T_c_r_estimated_，正确点个数保存于num_inliers
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    
    for ( auto m:matches_ )
    {
        pts3d.push_back( frame_refe_->points_3d_[m.queryIdx] );
        pts2d.push_back( frame_curr_->keypoints_[m.trainIdx].pt );
    }
    
    Mat K = ( cv::Mat_<double>(3,3)<<
        camera_->fx_, 0, camera_->cx_,
        0, camera_->fy_, camera_->cy_,
        0,0,1
    );
    
    Mat rvec, tvec, inliers;
    //随机采样一致性求解PNP，其中inliers是正确数据的容器，每一行包含一个正确点的信息，用于判断计算结果是否可取
    cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    
    
    
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_r_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
}

void VisualOdometry::get_3d_points()
{
    for( unsigned i = 0; i != frame_curr_->keypoints_.size(); ++i )
    {   
        
        Eigen::Vector3d p(camera_->pixel2camera( Eigen::Vector2d(frame_curr_->keypoints_[i].pt.x, frame_curr_->keypoints_[i].pt.y), frame_curr_->depths_[i] ));
        
      
        frame_curr_->points_3d_.push_back(cv::Point3f( p(0,0), p(1,0), p(2,0) ) );
    }
}



}
