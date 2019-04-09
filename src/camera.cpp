#include "myslam/camera.h"
#include "myslam/config.h"

namespace myslam
{

Camera::Camera()
{
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
}


Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth )
{
     
    return Vector3d (
        ( p_p ( 0,0 )-cx_ ) *depth/fx_,
        ( p_p ( 1,0 )-cy_ ) *depth/fy_,
        depth
    );
}

Vector2d Camera::camera2pixel( const Vector3d& point_3d  )
{
    return Vector2d(
            point_3d[0]/point_3d[2]*fx_ + cx_,
            point_3d[1]/point_3d[2]*fy_ + cy_
    );
}



}
