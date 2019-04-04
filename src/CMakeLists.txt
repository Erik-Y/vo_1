add_library( myslam SHARED
    config.cpp
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    
    visual_odometry.cpp
)#compile cpp documents to shared library called myslam

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)
