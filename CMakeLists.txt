cmake_minimum_required(VERSION 3.10)
project(roboteam_world)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(Eigen3 REQUIRED)

add_library(net STATIC
        src/net/netraw.cpp
        src/net/robocup_ssl_client.cpp
        )

target_link_libraries(net
        PUBLIC roboteam_proto)

add_library(worldUtils STATIC
        include/roboteam_world/filters/KalmanFilter.h
        include/roboteam_world/filters/WorldFilter.h
        src/filters/WorldFilter.cpp
        include/roboteam_world/filters/RobotFilter.h
        src/filters/RobotFilter.cpp
        include/roboteam_world/filters/BallFilter.h
        src/filters/BallFilter.cpp
        include/roboteam_world/filters/CameraFilter.h
        src/filters/CameraFilter.cpp
        src/filters/Scaling.h
        include/roboteam_world/data/RobotObservation.h
        include/roboteam_world/data/BallObservation.h
        src/data/Camera.cpp
        include/roboteam_world/data/Camera.h
        src/ball/solvers/RotatedLinearChip6D.cpp
        include/roboteam_world/ball/solvers/RotatedLinearChip6D.h
        src/ball/solvers/LinearChip6D.cpp
        include/roboteam_world/ball/solvers/LinearChip6D.h
        src/ball/solvers/LinearChip5D.cpp include/roboteam_world/ball/solvers/LinearChip5D.h src/ball/solvers/LinearChip3D.cpp include/roboteam_world/ball/solvers/LinearChip3D.h src/ball/solvers/BouncePoint.cpp src/ball/solvers/BouncePoint.h)
target_link_libraries(worldUtils
        PUBLIC roboteam_proto
        PUBLIC Eigen3::Eigen
        )
target_include_directories(worldUtils
        INTERFACE include
        PRIVATE include/roboteam_world
        PRIVATE src
)
add_executable(roboteam_world
        src/main.cpp
        src/WorldHandler.cpp
        )
set_property(TARGET roboteam_world PROPERTY CXX_STANDARD 17)
set_target_properties(worldUtils PROPERTIES COMPILE_FLAGS "-O0")
set_target_properties(roboteam_world PROPERTIES COMPILE_FLAGS "-O0")


target_include_directories(roboteam_world
        INTERFACE include
        PRIVATE include/roboteam_world
        PRIVATE src
        )


target_link_libraries(roboteam_world
        PRIVATE net
        PRIVATE roboteam_utils
        PRIVATE worldUtils
        PUBLIC roboteam_proto)

add_executable(world_tests
        include/roboteam_world/filters/KalmanFilter.h
        test/main.cpp
        test/KalmanFilterTest.cpp
        )


target_include_directories(world_tests
        INTERFACE include
        PRIVATE include/roboteam_world
        PRIVATE src
        )



if (UNIX AND NOT APPLE)
    target_link_libraries(world_tests
            PRIVATE net
            PRIVATE roboteam_utils
            PRIVATE gtest
            PUBLIC roboteam_proto)
endif ()

if (APPLE)
    target_link_libraries(world_tests
            PRIVATE net
            PRIVATE roboteam_utils
            PUBLIC roboteam_proto
            /usr/local/lib/libgtest.a
            /usr/local/lib/libgtest_main.a)
endif ()
