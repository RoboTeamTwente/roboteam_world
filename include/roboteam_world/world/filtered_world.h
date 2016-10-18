#pragma once

#include <map>
#include <gtest/gtest_prod.h>
#include "ros/ros.h"

#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/DetectionRobot.h"
#include "roboteam_msgs/DetectionBall.h"
#include "roboteam_msgs/World.h"

#include "roboteam_world/robot.h"
#include "roboteam_world/ball.h"

#include "roboteam_world/world/world_base.h"

// TODO: Make sure the us/them nomenclature also propagates to the buffers
// TODO: Make sure the us/them stuff is decided by a parameters settable somewhere

namespace rtt {

    class FilteredWorld : public WorldBase {

    private:
        /**
         * These buffers store for every camera the robots and balls.
         * Accessing goes like this:
         * `robots_blue_buffer[robot_id][camera_id]`
         */
        typedef std::map<int, std::map<int, roboteam_msgs::DetectionRobot>> RobotMultiCamBuffer;
        RobotMultiCamBuffer robots_blue_buffer;
        RobotMultiCamBuffer robots_yellow_buffer;

        roboteam_msgs::DetectionBall ball_buffer;


        std::map<int, rtt::Robot> old_blue, old_yellow;

        // Keeps track which cameras have sent a frame since last world calculation.
        // Also keeps track of which cameras are on-line and sending frames.
        std::map<int, bool> updated_cams;

        /**
         * Final world state being converted to a message when
         * `as_message()` is called.
         */
        std::vector<rtt::Robot> robots_yellow_world;
        std::vector<rtt::Robot> robots_blue_world;
        rtt::Ball ball_world;

    public:
        FilteredWorld();

        /**
        * Resets the world.
        */
        void reset();

        /**
         * Converts this world into a ros message.
         */
        roboteam_msgs::World as_message();

        /**
         * To be called when a detectionframe message is received.
         */
        void detection_callback(const roboteam_msgs::DetectionFrame msg);

    private:

        // Allows for testing of private methods
        FRIEND_TEST(WorldTests, filtered);

        /**
         * Puts a received detection frame in the associated camera's buffer.
         */
        void buffer_detection_frame(const roboteam_msgs::DetectionFrame msg);

        /**
         * Returns true when every camera's frame has updated.
         */
        bool is_calculation_needed();

        /**
         * Merges the frames from all cameras into the final world state.
         */
        void merge_frames();

        void merge_robots(RobotMultiCamBuffer& robots_buffer, std::vector<rtt::Robot>& robots_output, std::map<int, rtt::Robot>& old_buffer);
    };

}
