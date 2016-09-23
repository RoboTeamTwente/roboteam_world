#pragma once

#include <map>

#include "ros/ros.h"

#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/DetectionRobot.h"
#include "roboteam_msgs/DetectionBall.h"
#include "roboteam_msgs/World.h"

#include "../robot.h"
#include "../ball.h"

#include "world_base.h"


namespace rtt {

    class FilteredWorld : public WorldBase {

    private:
        typedef std::map<int, std::vector<roboteam_msgs::DetectionRobot>> RobotMultiCamBuffer;

        /**
         * These buffers store for every camera the robots and balls.
         * Accessing goes like this:
         * `robots_blue_buffer[robot_id][camera_id]`
         */
        RobotMultiCamBuffer robots_blue_buffer;
        RobotMultiCamBuffer robots_yellow_buffer;

        // Keeps track which cameras have sent a frame since last world calculation.
        std::vector<bool> updated_cams;

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
        * Resets the world using the stored configuration.
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

        void merge_robots(RobotMultiCamBuffer* robots_buffer, std::vector<rtt::Robot>* robots_output);
    };

}
