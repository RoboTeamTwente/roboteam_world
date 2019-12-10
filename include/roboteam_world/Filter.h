#ifndef RTT_FILTER_H
#define RTT_FILTER_H

#include <roboteam_proto/DemoRobot.pb.h>

/**
 * The Filter interface changes a received protobuf message into a new filtered protobuf message (by using all the
 * previous observations and based on these observations it tries to predict the current state).
 * @author Haico Dorenbos
 * @since 2019-12-2
 */
class Filter {
    public:
        /**
         * Filter the protobuf message
         * @param message The current received protobuf message
         * @return The filtered protobuf message
         */
        virtual google::protobuf::Message* filter(google::protobuf::Message* message) = 0;
};


#endif //RTT_FILTER_H
