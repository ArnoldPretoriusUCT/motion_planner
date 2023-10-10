// include standard C header files
#include <duckietown_msgs/WheelEncoderStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

// include standard C++ header files
#include <boost/bind.hpp>
#include <cmath>
#include <vector>

// include user defined C++ header files
#include "math.hpp"

// define node class
class OdometryNode {
    // declare standard attributes
    bool encoders_called, initialised;
    int left_ticks_previous, right_ticks_previous;
    double cosine, distance, left_distance, metres_per_tick, right_distance, sine, ticks_per_revolution, track, track_inverse, wheel_radius;
    std::vector<double> configuration_noise_standard_deviations, odometry_noise_standard_deviations, q_initial;
    std::vector<std::vector<double>> F_noise, F_q, P, q, V;
    
    // declare ROS attributes
    ros::Publisher pub_prediction;
    ros::Subscriber sub_initialise, sub_update;
    std_msgs::Float64MultiArray msg_prediction;
    
    public:
        // define node constructor
        OdometryNode(ros::NodeHandle* nh, message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<duckietown_msgs::WheelEncoderStamped, duckietown_msgs::WheelEncoderStamped>>* sync) {
            // initialise standard attributes
            encoders_called = false;
            initialised = false;
            
            // initialise ROS attributes
            pub_prediction = nh->advertise<std_msgs::Float64MultiArray>("prediction", 1);
            sub_initialise = nh->subscribe<std_msgs::Bool>("initialise", 1, boost::bind(&OdometryNode::initialise_callback, this, _1, nh, sync));
            sub_update = nh->subscribe("update", 1, &OdometryNode::update_callback, this);
            sync->registerCallback(boost::bind(&OdometryNode::time_synchronised_encoders_callback, this, _1, _2));
        }

        // define callback methods
        void initialise_callback(const std_msgs::BoolConstPtr& msg_initialise, ros::NodeHandle* nh, message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<duckietown_msgs::WheelEncoderStamped, duckietown_msgs::WheelEncoderStamped>>* sync) {
            // get parameter values
            nh->getParam("configuration_noise_standard_deviations", configuration_noise_standard_deviations);
            nh->getParam("odometry_noise_standard_deviations", odometry_noise_standard_deviations);
            nh->getParam("q_initial", q_initial);
            nh->getParam("ticks_per_revolution", ticks_per_revolution);
            nh->getParam("track", track);
            nh->getParam("wheel_radius", wheel_radius);

            // initialise standard attributes with parameter values
            metres_per_tick = 2 * M_PI * wheel_radius / ticks_per_revolution;
            track_inverse = 1 / track;
            q = {{q_initial[0]}, {q_initial[1]}, {q_initial[2]}};
            P = {{std::pow(configuration_noise_standard_deviations[0], 2), 0, 0}, {0, std::pow(configuration_noise_standard_deviations[1], 2), 0}, {0, 0, std::pow(configuration_noise_standard_deviations[2], 2)}};
            V = {{std::pow(odometry_noise_standard_deviations[0], 2), 0}, {0, std::pow(odometry_noise_standard_deviations[1] * M_PI / 180, 2)}};
            initialised = true;
        }

        void update_callback(const std_msgs::Float64MultiArrayConstPtr& msg_update) {
            q = {{msg_update->data[0]}, {msg_update->data[1]}, {msg_update->data[2]}};
            P = {{msg_update->data[3], msg_update->data[4], msg_update->data[5]}, {msg_update->data[6], msg_update->data[7], msg_update->data[8]}, {msg_update->data[9], msg_update->data[10], msg_update->data[11]}};
        }

        void time_synchronised_encoders_callback(const duckietown_msgs::WheelEncoderStampedConstPtr& msg_left_ticks, const duckietown_msgs::WheelEncoderStampedConstPtr& msg_right_ticks) {
            if (initialised && encoders_called) {
                if (msg_left_ticks->data != left_ticks_previous || msg_right_ticks->data != right_ticks_previous) {
                    left_distance = (msg_left_ticks->data - left_ticks_previous) * metres_per_tick;
                    right_distance = (msg_right_ticks->data - right_ticks_previous) * metres_per_tick;
                    distance = (left_distance + right_distance) * 0.5;
                    cosine = std::cos(q[2][0]);
                    sine = std::sin(q[2][0]);
                    F_q = {{1, 0, -distance * sine}, {0, 1, distance * cosine}, {0, 0, 1}};
                    F_noise = {{cosine, 0}, {sine, 0}, {0, 1}};
                    q[0][0] += F_q[1][2];
                    q[1][0] += -F_q[0][2];
                    q[2][0] += (right_distance - left_distance) * track_inverse;
                    P = matrix_addition(matrix_multiplication(matrix_multiplication(F_q, P), matrix_transposition(F_q)), matrix_multiplication(matrix_multiplication(F_noise, V), matrix_transposition(F_noise)));
                }
                msg_prediction.data = {q[0][0], q[1][0], q[2][0], P[0][0], P[0][1], P[0][2], P[1][0], P[1][1], P[1][2], P[2][0], P[2][1], P[2][2]};
                pub_prediction.publish(msg_prediction);
            }
            else {
                encoders_called = true;
            }
            left_ticks_previous = msg_left_ticks->data;
            right_ticks_previous = msg_right_ticks->data;
        }
};

// define main function
int main(int argc, char** argv) {
    // initialise node
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh("~");
    message_filters::Subscriber<duckietown_msgs::WheelEncoderStamped> sub_left_ticks(nh, "left_ticks", 1);
    message_filters::Subscriber<duckietown_msgs::WheelEncoderStamped> sub_right_ticks(nh, "right_ticks", 1);
    typedef message_filters::sync_policies::ApproximateTime<duckietown_msgs::WheelEncoderStamped, duckietown_msgs::WheelEncoderStamped> sync_policy;
    message_filters::Synchronizer<sync_policy> sync(sync_policy(10), sub_left_ticks, sub_right_ticks);
    OdometryNode odometry_node = OdometryNode(&nh, &sync);
    ros::spin();
    
    return 0;
}
