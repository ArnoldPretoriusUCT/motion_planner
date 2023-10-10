// include standard C header files
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
class ExtendedKalmanFilterNode {
    // declare standard attributes
    bool apriltag_measurement_called;
    double bearing, delta_x, delta_x_times_estimated_range_inverse, delta_y, delta_y_times_estimated_range_inverse, estimated_range, estimated_range_inverse, range;
    std::vector<double> landmark_detector_noise_standard_deviations, q_tag;
    std::vector<std::vector<double>> H_q, K, P, q, W, X;
    
    // declare ROS attributes
    ros::Publisher pub_update;
    ros::Subscriber sub_apriltag_measurement, sub_initialise, sub_prediction;
    std_msgs::Float64MultiArray msg_update;
    
    public:
        // define node constructor
        ExtendedKalmanFilterNode(ros::NodeHandle* nh) {
            // initialise standard attributes
            apriltag_measurement_called = false;

            // initialise ROS attributes
            pub_update = nh->advertise<std_msgs::Float64MultiArray>("update", 1);
            sub_apriltag_measurement = nh->subscribe("apriltag_measurement", 1, &ExtendedKalmanFilterNode::apriltag_measurement_callback, this);
            sub_initialise = nh->subscribe<std_msgs::Bool>("initialise", 1, boost::bind(&ExtendedKalmanFilterNode::initialise_callback, this, _1, nh));
            sub_prediction = nh->subscribe("prediction", 1, &ExtendedKalmanFilterNode::prediction_callback, this);
        }
        
        // define callback methods
        void apriltag_measurement_callback(const std_msgs::Float64MultiArrayConstPtr& msg_apriltag_measurement) {
            range = msg_apriltag_measurement->data[0];
            bearing = msg_apriltag_measurement->data[1];
            q_tag = {msg_apriltag_measurement->data[2], msg_apriltag_measurement->data[3], msg_apriltag_measurement->data[4]};
            apriltag_measurement_called = true;
        }

        void initialise_callback(const std_msgs::BoolConstPtr& msg_initialise, ros::NodeHandle* nh) {
            // get parameter values
            nh->getParam("landmark_detector_noise_standard_deviations", landmark_detector_noise_standard_deviations);

            // initialise standard attributes with parameter values
            W = {{std::pow(landmark_detector_noise_standard_deviations[0], 2), 0}, {0, std::pow(landmark_detector_noise_standard_deviations[1] * M_PI / 180, 2)}};
        }

        void prediction_callback(const std_msgs::Float64MultiArrayConstPtr& msg_prediction) {
            q = {{msg_prediction->data[0]}, {msg_prediction->data[1]}, {msg_prediction->data[2]}};
            P = {{msg_prediction->data[3], msg_prediction->data[4], msg_prediction->data[5]}, {msg_prediction->data[6], msg_prediction->data[7], msg_prediction->data[8]}, {msg_prediction->data[9], msg_prediction->data[10], msg_prediction->data[11]}};
            if (apriltag_measurement_called) {
                delta_x = q_tag[0] - q[0][0];
                delta_y = q_tag[1] - q[1][0];
                estimated_range = std::sqrt(delta_x * delta_x + delta_y * delta_y);
                estimated_range_inverse = 1 / estimated_range;
                delta_x_times_estimated_range_inverse = delta_x * estimated_range_inverse;
                delta_y_times_estimated_range_inverse = delta_y * estimated_range_inverse;
                H_q = {{-delta_x_times_estimated_range_inverse, -delta_y_times_estimated_range_inverse, 0}, {delta_y_times_estimated_range_inverse * estimated_range_inverse, -delta_x_times_estimated_range_inverse * estimated_range_inverse, -1}};
                X = matrix_multiplication(P, matrix_transposition(H_q));
                K = matrix_multiplication(X, two_by_two_matrix_inversion(matrix_addition(matrix_multiplication(H_q, X), W)));
                q = matrix_addition(q, matrix_multiplication(K, {{range - estimated_range}, {bearing - std::atan2(delta_y, delta_x) + q[2][0]}}));
                P = matrix_subtraction(P, matrix_multiplication(matrix_multiplication(K, H_q), P));
                apriltag_measurement_called = false;
            }
            msg_update.data = {q[0][0], q[1][0], q[2][0], P[0][0], P[0][1], P[0][2], P[1][0], P[1][1], P[1][2], P[2][0], P[2][1], P[2][2]};
            pub_update.publish(msg_update);
        }
};

// define main function
int main(int argc, char** argv) {
    // initialise node
    ros::init(argc, argv, "extended_kalman_filter_node");
    ros::NodeHandle nh("~");
    ExtendedKalmanFilterNode extended_kalman_filter_node = ExtendedKalmanFilterNode(&nh);
    ros::spin();
    
    return 0;
}
