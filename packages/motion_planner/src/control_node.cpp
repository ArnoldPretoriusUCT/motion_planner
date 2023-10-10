// include standard C header files
#include <duckietown_msgs/WheelsCmdStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64.h>

// include standard C++ header files
#include <boost/bind.hpp>
#include <cmath>
#include <vector>

// include user defined C++ header files
#include "math.hpp"

// define node class
class ControlNode {
    // declare standard attributes
    bool control_complete, planning_complete, timer_created;
    int controlled_trajectory_index, controlled_trajectory_size;
    double abs_u_v_d, cosine, cosine_theta_error, delta, motor_constant, motor_constant_times_wheel_radius_inverse, maximum_curvature, phi_max_degrees_control, range_min, sine, tangent_theta_error, theta_dot_d, theta_error, time_step, tof_range, u_omega_bounded, u_omega_max, u_v_bounded, u_v_max, u_v_min, wheel_distance, wheel_radius, wheelbase, wheelbase_inverse, x_error, x_error_spatial, y_error, y_error_spatial;
    std::vector<double> controlled_trajectory_times, u_d, q_d, gains;
    std::vector<std::vector<double>> controlled_trajectory;
    
    // declare ROS attributes
    ros::Publisher pub_control_complete, pub_controlled_trajectory_index, pub_wheels_cmd;
    ros::Subscriber sub_complete, sub_controlled_trajectory, sub_q_d, sub_initialise, sub_planning_complete, sub_update, sub_tof_range;
    ros::Timer timer;
    std_msgs::Bool msg_control_complete;
    std_msgs::Int64 msg_controlled_trajectory_index;
    duckietown_msgs::WheelsCmdStamped msg_wheels_cmd, msg_wheels_cmd_previous;
    
    public:
        // define node constructor
        ControlNode(ros::NodeHandle* nh) {
            // initialise standard attributes
            control_complete = false;
            planning_complete = false;
            timer_created = false;
            controlled_trajectory_index = 0;
            controlled_trajectory_size = 0;

            // initialise ROS attributes
            pub_control_complete = nh->advertise<std_msgs::Bool>("control_complete", 1);
            pub_controlled_trajectory_index = nh->advertise<std_msgs::Int64>("controlled_trajectory_index", 1);
            pub_wheels_cmd = nh->advertise<duckietown_msgs::WheelsCmdStamped>("wheels_cmd", 1);
            sub_initialise = nh->subscribe<std_msgs::Bool>("initialise", 1, boost::bind(&ControlNode::initialise_callback, this, _1, nh));
            sub_update = nh->subscribe("update", 1, &ControlNode::update_callback, this);
            sub_planning_complete = nh->subscribe("planning_complete", 1, &ControlNode::planning_complete_callback, this);
            sub_tof_range = nh->subscribe("tof_range", 1, &ControlNode::tof_range_callback, this);
            sub_controlled_trajectory = nh->subscribe<std_msgs::Float64MultiArray>("controlled_trajectory", 1, boost::bind(&ControlNode::controlled_trajectory_callback, this, _1, nh));
        }

        // define callback methods
        void initialise_callback(const std_msgs::BoolConstPtr& msg_initialise, ros::NodeHandle* nh) {
            // get parameter values
            nh->getParam("gains", gains);
            nh->getParam("motor_constant", motor_constant);
            nh->getParam("phi_max_degrees_control", phi_max_degrees_control);
            nh->getParam("range_min", range_min);
            nh->getParam("u_v_max", u_v_max);
            nh->getParam("u_v_min", u_v_min);
            nh->getParam("wheel_distance", wheel_distance);
            nh->getParam("wheel_radius", wheel_radius);
            nh->getParam("wheelbase", wheelbase);
            
            // initialise standard attributes with parameter values
            motor_constant_times_wheel_radius_inverse = 1 / (motor_constant * wheel_radius);
            wheelbase_inverse = 1 / wheelbase;
            maximum_curvature = std::tan(phi_max_degrees_control * M_PI / 180) * wheelbase_inverse;
        }

        void update_callback(const std_msgs::Float64MultiArrayConstPtr& msg_update) {
            if (controlled_trajectory_index) {
                if (tof_range < range_min || control_complete) {
                    msg_wheels_cmd.vel_left = 0;
                    msg_wheels_cmd.vel_right = 0;
                }
                else {
                    x_error_spatial = msg_update->data[0] - q_d[0];
                    y_error_spatial = msg_update->data[1] - q_d[1];
                    x_error = x_error_spatial * cosine + y_error_spatial * sine;
                    y_error = -x_error_spatial * sine + y_error_spatial * cosine;
                    theta_error = msg_update->data[2] - q_d[2];
                    cosine_theta_error = std::cos(theta_error);
                    tangent_theta_error = std::tan(theta_error);
                    u_v_bounded = std::max(std::min((u_d[0] - gains[0] * abs_u_v_d * (x_error + y_error * tangent_theta_error)) / cosine_theta_error, u_v_max), u_v_min);
                    u_omega_max = std::abs(u_v_bounded) * maximum_curvature;
                    u_omega_bounded = std::max(std::min(theta_dot_d - (gains[1] * u_d[0] * y_error + gains[2] * abs_u_v_d * tangent_theta_error) * cosine_theta_error * cosine_theta_error, u_omega_max), -u_omega_max);
                    delta = u_omega_bounded * wheel_distance;
                    msg_wheels_cmd.vel_left = gains[3] * (u_v_bounded - delta) * motor_constant_times_wheel_radius_inverse;
                    msg_wheels_cmd.vel_right = gains[4] * (u_v_bounded + delta) * motor_constant_times_wheel_radius_inverse;
                }
                pub_wheels_cmd.publish(msg_wheels_cmd);
                if (planning_complete && controlled_trajectory_index == controlled_trajectory_size && !control_complete) {
                    msg_wheels_cmd_previous = msg_wheels_cmd;
                    if (equal(msg_wheels_cmd.vel_left, msg_wheels_cmd_previous.vel_left) && equal(msg_wheels_cmd.vel_right, msg_wheels_cmd_previous.vel_right)) {
                        control_complete = true;
                        msg_control_complete.data = control_complete;
                        pub_control_complete.publish(msg_control_complete);
                        ROS_INFO("control complete");
                    }
                }
            }
        }
        
        void timer_callback(const ros::TimerEvent& event) {
            timer.stop();
            if (controlled_trajectory_index < controlled_trajectory_size) {
                msg_controlled_trajectory_index.data = controlled_trajectory_index;
                pub_controlled_trajectory_index.publish(msg_controlled_trajectory_index);
                q_d = {controlled_trajectory[0][controlled_trajectory_index], controlled_trajectory[1][controlled_trajectory_index], controlled_trajectory[2][controlled_trajectory_index]};
                cosine = std::cos(q_d[2]);
                sine = std::sin(q_d[2]);
                u_d = {controlled_trajectory[3][controlled_trajectory_index], controlled_trajectory[4][controlled_trajectory_index]};
                abs_u_v_d = std::abs(u_d[0]);
                theta_dot_d = u_d[0] * std::tan(u_d[1]) * wheelbase_inverse;
                controlled_trajectory_index++;
                timer.setPeriod(ros::Duration(controlled_trajectory_times[controlled_trajectory_index]));
                timer.start();
            }
        }

        void planning_complete_callback(const std_msgs::BoolConstPtr& msg_planning_complete) {
            planning_complete = msg_planning_complete->data;
        }

        void tof_range_callback(const sensor_msgs::RangeConstPtr& msg_tof_range) {
            tof_range = msg_tof_range->range;
        }

        void controlled_trajectory_callback(const std_msgs::Float64MultiArrayConstPtr& msg_controlled_trajectory, ros::NodeHandle* nh) {
            controlled_trajectory_size = msg_controlled_trajectory->data[0];
            controlled_trajectory_times = {msg_controlled_trajectory->data.begin() + 1, msg_controlled_trajectory->data.begin() + controlled_trajectory_size + 1};
            controlled_trajectory = {{msg_controlled_trajectory->data.begin() + controlled_trajectory_size + 1, msg_controlled_trajectory->data.begin() + 2 * controlled_trajectory_size + 1}, {msg_controlled_trajectory->data.begin() + 2 * controlled_trajectory_size + 1, msg_controlled_trajectory->data.begin() + 3 * controlled_trajectory_size + 1}, {msg_controlled_trajectory->data.begin() + 3 * controlled_trajectory_size + 1, msg_controlled_trajectory->data.begin() + 4 * controlled_trajectory_size + 1}, {msg_controlled_trajectory->data.begin() + 4 * controlled_trajectory_size + 1, msg_controlled_trajectory->data.begin() + 5 * controlled_trajectory_size + 1}, {msg_controlled_trajectory->data.begin() + 5 * controlled_trajectory_size + 1, msg_controlled_trajectory->data.begin() + 6 * controlled_trajectory_size + 1}};
            if (!timer_created) {
                timer = nh->createTimer(ros::Duration(controlled_trajectory_times[0]), &ControlNode::timer_callback, this, true);
                timer_created = true;
            }
        }
};

// define main function
int main(int argc, char** argv) {
    // initialise node
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh("~");
    ControlNode control_node = ControlNode(&nh);
    ros::spin();

    return 0;
}
