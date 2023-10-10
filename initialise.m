% include parameters
parameters;

% initialise ROS
rosinit("duckiebot.local");

% set parameter values
set(rosparam, "/duckiebot/apriltag_detector_node/tag_vector", tag_vector);
set(rosparam, "/duckiebot/apriltag_detector_node/tag_size", tag_size);
set(rosparam, "/duckiebot/control_node/gains", gains);
set(rosparam, "/duckiebot/control_node/motor_constant", motor_constant);
set(rosparam, "/duckiebot/control_node/phi_max_degrees_control", phi_max_degrees_control);
set(rosparam, "/duckiebot/control_node/range_min", range_min);
set(rosparam, "/duckiebot/control_node/u_v_max", u_v_max);
set(rosparam, "/duckiebot/control_node/u_v_min", u_v_min);
set(rosparam, "/duckiebot/control_node/wheel_distance", wheel_distance);
set(rosparam, "/duckiebot/control_node/wheel_radius", wheel_radius);
set(rosparam, "/duckiebot/control_node/wheelbase", wheelbase);
set(rosparam, "/duckiebot/extended_kalman_filter_node/landmark_detector_noise_standard_deviations", landmark_detector_noise_standard_deviations);
set(rosparam, "/duckiebot/odometry_node/configuration_noise_standard_deviations", configuration_noise_standard_deviations);
set(rosparam, "/duckiebot/odometry_node/odometry_noise_standard_deviations", odometry_noise_standard_deviations);
set(rosparam, "/duckiebot/odometry_node/q_initial", {q_initial(1), q_initial(2), q_initial(3)});
set(rosparam, "/duckiebot/odometry_node/ticks_per_revolution", ticks_per_revolution);
set(rosparam, "/duckiebot/odometry_node/track", track);
set(rosparam, "/duckiebot/odometry_node/wheel_radius", wheel_radius);
set(rosparam, "/duckiebot/planning_node/biased_step", biased_step);
set(rosparam, "/duckiebot/planning_node/buffer", body_width_buffer);
set(rosparam, "/duckiebot/planning_node/gamma", gamma);
set(rosparam, "/duckiebot/planning_node/goal_radius", goal_radius);
set(rosparam, "/duckiebot/planning_node/number_of_samples", number_of_samples);
set(rosparam, "/duckiebot/planning_node/number_of_steps", number_of_steps);
set(rosparam, "/duckiebot/planning_node/phi_max_degrees_planning", phi_max_degrees_planning);
set(rosparam, "/duckiebot/planning_node/root_index", root_index);
set(rosparam, "/duckiebot/planning_node/q_goal", {q_goal(1), q_goal(2), q_goal(3)});
set(rosparam, "/duckiebot/planning_node/q_initial", {q_initial(1), q_initial(2), q_initial(3)});
set(rosparam, "/duckiebot/planning_node/steered_distance", steered_distance);
set(rosparam, "/duckiebot/planning_node/tag_obstacle_length", tag_obstacle_length);
set(rosparam, "/duckiebot/planning_node/tag_obstacle_width", tag_obstacle_width);
set(rosparam, "/duckiebot/planning_node/tag_vector", tag_vector);
set(rosparam, "/duckiebot/planning_node/u_v", u_v);
set(rosparam, "/duckiebot/planning_node/wheelbase", wheelbase);
set(rosparam, "/duckiebot/planning_node/x_range", {x_range(1), x_range(2)});
set(rosparam, "/duckiebot/planning_node/y_range", {y_range(1), y_range(2)});

% initialise nodes
topic_name = "/duckiebot/matlab/initialise";
disp("Connecting to " + topic_name);
pub_initialise = rospublisher(topic_name, "std_msgs/Bool");
disp("Connected");
msg_initialise = rosmessage(pub_initialise);
msg_initialise.Data = 1;
send(pub_initialise, msg_initialise);
