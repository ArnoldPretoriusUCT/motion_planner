% test number
test_number = 1;

% initialise robot parameters
wheelbase = 0.15;
track = 0.106;
wheel_distance = track / 2;
wheel_radius = 0.033;
body_length = wheelbase + 2 * wheel_radius;
body_width = track + 0.025;
body_width_buffer = 0.75 * body_width;
body_length_buffer = 1.01 * (body_length - (body_length - wheelbase) / 2);

% initialise AprilTag parameters
tag_size = 0.065;
tag_obstacle_length = 0.08;
tag_obstacle_width = 0.054;
if test_number == 1
    tags = {{20, [1.3; 0; pi]}};
elseif test_number == 2
    tags = {{2, [0.4; 0; pi]}, {20, [1.3; 0; pi]}};
else
    tags = {{2, [0.4; 0; pi]}, {23, [0.8; 0.2; pi]}, {7, [0.8; -0.2; pi]}, {20, [1.3; 0; pi]}};
end
number_of_tags = size(tags, 2);
tag_list = zeros(1, number_of_tags * 4);
for i = 1 : number_of_tags
    tag_list(4 * i - 3 : 4 * i) = [tags{i}{1}, tags{i}{2}(:)'];
end
tag_vector = cell(1, number_of_tags * 4);
for i = 1 : number_of_tags * 4
    tag_vector{i} = tag_list(i);
end

% initialise map parameters
q_initial = [0; 0; 0];
q_goal = [1; 0; 0];
x_range = [q_initial(1), tags{end}{2}(1) - tag_obstacle_width / 2 - body_length_buffer];
y_range = [-0.2, 0.2];

% initialise goal parameters
distances = zeros(1, number_of_tags + 4);
for i = 1 : number_of_tags
    distances(i) = max(norm(q_goal(1 : 2)' - tags{i}{2}(1 : 2)') - sqrt((tag_obstacle_length / 2) ^ 2 + (tag_obstacle_width / 2) ^ 2) - body_length_buffer, 0);
end
distances(number_of_tags + 1) = abs(q_goal(1) - x_range(1));
distances(number_of_tags + 2) = abs(q_goal(1) - x_range(2));
distances(number_of_tags + 3) = abs(q_goal(2) - y_range(1));
distances(number_of_tags + 4) = abs(q_goal(2) - y_range(2));
goal_radius_max = 0.1;
goal_radius = min(min(distances), goal_radius_max);

% initialise planning parameters
u_v = 0.05;
steered_distance = 0.1;
number_of_steps = 5;
phi_max_degrees_planning = 25;
root_index = 1;
biased_step = 10;
number_of_samples = 8;
if test_number == 1
    Q_free_area = (x_range(2) - x_range(1)) * (y_range(2) - y_range(1));
elseif test_number == 2
    Q_free_area = (x_range(2) - x_range(1)) * (y_range(2) - y_range(1)) - (tag_obstacle_length * tag_obstacle_width + 2 * tag_obstacle_length * body_width_buffer + 2 * tag_obstacle_width * body_width_buffer + pi * body_width_buffer ^ 2);
else
    Q_free_area = (x_range(2) - x_range(1)) * (y_range(2) - y_range(1)) - 2 * (tag_obstacle_length * tag_obstacle_width + 2 * tag_obstacle_length * body_width_buffer + 2 * tag_obstacle_width * body_width_buffer + pi * body_width_buffer ^ 2);
end
gamma = 2 * sqrt(1.5 * Q_free_area / pi);

% initialise control parameters
u_v_max = 0.5;
u_v_min = 0;
phi_max_degrees_control = 75;
gains = {150, 500, 500, 1, 1};
motor_constant = 27;
range_min = 0.1;

% initialise perception parameters
ticks_per_revolution = 135;

% initialise localisation parameters
configuration_noise_standard_deviations = {0.005, 0.005, 0.001};
odometry_noise_standard_deviations = {0.01, 0.5};
landmark_detector_noise_standard_deviations = {0.1, 5};
