% include parameters
parameters;

% initialise animation parameters
bag_file = "";
gif_name = "";
gif_resolution = 400;
create_gif = 0;

body_x = [-body_width / 2, body_width / 2, body_width / 2, -body_width / 2];
body_y = [-body_length / 2, -body_length / 2, body_length / 2, body_length / 2];
body = polyshape(body_x, body_y);
body = rotate(body, -90);
body = translate(body, [wheelbase / 2, 0]);
[body_d, body] = deal(body);

tag_x = [-tag_obstacle_width / 2, tag_obstacle_width / 2, tag_obstacle_width / 2, -tag_obstacle_width / 2];
tag_y = [-tag_obstacle_length / 2, -tag_obstacle_length / 2, tag_obstacle_length / 2, tag_obstacle_length / 2];
[tag_obstacles, tag_obstacles_buffered] = deal(cell(1, number_of_tags));
for i = 1 : number_of_tags
    tag_obstacles{i} = polyshape(tag_x, tag_y);
    tag_obstacles{i} = rotate(tag_obstacles{i}, rad2deg(tags{i}{2}(3)));
    tag_obstacles{i} = translate(tag_obstacles{i}, tags{i}{2}(1 : 2)');
    tag_obstacles_buffered{i} = polybuffer(tag_obstacles{i}, body_width_buffer);
end

goal = polyshape(goal_radius * cos(0 : 0.1 : 2 * pi), goal_radius * sin(0 : 0.1 : 2 * pi));
goal = translate(goal, q_goal(1 : 2)');

E = {};
[controlled_trajectory_x, controlled_trajectory_y, controlled_trajectory_theta, x_history, y_history, theta_history] = deal([]);
x_history(1) = q_initial(1);
y_history(1) = q_initial(2);
theta_history(1) = q_initial(3);
P_history = {};
circle = sqrt(chi2inv(0.99, 2)) * [cos(0 : 0.1 : 2 * pi); sin(0 : 0.1 : 2 * pi)];
q_d_previous = q_initial;
image_out_msg = {};

bag = rosbag(bag_file);

bag_selection = select(bag, "Topic", "/duckiebot/control_node/control_complete");
control_complete_time = bag_selection.MessageList{1, "Time"};

bag_selection = select(bag, "Topic", "/duckiebot/control_node/controlled_trajectory_index");
controlled_trajectory_index_msg_struct = readMessages(bag_selection, "DataFormat", "struct");
controlled_trajectory_index_times = bag_selection.MessageList{:, "Time"};
number_of_controlled_trajectory_index_messages = bag_selection.NumMessages;
controlled_trajectory_index_i = 0;
controlled_trajectory_index_i_changed = false;

bag_selection = select(bag, "Topic", "/duckiebot/planning_node/edges");
edges_msg_struct = readMessages(bag_selection, "DataFormat", "struct");
edges_times = bag_selection.MessageList{:, "Time"};
number_of_edges_messages = bag_selection.NumMessages;
edges_i = 0;
edges_i_changed = false;

bag_selection = select(bag, "Topic", "/duckiebot/apriltag_detector_node/image_out");
image_out_msg_struct = readMessages(bag_selection, "DataFormat", "struct");
image_out_times = bag_selection.MessageList{:, "Time"};
number_of_image_out_messages = bag_selection.NumMessages;
image_out_i = 0;
image_out_i_changed = false;

bag_selection = select(bag, "Topic", "/duckiebot/extended_kalman_filter_node/update");
update_msg_struct = readMessages(bag_selection, "DataFormat", "struct");
update_times = bag_selection.MessageList{:, "Time"};
number_of_update_messages = bag_selection.NumMessages;

bag_selection = select(bag, "Topic", "/duckiebot/planning_node/controlled_trajectory");
controlled_trajectory_msg_struct = readMessages(bag_selection, "DataFormat", "struct");
controlled_trajectory_times = bag_selection.MessageList{:, "Time"};
number_of_controlled_trajectory_messages = bag_selection.NumMessages;
controlled_trajectory_i = 0;
controlled_trajectory_i_changed = false;

bag_selection = select(bag, "Topic", "/duckiebot/planning_node/planning_complete");
planning_complete_time = bag_selection.MessageList{1, "Time"};
planning_complete = false;

for update_i = 1 : number_of_update_messages
    update_time = update_times(update_i);
    if update_time < control_complete_time
        update_msg = update_msg_struct{update_i};
        x_history(end + 1) = update_msg.Data(1);
        y_history(end + 1) = update_msg.Data(2);
        theta_history(end + 1) = update_msg.Data(3);
        P_history{end + 1} = reshape(update_msg.Data(4 : end), 3, 3);
        body = translate(body, [x_history(end) - x_history(end - 1), y_history(end) - y_history(end - 1)]);
        body = rotate(body, rad2deg(theta_history(end) - theta_history(end - 1)), [x_history(end), y_history(end)]);
            
        while controlled_trajectory_i < number_of_controlled_trajectory_messages && controlled_trajectory_times(controlled_trajectory_i + 1) < update_time
            controlled_trajectory_i = controlled_trajectory_i + 1;
            controlled_trajectory_i_changed = true;
        end
        if controlled_trajectory_i_changed
            controlled_trajectory_msg = controlled_trajectory_msg_struct{controlled_trajectory_i};
            controlled_trajectory_size = controlled_trajectory_msg.Data(1);
            controlled_trajectory_x = controlled_trajectory_msg.Data(controlled_trajectory_size + 2 : 2 * controlled_trajectory_size + 1);
            controlled_trajectory_y = controlled_trajectory_msg.Data(2 * controlled_trajectory_size + 2 : 3 * controlled_trajectory_size + 1);
            controlled_trajectory_theta = controlled_trajectory_msg.Data(3 * controlled_trajectory_size + 2 : 4 * controlled_trajectory_size + 1);
            controlled_trajectory_i_changed = false;
        end
        
        while controlled_trajectory_index_i < number_of_controlled_trajectory_index_messages && controlled_trajectory_index_times(controlled_trajectory_index_i + 1) < update_time
            controlled_trajectory_index_i = controlled_trajectory_index_i + 1;
            controlled_trajectory_index_i_changed = true;
        end
        if controlled_trajectory_index_i_changed
            controlled_trajectory_index_msg = controlled_trajectory_index_msg_struct{controlled_trajectory_index_i};
            q_d = [controlled_trajectory_x(controlled_trajectory_index_msg.Data + 1); controlled_trajectory_y(controlled_trajectory_index_msg.Data + 1); controlled_trajectory_theta(controlled_trajectory_index_msg.Data + 1)];
            body_d = translate(body_d, [q_d(1) - q_d_previous(1), q_d(2) - q_d_previous(2)]);
            body_d = rotate(body_d, rad2deg(q_d(3) - q_d_previous(3)), [q_d(1), q_d(2)]);
            q_d_previous = q_d;
            controlled_trajectory_index_i_changed = false;
        end
        
        while edges_i < number_of_edges_messages && edges_times(edges_i + 1) < update_time
            edges_i = edges_i + 1;
            edges_i_changed = true;
        end
        if edges_i_changed
            edges_msg = edges_msg_struct{edges_i};
            E = cell(1, edges_msg.Data(1));
            i = 3;
            for j = 1 : edges_msg.Data(1)
                edge_size = edges_msg.Data(i - 1);
                E{j}{1} = edges_msg.Data(i : i + edge_size - 1);
                E{j}{2} = edges_msg.Data(i + edge_size : i + 2 * edge_size - 1);
                E{j}{3} = edges_msg.Data(i + 2 * edge_size : i + 3 * edge_size - 1);
                i = i + 3 * edge_size + 1;
            end
            edges_i_changed = false;
        end
        
        while image_out_i < number_of_image_out_messages && image_out_times(image_out_i + 1) < update_time
            image_out_i = image_out_i + 1;
            image_out_i_changed = true;
        end
        if image_out_i_changed
            image_out_msg = image_out_msg_struct{image_out_i};
            image_out_i_changed = false;
        end
        
        if planning_complete_time < update_time
            planning_complete = true;
        end
        
        print_figure(planning_complete, E, q_initial, q_goal, goal, tag_obstacles, tag_obstacles_buffered, body_d, body, x_range, y_range, controlled_trajectory_x, controlled_trajectory_y, x_history, y_history, P_history, circle, update_i, image_out_msg, create_gif, gif_name, gif_resolution);
    end
end

% define functions
function print_figure(planning_complete, E, q_initial, q_goal, goal, tag_obstacles, tag_obstacles_buffered, body_d, body, x_range, y_range, controlled_trajectory_x, controlled_trajectory_y, x_history, y_history, P_history, circle, update_i, image_out_msg, create_gif, gif_name, gif_resolution)
    subplot(2, 1, 1);
    cla();
    hold on;
    axis([-0.5, 2, -0.5, 0.5]);
    xlabel("\(x\) (m)", "Interpreter", "latex", "FontSize", 15);
    ylabel("\(y\) (m)", "Interpreter", "latex", "FontSize", 15);
    plot(goal, "FaceColor", "c");
    for i = 1 : size(tag_obstacles, 2)
        plot(tag_obstacles{i}, "FaceColor", "k");
        plot(tag_obstacles_buffered{i}, "FaceColor", "w");
    end
    plot([x_range(1), x_range(2)], [y_range(1), y_range(1)], "k");
    plot([x_range(2), x_range(2)], [y_range(1), y_range(2)], "k");
    plot([x_range(2), x_range(1)], [y_range(2), y_range(2)], "k");
    plot([x_range(1), x_range(1)], [y_range(2), y_range(1)], "k");
    plot(q_initial(1), q_initial(2), "b.", "MarkerSize", 10);
    plot(q_goal(1), q_goal(2), "b.", "MarkerSize", 10);
    if ~planning_complete
        for i = 1 : size(E, 2)
            plot(E{i}{1}, E{i}{2}, "k");
        end
    end
    for i = 1 : size(controlled_trajectory_x, 1) - 1
        plot([controlled_trajectory_x(i), controlled_trajectory_x(i + 1)], [controlled_trajectory_y(i), controlled_trajectory_y(i + 1)], "b", "LineWidth", 1.5);
        %plot(controlled_trajectory_x(i + 1), controlled_trajectory_y(i + 1), "b.", "MarkerSize", 10);
    end
    plot(body_d, "FaceColor", "b");
    plot(body, "FaceColor", "r");
    plot(x_history, y_history, "r", "LineWidth", 1.5);
    if size(P_history, 2)
        ellipse = sqrtm(P_history{end}(1 : 2, 1 : 2)) * circle;
        ellipse = polyshape(ellipse(1, : ), ellipse(2, : ));
        ellipse = translate(ellipse, [x_history(end), y_history(end)]);
        plot(ellipse, "FaceColor", "g", "FaceAlpha", 0.25, "LineWidth", 0.5);
    end
    view();
    subplot(2, 1, 2);
    cla();
    if size(image_out_msg, 1)
        imshow(rosReadImage(image_out_msg));
    else
        imshow(rosReadImage(struct("MessageType", 'sensor_msgs/Image', "Header", struct(), "Height", 480, "Width", 640, "Encoding", 'mono8', "IsBigendian", 0, "Step", 640, "Data", ones(307200, 1, "uint8") * 254)));
    end
    if create_gif
        if update_i == 1
            exportgraphics(gcf, pwd + "/" + gif_name + ".gif", "Resolution", gif_resolution);
        else
            exportgraphics(gcf, pwd + "/" + gif_name + ".gif", "Resolution", gif_resolution, "Append", true);
        end
    end
end
