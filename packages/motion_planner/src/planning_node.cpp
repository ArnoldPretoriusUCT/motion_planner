// include standard C header files
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64.h>

// include standard C++ header files
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>
#include <tuple>

// include user defined C++ header files
#include "math.hpp"

// define node class
class PlanningNode {
    // declare standard attributes
    bool new_controlled_trajectory, planning_complete;
    int biased_step, committed_trajectory_size, controlled_trajectory_size, counter, i, index, j, number_of_path_types, number_of_samples, number_of_steps, number_of_tags, Q_near_size, root_index, successes, U_phi_size, uncommitted_trajectory_size, V_size;
    double body_length, buffer, circumference, connection_time, connection_time_min, cost, cost_min, distance_step, distance_step_times_wheelbase_inverse, gamma, goal_radius, M_3PI_2, M_2PI, phi_max, phi_max_degrees_planning, Q_radius, semicircumference, steered_distance, steered_time, steps_per_second, tag_obstacle_length, tag_obstacle_width, tag_size, tic, time_step, uncommitted_trajectory_cost, u_v, u_v_inverse, wheelbase, wheelbase_inverse;
    std::vector<double> committed_trajectory_costs, connection_time_vector, connection_time_min_vector, controlled_trajectory_costs, costs, distances, q_goal, q_initial, q_min, q_near, q_nearest, q_new, q_random, steered_time_vector, tag_vector, U_phi, uncommitted_trajectory_costs, x_range, y_range;
    std::vector<std::vector<double>> committed_trajectory, controlled_trajectory, cost_vectors, edge, edge_min, path_phi_vector, Q_near, r_vector, steered_edge, successful_vertices, tag_configurations, uncommitted_trajectory, V;
    std::vector<std::vector<std::vector<double>>> E, tag_obstacles;
    
    // declare ROS attributes
    ros::Publisher pub_controlled_trajectory, pub_edges, pub_planning_complete;
    ros::Subscriber sub_controlled_trajectory_index, sub_initialise;
    std_msgs::Bool msg_planning_complete;

    public:
        // define node constructor
        PlanningNode(ros::NodeHandle* nh) {
            // initialise standard attributes
            new_controlled_trajectory = false;
            planning_complete = false;
            committed_trajectory_size = 0;
            counter = 0;
            successes = 0;
            number_of_path_types = 6;
            U_phi_size = 3;
            V_size = 1;
            M_2PI = 2 * M_PI;
            M_3PI_2 = 3 * M_PI_2;
            uncommitted_trajectory_cost = INFINITY;
            committed_trajectory_costs = {};
            costs = {0};
            committed_trajectory = {{}, {}, {}};
            srand(time(0));

            // initialise ROS attributes
            pub_planning_complete = nh->advertise<std_msgs::Bool>("planning_complete", 1);
            pub_controlled_trajectory = nh->advertise<std_msgs::Float64MultiArray>("controlled_trajectory", 1);
            pub_edges = nh->advertise<std_msgs::Float64MultiArray>("edges", 1);
            sub_controlled_trajectory_index = nh->subscribe("controlled_trajectory_index", 1, &PlanningNode::controlled_trajectory_index_callback, this);
            sub_initialise = nh->subscribe<std_msgs::Bool>("initialise", 1, boost::bind(&PlanningNode::initialise_callback, this, _1, nh));
        }

        // define callback methods
        void controlled_trajectory_index_callback(const std_msgs::Int64ConstPtr& msg_controlled_trajectory_index) {
            if (msg_controlled_trajectory_index->data == committed_trajectory_size) {
                if (root_index < uncommitted_trajectory_size) {
                    for (i = 0; i < 3; i++) {
                        committed_trajectory[i].insert(committed_trajectory[i].end(), uncommitted_trajectory[i].begin(), uncommitted_trajectory[i].begin() + root_index);
                        uncommitted_trajectory[i] = {uncommitted_trajectory[i].begin() + root_index, uncommitted_trajectory[i].end()};
                    }
                    committed_trajectory_size = committed_trajectory[0].size();
                    committed_trajectory_costs.insert(committed_trajectory_costs.end(), uncommitted_trajectory_costs.begin(), uncommitted_trajectory_costs.begin() + root_index);
                    uncommitted_trajectory_size = uncommitted_trajectory[0].size();
                    uncommitted_trajectory_costs = {uncommitted_trajectory_costs.begin() + root_index, uncommitted_trajectory_costs.end()};
                    uncommitted_trajectory_cost = std::accumulate(uncommitted_trajectory_costs.begin(), uncommitted_trajectory_costs.end(), 0.0);
                    q_initial = {committed_trajectory[0].back(), committed_trajectory[1].back(), committed_trajectory[2].back()};
                    V = {q_initial};
                    V_size = 1;
                    E = {{std::vector<double>(number_of_steps, q_initial[0]), std::vector<double>(number_of_steps, q_initial[1]), std::vector<double>(number_of_steps, q_initial[2])}};
                    publish_edges(pub_edges, E);
                    costs = {0};
                    cost_vectors = {std::vector<double>(number_of_steps, 0)};
                    successful_vertices = {};
                    successes = 0;
                }
                else {
                    planning_complete = true;
                    msg_planning_complete.data = planning_complete;
                    pub_planning_complete.publish(msg_planning_complete);
                }
            }
        }

        void initialise_callback(const std_msgs::BoolConstPtr& msg_initialise, ros::NodeHandle* nh) {
            // get parameter values
            nh->getParam("biased_step", biased_step);
            nh->getParam("buffer", buffer);
            nh->getParam("gamma", gamma);
            nh->getParam("goal_radius", goal_radius);
            nh->getParam("number_of_samples", number_of_samples);
            nh->getParam("number_of_steps", number_of_steps);
            nh->getParam("phi_max_degrees_planning", phi_max_degrees_planning);
            nh->getParam("root_index", root_index);
            nh->getParam("q_goal", q_goal);
            nh->getParam("q_initial", q_initial);
            nh->getParam("steered_distance", steered_distance);
            nh->getParam("tag_obstacle_length", tag_obstacle_length);
            nh->getParam("tag_obstacle_width", tag_obstacle_width);
            nh->getParam("tag_vector", tag_vector);
            nh->getParam("u_v", u_v);
            nh->getParam("wheelbase", wheelbase);
            nh->getParam("x_range", x_range);
            nh->getParam("y_range", y_range);
            
            // initialise standard attributes with parameter values
            u_v_inverse = 1 / u_v;
            steered_time = steered_distance * u_v_inverse;
            steps_per_second = number_of_steps / steered_time;
            time_step = 1 / steps_per_second;
            steered_time_vector = std::vector<double>(number_of_steps, time_step);
            distance_step = u_v * time_step;
            wheelbase_inverse = 1 / wheelbase;
            distance_step_times_wheelbase_inverse = distance_step * wheelbase_inverse;
            V = {q_initial};
            E = {{std::vector<double>(number_of_steps, q_initial[0]), std::vector<double>(number_of_steps, q_initial[1]), std::vector<double>(number_of_steps, q_initial[2])}};
            cost_vectors = {std::vector<double>(number_of_steps, 0)};
            number_of_tags = tag_vector.size() / 4;
            tag_configurations = std::vector<std::vector<double>>(number_of_tags, std::vector<double>(3, 0));
            for (i = 0; i < number_of_tags; i++) {
                tag_configurations[i] = {tag_vector[4 * i + 1], tag_vector[4 * i + 2], tag_vector[4 * i + 3]};
            }
            tag_obstacles = tag_obstacle_generator(tag_configurations, number_of_tags, tag_obstacle_length, tag_obstacle_width);
            phi_max = phi_max_degrees_planning * M_PI / 180;
            U_phi = {-phi_max, 0, phi_max};
            path_phi_vector = {{phi_max, 0, phi_max}, {phi_max, 0, -phi_max}, {-phi_max, 0, phi_max}, {-phi_max, 0, -phi_max}, {phi_max, -phi_max, phi_max}, {-phi_max, phi_max, -phi_max}};
            r_vector = std::vector<std::vector<double>>(number_of_path_types, std::vector<double>(3, 0));
            for (i = 0; i < number_of_path_types; i++) {
                for (j = 0; j < 3; j++) {
                    r_vector[i][j] = wheelbase / std::tan(path_phi_vector[i][j]);
                }
            }
            circumference = wheelbase / std::tan(phi_max) * M_2PI;
            semicircumference = circumference * 0.5;
            
            // begin RRT* loop
            tic = ros::Time::now().toSec();
            while (!planning_complete) {
                q_random = sample_free_space(counter, biased_step, q_goal, x_range, y_range, tag_obstacles, tag_configurations, buffer, number_of_samples);
                q_nearest = nearest(V, V_size, q_random);
                std::tie(q_new, steered_edge) = steer(q_nearest, q_random, number_of_steps, distance_step, distance_step_times_wheelbase_inverse, U_phi, U_phi_size);
                if (!repeated(V, q_new, M_2PI) && less_or_equal(x_range[0], q_new[0]) && less_or_equal(q_new[0], x_range[1]) && less_or_equal(y_range[0], q_new[1]) && less_or_equal(q_new[1], y_range[1])) {
                    if (collision_free(x_range, y_range, tag_obstacles, tag_configurations, buffer, steered_edge[0], steered_edge[1], number_of_samples)) {
                        Q_radius = gamma * std::sqrt(std::log(V_size) / V_size);
                        Q_near = near(V, q_new, Q_radius);
                        Q_near_size = Q_near.size();
                        V.push_back(q_new);
                        V_size++;
                        q_min = q_nearest;
                        cost_min = calculate_cost(E, costs, q_initial, q_nearest, M_2PI) + steered_time;
                        for (i = 0; i < Q_near_size; i++) {
                            q_near = Q_near[i];
                            std::tie(edge, connection_time, connection_time_vector) = connect(r_vector, path_phi_vector, number_of_path_types, q_near, q_new, u_v, wheelbase, steps_per_second, circumference, semicircumference, M_3PI_2, M_2PI);
                            cost = calculate_cost(E, costs, q_initial, q_near, M_2PI) + connection_time;
                            if (collision_free(x_range, y_range, tag_obstacles, tag_configurations, buffer, edge[0], edge[1], number_of_samples) && cost < cost_min) {
                                q_min = q_near;
                                cost_min = cost;
                                edge_min = edge;
                                connection_time_min = connection_time;
                                connection_time_min_vector = connection_time_vector;
                            }
                        }
                        if (equal_configurations(q_min, q_nearest, M_2PI)) {
                            E.push_back(steered_edge);
                            costs.push_back(steered_time);
                            cost_vectors.push_back(steered_time_vector);
                        }
                        else {
                            E.push_back(edge_min);
                            costs.push_back(connection_time_min);
                            cost_vectors.push_back(connection_time_min_vector);
                        }
                        publish_edges(pub_edges, E);
                        for (i = 0; i < Q_near_size; i++) {
                            q_near = Q_near[i];
                            std::tie(edge, connection_time, connection_time_vector) = connect(r_vector, path_phi_vector, number_of_path_types, q_new, q_near, u_v, wheelbase, steps_per_second, circumference, semicircumference, M_3PI_2, M_2PI);
                            if (collision_free(x_range, y_range, tag_obstacles, tag_configurations, buffer, edge[0], edge[1], number_of_samples) && calculate_cost(E, costs, q_initial, q_new, M_2PI) + connection_time < calculate_cost(E, costs, q_initial, q_near, M_2PI)) {
                                std::tie(std::ignore, index) = parent(E, q_near, M_2PI);
                                E.erase(E.begin() + index);
                                costs.erase(costs.begin() + index);
                                cost_vectors.erase(cost_vectors.begin() + index);
                                E.push_back(edge);
                                costs.push_back(connection_time);
                                cost_vectors.push_back(connection_time_vector);
                            }
                        }
                        publish_edges(pub_edges, E);
                        if (less_or_equal(std::sqrt(std::pow(q_goal[0] - q_new[0], 2) + std::pow(q_goal[1] - q_new[1], 2)), goal_radius)) {
                            successful_vertices.push_back(q_new);
                            successes++;
                        }
                        if (successes) {
                            for (i = 0; i < successes; i++) {
                                cost = calculate_cost(E, costs, q_initial, successful_vertices[i], M_2PI);
                                if (cost < uncommitted_trajectory_cost) {
                                    index = i;
                                    uncommitted_trajectory_cost = cost;
                                    new_controlled_trajectory = true;
                                }
                            }
                            if (new_controlled_trajectory) {
                                std::tie(uncommitted_trajectory, uncommitted_trajectory_size, uncommitted_trajectory_costs) = find_trajectory(E, cost_vectors, q_initial, successful_vertices[index]);
                                std::tie(controlled_trajectory, controlled_trajectory_size, controlled_trajectory_costs) = create_controlled_trajectory(committed_trajectory, committed_trajectory_size, committed_trajectory_costs, uncommitted_trajectory, uncommitted_trajectory_size, uncommitted_trajectory_costs, u_v, wheelbase);
                                publish_controlled_trajectory(pub_controlled_trajectory, controlled_trajectory, controlled_trajectory_size, controlled_trajectory_costs);
                                new_controlled_trajectory = false;
                            }
                        }
                        if (uncommitted_trajectory_cost != INFINITY) {
                            std::tie(V, V_size, E, costs, cost_vectors, successful_vertices, successes) = branch_and_bound(q_initial, q_goal, V, V_size, E, costs, cost_vectors, goal_radius, u_v_inverse, uncommitted_trajectory_cost, successful_vertices, successes, M_2PI);
                            publish_edges(pub_edges, E);
                        }
                    }
                }
                counter++;
                ROS_INFO("planning counter = %d", counter);
                ros::spinOnce();
            }
            ROS_INFO("planning complete");
            ROS_INFO("planning time = %f", ros::Time::now().toSec() - tic);
        }

        // define standard methods
        std::tuple<std::vector<std::vector<double>>, int, std::vector<std::vector<std::vector<double>>>, std::vector<double>, std::vector<std::vector<double>>, std::vector<std::vector<double>>, int> branch_and_bound(const std::vector<double>& q_initial, const std::vector<double>& q_goal, const std::vector<std::vector<double>>& V, const int& V_size, const std::vector<std::vector<std::vector<double>>>& E, const std::vector<double>& costs, const std::vector<std::vector<double>>& cost_vectors, const double& goal_radius, const double& u_v_inverse, const double& uncommitted_trajectory_cost, const std::vector<std::vector<double>>& successful_vertices, const int& successes, const double& M_2PI) {
            int i, index, j, successes_new, V_size_new;
            double cost, cost_to_go;
            std::vector<int> V_index_vector;
            std::vector<double> costs_new;
            std::vector<std::vector<double>> cost_vectors_new, successful_vertices_new, V_new;
            std::vector<std::vector<std::vector<double>>> E_new;
            
            successful_vertices_new = successful_vertices;
            successes_new = successes;
            for (i = 1; i < V_size; i++) {
                cost = calculate_cost(E, costs, q_initial, V[i], M_2PI);
                cost_to_go = (std::sqrt(std::pow(q_goal[0] - V[i][0], 2) + std::pow(q_goal[1] - V[i][1], 2)) - goal_radius) * u_v_inverse;
                if (cost_to_go > 0) {
                    if (greater_or_equal(cost + cost_to_go, uncommitted_trajectory_cost)) {
                        V_index_vector.push_back(i);
                    }
                }
                else if (cost > uncommitted_trajectory_cost) {
                    for (j = 0; j < successes_new; j++) {
                        if (equal_configurations(successful_vertices_new[j], V[i], M_2PI)) {
                            V_index_vector.push_back(i);
                            successful_vertices_new.erase(successful_vertices_new.begin() + j);
                            successes_new--;
                            break;
                        }
                    }
                }
            }
            V_new = V;
            E_new = E;
            costs_new = costs;
            cost_vectors_new = cost_vectors;
            for (i = 0; i < V_index_vector.size(); i++) {
                std::tie(std::ignore, index) = parent(E_new, V_new[V_index_vector[i] - i], M_2PI);
                E_new.erase(E_new.begin() + index);
                costs_new.erase(costs_new.begin() + index);
                cost_vectors_new.erase(cost_vectors_new.begin() + index);
                V_new.erase(V_new.begin() + V_index_vector[i] - i);
            }
            V_size_new = V_new.size();

            return {V_new, V_size_new, E_new, costs_new, cost_vectors_new, successful_vertices_new, successes_new};
        }

        double calculate_cost(const std::vector<std::vector<std::vector<double>>>& E, const std::vector<double>& costs, const std::vector<double>& q_initial, const std::vector<double>& q, const double& M_2PI) {
            int index;
            double cost;
            std::vector<double> p;
            
            std::tie(p, index) = parent(E, q, M_2PI);
            cost = costs[index];
            while (!equal_configurations(p, q_initial, M_2PI)) {
                std::tie(p, index) = parent(E, p, M_2PI);
                cost += costs[index];
            }
            
            return cost;
        }

        bool collision_free(const std::vector<double>& x_range, const std::vector<double>& y_range, const std::vector<std::vector<std::vector<double>>>& tag_obstacles, const std::vector<std::vector<double>>& tag_configurations, const double& buffer, const std::vector<double>& x, const std::vector<double>& y, const int& number_of_samples) {
            bool free;
            int i;
            
            free = 1;
            for (i = 0; i < tag_obstacles.size(); i++) {
                if (obstacle_collision(tag_obstacles[i], tag_configurations[i], buffer, x, y, number_of_samples)) {
                    free = 0;
                    break;
                }
            }
            if (free) {
                for (i = 0; i < x.size(); i++) {
                    if (x[i] < x_range[0] || x[i] > x_range[1] || y[i] < y_range[0] || y[i] > y_range[1]) {
                        free = 0;
                        break;
                    }
                }
            }
            
            return free;
        }

        std::tuple<std::vector<double>, std::vector<double>> compare_paths(const int& number_of_path_types, const std::vector<std::vector<double>>& r_vector, const double& wheelbase, const double& u_v_inverse, const std::vector<std::vector<double>>& path_phi_vector, const double& Delta_x_b, const double& Delta_y_b, const double& Delta_theta_total, const double& circumference, const double& M_2PI) {
            int i, index, j;
            double alpha, beta, Delta_r, Delta_x_2, gamma;
            std::vector<double> Delta_theta, path_time_sum_vector, shortest_path_phis, shortest_path_times;
            std::vector<std::vector<double>> path_time_vector;

            path_time_vector = std::vector<std::vector<double>>(number_of_path_types, {INFINITY, INFINITY, INFINITY});
            path_time_sum_vector = std::vector<double>(number_of_path_types, 0);
            for (i = 0; i < number_of_path_types; i++) {
                Delta_theta = {0, 0, 0};
                if (equal(path_phi_vector[i][1], 0)) {
                    alpha = Delta_x_b - r_vector[i][2] * std::sin(Delta_theta_total);
                    beta = Delta_y_b - r_vector[i][0] + r_vector[i][2] * std::cos(Delta_theta_total);
                    Delta_r = r_vector[i][2] - r_vector[i][0];
                    gamma = alpha * alpha + beta * beta - Delta_r * Delta_r;
                    if (greater_or_equal(gamma, 0)) {
                        Delta_x_2 = std::sqrt(gamma);
                        Delta_theta[0] = 2 * std::atan2(Delta_x_2 - alpha, beta + Delta_r);
                        Delta_theta[2] = Delta_theta_total - Delta_theta[0];
                        for (j = 0; j < 3; j++) {
                            if (j != 1) {
                                path_time_vector[i][j] = modulo(Delta_theta[j] * r_vector[i][j], circumference) * u_v_inverse;
                            }
                        }
                        path_time_vector[i][1] = Delta_x_2 * u_v_inverse;
                    }
                }
                else {
                    alpha = (std::sin(Delta_theta_total) - Delta_x_b / r_vector[i][0]) * 0.5;
                    beta = (1 - std::cos(Delta_theta_total) - Delta_y_b / r_vector[i][0]) * 0.5;
                    gamma = alpha * alpha + beta * beta;
                    Delta_theta[1] = modulo(((path_phi_vector[i][0] > 0) - (path_phi_vector[i][0] < 0)) * std::acos(1 - gamma * 0.5), M_2PI);
                    Delta_theta[0] = 2 * std::atan2(std::sqrt(gamma) - alpha, beta) - Delta_theta[1] * 0.5;
                    Delta_theta[2] = Delta_theta_total - Delta_theta[0] - Delta_theta[1];
                    for (j = 0; j < 3; j++) {
                        path_time_vector[i][j] = modulo(Delta_theta[j] * r_vector[i][j], circumference) * u_v_inverse;
                    }
                }
                path_time_sum_vector[i] = path_time_vector[i][0] + path_time_vector[i][1] + path_time_vector[i][2];
            }
            index = std::min_element(path_time_sum_vector.begin(), path_time_sum_vector.end()) - path_time_sum_vector.begin();
            shortest_path_phis = path_phi_vector[index];
            shortest_path_times = path_time_vector[index];

            return {shortest_path_phis, shortest_path_times};
        }

        std::tuple<std::vector<std::vector<double>>, double, std::vector<double>> connect(const std::vector<std::vector<double>>& r_vector, const std::vector<std::vector<double>>& path_phi_vector, const int& number_of_path_types, const std::vector<double>& q_initial, const std::vector<double>& q_goal, const double& u_v, const double& wheelbase, const double& steps_per_second, const double& circumference, const double& semicircumference, const double& M_3PI_2, const double& M_2PI) {
            int i, j, k;
            double alpha, beta, connection_time, cosine, Delta_theta, Delta_theta_total, Delta_x, Delta_x_b, Delta_x_s, Delta_y_b, Delta_y_s, gamma, sine, time_step;
            std::vector<int> linear_space_sizes;
            std::vector<double> connection_time_vector, edge_theta, edge_x, edge_y, shortest_path_phis, shortest_path_times, zero_vector;
            std::vector<std::vector<double>> edge, flow, g, g_f, g_i, linear_spaces;
            std::vector<std::vector<std::vector<double>>> twists;
            
            Delta_x_s = q_goal[0] - q_initial[0];
            Delta_y_s = q_goal[1] - q_initial[1];
            cosine = std::cos(q_initial[2]);
            sine = std::sin(q_initial[2]);
            Delta_x_b = cosine * Delta_x_s + sine * Delta_y_s;
            Delta_y_b = -sine * Delta_x_s + cosine * Delta_y_s;
            Delta_theta_total = q_goal[2] - q_initial[2];
            gamma = std::atan2(Delta_y_s, Delta_x_s);
            alpha = modulo(q_initial[2] - gamma, M_2PI);
            beta = modulo(q_goal[2] - gamma, M_2PI);
            if (std::sqrt(Delta_x_s * Delta_x_s + Delta_y_s * Delta_y_s) > std::sqrt(4 - std::pow(std::abs(std::cos(alpha)) + std::abs(std::cos(beta)), 2)) + std::abs(std::sin(alpha)) + std::abs(std::sin(beta))) {
                std::tie(shortest_path_phis, shortest_path_times) = shortest_path(wheelbase, alpha, beta, Delta_x_b, Delta_y_b, Delta_theta_total, r_vector, path_phi_vector, semicircumference, M_3PI_2, M_2PI);
            }
            else {
                std::tie(shortest_path_phis, shortest_path_times) = compare_paths(number_of_path_types, r_vector, wheelbase, u_v_inverse, path_phi_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, M_2PI);
            }
            if (shortest_path_times[0] != INFINITY) {
                twists = std::vector<std::vector<std::vector<double>>>(3, std::vector<std::vector<double>>(3, std::vector<double>(3, 0)));
                g_i = {{cosine, -sine, q_initial[0]}, {sine, cosine, q_initial[1]}, {0, 0, 1}};
                g = g_i;
                linear_spaces = {linear_space(0, 1, static_cast<int> (std::ceil(shortest_path_times[0] * steps_per_second))), linear_space(0, 1, static_cast<int> (std::ceil(shortest_path_times[1] * steps_per_second))), linear_space(0, 1, static_cast<int> (std::ceil(shortest_path_times[2] * steps_per_second)))};
                linear_space_sizes = {static_cast<int> (linear_spaces[0].size()), static_cast<int> (linear_spaces[1].size()), static_cast<int> (linear_spaces[2].size())};
                zero_vector = std::vector<double>(linear_space_sizes[0] + linear_space_sizes[1] + linear_space_sizes[2] + 1, 0);
                edge_x = zero_vector;
                edge_y = zero_vector;
                edge_theta = zero_vector;
                connection_time_vector = zero_vector;
                edge_x[0] = q_initial[0];
                edge_y[0] = q_initial[1];
                edge_theta[0] = q_initial[2];
                k = 1;
                for (i = 0; i < 3; i++) {
                    Delta_x = u_v * shortest_path_times[i];
                    Delta_theta = Delta_x * std::tan(shortest_path_phis[i]) * wheelbase_inverse;
                    twists[i] = {{0, -Delta_theta, Delta_x}, {Delta_theta, 0, 0}, {0, 0, 0}};
                    if (i == 1) {
                        flow = flow_matrix_exponential(twists[0]);
                        g = matrix_multiplication(g_i, flow);
                    }
                    else if (i == 2) {
                        g = matrix_multiplication(g_i, matrix_multiplication(flow, flow_matrix_exponential(twists[1])));
                    }
                    time_step = shortest_path_times[i] / linear_space_sizes[i];
                    for (j = 0; j < linear_space_sizes[i]; j++) {
                        g_f = matrix_multiplication(g, flow_matrix_exponential(scale_matrix(twists[i], linear_spaces[i][j])));
                        edge_x[k] = g_f[0][2];
                        edge_y[k] = g_f[1][2];
                        edge_theta[k] = std::atan2(g_f[1][0], g_f[0][0]);
                        connection_time_vector[k] = time_step;
                        k++;
                    }
                }
                edge_x.back() = q_goal[0];
                edge_y.back() = q_goal[1];
                edge_theta.back() = q_goal[2];
                edge = {edge_x, edge_y, edge_theta};
                connection_time = shortest_path_times[0] + shortest_path_times[1] + shortest_path_times[2];
            }
            
            return {edge, connection_time, connection_time_vector};
        }

        std::tuple<std::vector<std::vector<double>>, int, std::vector<double>> create_controlled_trajectory(const std::vector<std::vector<double>>& committed_trajectory, const int& committed_trajectory_size, const std::vector<double>& committed_trajectory_costs, const std::vector<std::vector<double>>& uncommitted_trajectory, const int& uncommitted_trajectory_size, const std::vector<double>& uncommitted_trajectory_costs, const double& u_v, const double& wheelbase) {
            int controlled_trajectory_size, i;
            std::vector<double> controlled_trajectory_costs;
            std::vector<std::vector<double>> controlled_trajectory;
            
            controlled_trajectory_size = committed_trajectory_size + uncommitted_trajectory_size;
            controlled_trajectory_costs = committed_trajectory_costs;
            controlled_trajectory_costs.insert(controlled_trajectory_costs.end(), uncommitted_trajectory_costs.begin(), uncommitted_trajectory_costs.end());
            controlled_trajectory = {committed_trajectory[0], committed_trajectory[1], committed_trajectory[2], std::vector<double>(controlled_trajectory_size, u_v), std::vector<double>(controlled_trajectory_size, 0)};
            for (i = 0; i < 3; i++) {
                controlled_trajectory[i].insert(controlled_trajectory[i].end(), uncommitted_trajectory[i].begin(), uncommitted_trajectory[i].end());
            }
            for (i = 0; i < controlled_trajectory_size - 1; i++) {
                controlled_trajectory[4][i + 1] = std::atan2((controlled_trajectory[2][i + 1] - controlled_trajectory[2][i]) * wheelbase, u_v * controlled_trajectory_costs[i + 1]);
            }

            return {controlled_trajectory, controlled_trajectory_size, controlled_trajectory_costs};
        }
        
        std::tuple<std::vector<std::vector<double>>, int, std::vector<double>> find_trajectory(const std::vector<std::vector<std::vector<double>>>& E, const std::vector<std::vector<double>>& cost_vectors, const std::vector<double>& q_initial, const std::vector<double>& q) {
            int i, j, trajectory_size;
            std::vector<double> c, p, trajectory_costs;
            std::vector<std::vector<double>> trajectory;
            std::vector<std::vector<std::vector<double>>> E_path;
            
            c = q;
            std::tie(p, std::ignore) = parent(E, c, M_2PI);
            E_path = {{p, c}};
            while (!equal_configurations(p, q_initial, M_2PI)) {
                c = p;
                std::tie(p, std::ignore) = parent(E, c, M_2PI);
                E_path.push_back({p, c});
            }
            std::reverse(E_path.begin(), E_path.end());
            trajectory = {{q_initial[0]}, {q_initial[1]}, {q_initial[2]}};
            trajectory_costs = {0};
            for (i = 0; i < E_path.size(); i++) {
                for (j = 0; j < E.size(); j++) {
                    if (equal_configurations({E[j][0].back(), E[j][1].back(), E[j][2].back()}, E_path[i][1], M_2PI)) {
                        trajectory[0].insert(trajectory[0].end(), E[j][0].begin() + 1, E[j][0].end());
                        trajectory[1].insert(trajectory[1].end(), E[j][1].begin() + 1, E[j][1].end());
                        trajectory[2].insert(trajectory[2].end(), E[j][2].begin() + 1, E[j][2].end());
                        trajectory_costs.insert(trajectory_costs.end(), cost_vectors[j].begin() + 1, cost_vectors[j].end());
                        break;
                    }
                }
            }
            trajectory_size = trajectory[0].size();

            return {trajectory, trajectory_size, trajectory_costs};
        }

        bool interior(const std::vector<std::vector<double>>& obstacle, const std::vector<double>& q_obstacle, const double& buffer, const double& x, const double& y) {
            bool c;
            int i;
            double cosine, sine, x_transformed, x_translated, y_transformed, y_translated;
            std::vector<std::vector<double>> obstacle_transformed;
            
            c = 0;
            obstacle_transformed = rotate_obstacle(translate_obstacle(obstacle, -q_obstacle[0], -q_obstacle[1]), -q_obstacle[2]);
            x_translated = x - q_obstacle[0];
            y_translated = y - q_obstacle[1];
            cosine = std::cos(-q_obstacle[2]);
            sine = std::sin(-q_obstacle[2]);
            x_transformed = x_translated * cosine - y_translated * sine;
            y_transformed = x_translated * sine + y_translated * cosine;
            if (less_or_equal(obstacle_transformed[0][0] - buffer, x_transformed) && less_or_equal(x_transformed, obstacle_transformed[2][0] + buffer) && less_or_equal(obstacle_transformed[0][1], y_transformed) && less_or_equal(y_transformed, obstacle_transformed[2][1])) {
                c = 1;
            }
            else if (less_or_equal(obstacle_transformed[0][0], x_transformed) && less_or_equal(x_transformed, obstacle_transformed[2][0]) && less_or_equal(obstacle_transformed[0][1] - buffer, y_transformed) && less_or_equal(y_transformed, obstacle_transformed[2][1] + buffer)) {
                c = 1;
            }
            else {
                for (i = 0; i < 4; i++) {
                    if (less_or_equal(std::sqrt(std::pow(obstacle_transformed[i][0] - x_transformed, 2) + std::pow(obstacle_transformed[i][1] - y_transformed, 2)), buffer)) {
                        c = 1;
                        break;
                    }
                }
            }
            
            return c;
        }

        std::vector<std::vector<double>> near(const std::vector<std::vector<double>>& V, const std::vector<double>& q_new, const double& Q_radius) {
            int i;
            std::vector<std::vector<double>> Q_near;

            for (i = 0; i < V.size(); i++) {
                if (std::sqrt(std::pow(V[i][0] - q_new[0], 2) + std::pow(V[i][1] - q_new[1], 2)) < Q_radius) {
                    Q_near.push_back(V[i]);
                }
            }

            return Q_near;
        }

        std::vector<double> nearest(const std::vector<std::vector<double>>& V, const int& V_size, const std::vector<double>& q_random) {
            int i;
            std::vector<double> distances, q_nearest;

            distances = std::vector<double>(V_size, 0);
            for (i = 0; i < V_size; i++) {
                distances[i] = std::sqrt(std::pow(V[i][0] - q_random[0], 2) + std::pow(V[i][1] - q_random[1], 2));
            }
            q_nearest = V[std::min_element(distances.begin(), distances.end()) - distances.begin()];

            return q_nearest;
        }
        
        bool obstacle_collision(const std::vector<std::vector<double>>& obstacle, const std::vector<double>& q_obstacle, const double& buffer, const std::vector<double>& x, const std::vector<double>& y, const int& number_of_samples) {
            bool c;
            int i, index, length, sample_size;
            
            c = 0;
            if (interior(obstacle, q_obstacle, buffer, x.back(), y.back())) {
                c = 1;
            }
            else {
                length = x.size();
                if (length > 1) {
                    sample_size = number_of_samples;
                    while (sample_size > length) {
                        sample_size *= 0.5;
                    }
                    for (i = 1; i < sample_size; i++) {
                        index = van_der_corput_sequence(length, i);
                        if (interior(obstacle, q_obstacle, buffer, x[index], y[index])) {
                            c = 1;
                            break;
                        }
                    }
                }
            }
            
            return c;
        }

        std::tuple<std::vector<double>, int> parent(const std::vector<std::vector<std::vector<double>>>& E, const std::vector<double>& c, const double& M_2PI) {
            int i;
            std::vector<double> p;
            
            for (i = 0; i < E.size(); i++) {
                if (equal_configurations({E[i][0].back(), E[i][1].back(), E[i][2].back()}, c, M_2PI)) {
                    p = {E[i][0][0], E[i][1][0], E[i][2][0]};
                    break;
                }
            }
            
            return {p, i};
        }

        std::tuple<double, double, double> path_length(const double& wheelbase, const std::vector<std::vector<double>>& r_vector, const double& Delta_x_b, const double& Delta_y_b, const double& Delta_theta_total, const double& circumference, const int& index) {
            double alpha, beta, Delta_r, Delta_theta_1, r_1, r_3, p, q, t;
            
            alpha = Delta_x_b - r_vector[index][2] * std::sin(Delta_theta_total);
            beta = Delta_y_b - r_vector[index][0] + r_vector[index][2] * std::cos(Delta_theta_total);
            Delta_r = r_vector[index][2] - r_vector[index][0];
            p = std::sqrt(alpha * alpha + beta * beta - Delta_r * Delta_r);
            Delta_theta_1 = 2 * std::atan2(p - alpha, beta + Delta_r);
            t = modulo(Delta_theta_1 * r_vector[index][0], circumference);
            q = modulo((Delta_theta_total - Delta_theta_1) * r_vector[index][2], circumference);

            return {t, p, q};
        }

        void publish_controlled_trajectory(const ros::Publisher& pub_controlled_trajectory, const std::vector<std::vector<double>>& controlled_trajectory, const int& controlled_trajectory_size, const std::vector<double>& controlled_trajectory_costs) {
            int i;
            std_msgs::Float64MultiArray msg_controlled_trajectory;
            
            msg_controlled_trajectory.data = {(double) controlled_trajectory_size};
            msg_controlled_trajectory.data.insert(msg_controlled_trajectory.data.end(), controlled_trajectory_costs.begin(), controlled_trajectory_costs.end());
            for (i = 0; i < 5; i++) {
                msg_controlled_trajectory.data.insert(msg_controlled_trajectory.data.end(), controlled_trajectory[i].begin(), controlled_trajectory[i].end());
            }
            pub_controlled_trajectory.publish(msg_controlled_trajectory);
        }

        void publish_edges(const ros::Publisher& pub_edges, const std::vector<std::vector<std::vector<double>>>& E) {
            int E_size, i, j;
            std_msgs::Float64MultiArray msg_edges;

            E_size = E.size();
            msg_edges.data = {(double) E_size};
            for (i = 0; i < E_size; i++) {
                msg_edges.data.push_back(E[i][0].size());
                for (j = 0; j < 3; j++) {
                    msg_edges.data.insert(msg_edges.data.end(), E[i][j].begin(), E[i][j].end());
                }
            }
            pub_edges.publish(msg_edges);
        }

        bool repeated(const std::vector<std::vector<double>>& V, const std::vector<double>& q_new, const double& M_2PI) {
            bool r;
            int i;

            r = 0;
            for (i = 0; i < V.size(); i++) {
                if (equal_configurations(V[i], q_new, M_2PI)) {
                    r = 1;
                    break;
                }
            }

            return r;
        }

        std::vector<std::vector<double>> rotate_obstacle(const std::vector<std::vector<double>>& tag_obstacle, const double& theta) {
            int i;
            double cosine, sine;
            std::vector<std::vector<double>> tag_obstacle_rotated;

            cosine = std::cos(theta);
            sine = std::sin(theta);
            tag_obstacle_rotated = std::vector<std::vector<double>>(4, std::vector<double>(2, 0));
            for (i = 0; i < 4; i++) {
                tag_obstacle_rotated[i][0] = tag_obstacle[i][0] * cosine - tag_obstacle[i][1] * sine;
                tag_obstacle_rotated[i][1] = tag_obstacle[i][0] * sine + tag_obstacle[i][1] * cosine;
            }

            return tag_obstacle_rotated;
        }

        double sample(const std::vector<double>& range) {
            double range_sample;
            std::vector<double> range_int;
            
            range_int = {range[0] * 100, range[1] * 100};
            range_sample = (range_int[0] + modulo(rand(), (range_int[1] - range_int[0] + 1))) * 0.01;
            
            return range_sample;
        }

        std::vector<double> sample_free_space(const int& counter, const int& biased_step, const std::vector<double>& q_goal, const std::vector<double>& x_range, const std::vector<double>& y_range, const std::vector<std::vector<std::vector<double>>>& tag_obstacles, const std::vector<std::vector<double>>& tag_configurations, const double& buffer, const int& number_of_samples) {
            std::vector<double> q_random, x, y;

            if (counter % biased_step) {
                x = {sample(x_range)};
                y = {sample(y_range)};
                while (!collision_free(x_range, y_range, tag_obstacles, tag_configurations, buffer, x, y, number_of_samples)) {
                    x = {sample(x_range)};
                    y = {sample(y_range)};
                }
                q_random = {x[0], y[0]};
            }
            else {
                q_random = {q_goal[0], q_goal[1]};
            }

            return q_random;
        }

        std::tuple<std::vector<double>, std::vector<double>> shortest_path(const double& wheelbase, const double& alpha, const double& beta, const double& Delta_x_b, const double& Delta_y_b, const double& Delta_theta_total, const std::vector<std::vector<double>>& r_vector, const std::vector<std::vector<double>>& path_phi_vector, const double& semicircumference, const double& M_3PI_2, const double& M_2PI) {
            double p, q, t, p_lsl, p_lsr, p_rsl, p_rsr, q_lsl, q_lsr, q_rsl, q_rsr, t_lsl, t_lsr, t_rsl, t_rsr;
            std::vector<double> shortest_path_phis, shortest_path_times;
            
            if (alpha < M_PI_2) {
                if (beta < M_PI_2) {
                    std::tie(t, p, q) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 2);
                    shortest_path_phis = path_phi_vector[2];
                }
                else if (beta < M_PI) {
                    std::tie(t_rsr, p_rsr, q_rsr) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 3);
                    std::tie(t_rsl, p_rsl, q_rsl) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 2);
                    if (p_rsr - p_rsl - 2 * q_rsl + circumference < 0) {
                        t = t_rsr;
                        p = p_rsr;
                        q = q_rsr;
                        shortest_path_phis = path_phi_vector[3];
                    }
                    else {
                        t = t_rsl;
                        p = p_rsl;
                        q = q_rsl;
                        shortest_path_phis = path_phi_vector[2];
                    }
                }
                else if (beta < M_3PI_2) {
                    std::tie(t_rsr, p_rsr, q_rsr) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 3);
                    if (t_rsr - semicircumference < 0) {
                        t = t_rsr;
                        p = p_rsr;
                        q = q_rsr;
                        shortest_path_phis = path_phi_vector[3];
                    }
                    else {
                        t = t_lsr;
                        p = p_lsr;
                        q = q_lsr;
                        shortest_path_phis = path_phi_vector[1];
                    }
                }
                else {
                    std::tie(t_rsr, p_rsr, q_rsr) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 3);
                    if (t_rsr - semicircumference > 0) {
                        t = t_lsr;
                        p = p_lsr;
                        q = q_lsr;
                        shortest_path_phis = path_phi_vector[1];
                    }
                    else if (q_rsr - semicircumference > 0) {
                        t = t_rsl;
                        p = p_rsl;
                        q = q_rsl;
                        shortest_path_phis = path_phi_vector[2];
                    }
                    else {
                        t = t_rsr;
                        p = p_rsr;
                        q = q_rsr;
                        shortest_path_phis = path_phi_vector[3];
                    }
                }
            }
            else if (alpha < M_PI) {
                if (beta < M_PI_2) {
                    std::tie(t_lsl, p_lsl, q_lsl) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 0);
                    std::tie(t_rsl, p_rsl, q_rsl) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 2);
                    if (p_lsl - p_rsl - 2 * t_rsl + circumference < 0) {
                        t = t_lsl;
                        p = p_lsl;
                        q = q_lsl;
                        shortest_path_phis = path_phi_vector[0];
                    }
                    else {
                        t = t_rsl;
                        p = p_rsl;
                        q = q_rsl;
                        shortest_path_phis = path_phi_vector[2];
                    }
                }
                else if (beta < M_PI) {
                    std::tie(t_rsl, p_rsl, q_rsl) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 2);
                    if (alpha > beta) {
                        std::tie(t_lsl, p_lsl, q_lsl) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 0);
                        if (p_lsl - p_rsl - 2 * t_rsl + circumference < 0) {
                            t = t_lsl;
                            p = p_lsl;
                            q = q_lsl;
                            shortest_path_phis = path_phi_vector[0];
                        }
                        else {
                            t = t_rsl;
                            p = p_rsl;
                            q = q_rsl;
                            shortest_path_phis = path_phi_vector[2];
                        }
                    }
                    else {
                        std::tie(t_rsr, p_rsr, q_rsr) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 3);
                        if (p_rsr - p_rsl - 2 * q_rsl + circumference < 0) {
                            t = t_rsr;
                            p = p_rsr;
                            q = q_rsr;
                            shortest_path_phis = path_phi_vector[3];
                        }
                        else {
                            t = t_rsl;
                            p = p_rsl;
                            q = q_rsl;
                            shortest_path_phis = path_phi_vector[2];
                        }
                    }
                }
                else if (beta < M_3PI_2) {
                    std::tie(t, p, q) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 3);
                    shortest_path_phis = path_phi_vector[3];
                }
                else {
                    std::tie(t_rsr, p_rsr, q_rsr) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 3);
                    if (q_rsr - semicircumference < 0) {
                        t = t_rsr;
                        p = p_rsr;
                        q = q_rsr;
                        shortest_path_phis = path_phi_vector[3];
                    }
                    else {
                        t = t_rsl;
                        p = p_rsl;
                        q = q_rsl;
                        shortest_path_phis = path_phi_vector[2];
                    }
                }
            }
            else if (alpha < M_3PI_2) {
                if (beta < M_PI_2) {
                    std::tie(t_lsl, p_lsl, q_lsl) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 0);
                    if (q_lsl - semicircumference < 0) {
                        t = t_lsl;
                        p = p_lsl;
                        q = q_lsl;
                        shortest_path_phis = path_phi_vector[0];
                    }
                    else {
                        t = t_lsr;
                        p = p_lsr;
                        q = q_lsr;
                        shortest_path_phis = path_phi_vector[1];
                    }
                }
                else if (beta < M_PI) {
                    std::tie(t, p, q) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 0);
                    shortest_path_phis = path_phi_vector[0];
                }
                else if (beta < M_3PI_2) {
                    std::tie(t_lsr, p_lsr, q_lsr) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 1);
                    if (alpha < beta) {
                        std::tie(t_rsr, p_rsr, q_rsr) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 3);
                        if (p_rsr - p_lsr - 2 * t_lsr + circumference < 0) {
                            t = t_rsr;
                            p = p_rsr;
                            q = q_rsr;
                            shortest_path_phis = path_phi_vector[3];
                        }
                        else {
                            t = t_lsr;
                            p = p_lsr;
                            q = q_lsr;
                            shortest_path_phis = path_phi_vector[1];
                        }
                    }
                    else {
                        std::tie(t_lsl, p_lsl, q_lsl) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 0);
                        if (p_lsl - p_lsr - 2 * q_lsr + circumference < 0) {
                            t = t_lsl;
                            p = p_lsl;
                            q = q_lsl;
                            shortest_path_phis = path_phi_vector[0];
                        }
                        else {
                            t = t_lsr;
                            p = p_lsr;
                            q = q_lsr;
                            shortest_path_phis = path_phi_vector[1];
                        }
                    }
                }
                else {
                    std::tie(t_lsr, p_lsr, q_lsr) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 1);
                    std::tie(t_rsr, p_rsr, q_rsr) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 3);
                    if (p_rsr - p_lsr - 2 * t_lsr + circumference < 0) {
                        t = t_rsr;
                        p = p_rsr;
                        q = q_rsr;
                        shortest_path_phis = path_phi_vector[3];
                    }
                    else {
                        t = t_lsr;
                        p = p_lsr;
                        q = q_lsr;
                        shortest_path_phis = path_phi_vector[1];
                    }
                }
            }
            else {
                if (beta < M_PI_2) {
                    std::tie(t_lsl, p_lsl, q_lsl) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 0);
                    if (t_lsl - semicircumference > 0) {
                        t = t_rsl;
                        p = p_rsl;
                        q = q_rsl;
                        shortest_path_phis = path_phi_vector[2];
                    }
                    else if (q_lsl - semicircumference > 0) {
                        t = t_lsr;
                        p = p_lsr;
                        q = q_lsr;
                        shortest_path_phis = path_phi_vector[1];
                    }
                    else {
                        t = t_lsl;
                        p = p_lsl;
                        q = q_lsl;
                        shortest_path_phis = path_phi_vector[0];
                    }
                }
                else if (beta < M_PI) {
                    std::tie(t_lsl, p_lsl, q_lsl) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 0);
                    if (t_lsl - semicircumference < 0) {
                        t = t_lsl;
                        p = p_lsl;
                        q = q_lsl;
                        shortest_path_phis = path_phi_vector[0];
                    }
                    else {
                        t = t_rsl;
                        p = p_rsl;
                        q = q_rsl;
                        shortest_path_phis = path_phi_vector[2];
                    }
                }
                else if (beta < M_3PI_2) {
                    std::tie(t_lsl, p_lsl, q_lsl) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 0);
                    std::tie(t_lsr, p_lsr, q_lsr) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 1);
                    if (p_lsl - p_lsr - 2 * q_lsr + circumference < 0) {
                        t = t_lsl;
                        p = p_lsl;
                        q = q_lsl;
                        shortest_path_phis = path_phi_vector[0];
                    }
                    else {
                        t = t_lsr;
                        p = p_lsr;
                        q = q_lsr;
                        shortest_path_phis = path_phi_vector[1];
                    }
                }
                else {
                    std::tie(t, p, q) = path_length(wheelbase, r_vector, Delta_x_b, Delta_y_b, Delta_theta_total, circumference, 1);
                    shortest_path_phis = path_phi_vector[1];
                }
            }
            shortest_path_times = {t * u_v_inverse, p * u_v_inverse, q * u_v_inverse};
            
            return {shortest_path_phis, shortest_path_times};
        }

        std::tuple<std::vector<double>, std::vector<std::vector<double>>> steer(const std::vector<double>& q_nearest, const std::vector<double>& q_random, const int& number_of_steps, const double& distance_step, const double& distance_step_times_wheelbase_inverse, const std::vector<double>& U_phi, const int& U_phi_size) {
            int i, index, j;
            std::vector<double> distances, q_new;
            std::vector<std::vector<double>> steered_edge, theta, x, y;

            distances = std::vector<double>(U_phi_size, 0);
            x = std::vector<std::vector<double>>(U_phi_size, std::vector<double>(number_of_steps, 0));
            y = std::vector<std::vector<double>>(U_phi_size, std::vector<double>(number_of_steps, 0));
            theta = std::vector<std::vector<double>>(U_phi_size, std::vector<double>(number_of_steps, 0));
            for (i = 0; i < U_phi_size; i++) {
                x[i][0] = q_nearest[0];
                y[i][0] = q_nearest[1];
                theta[i][0] = q_nearest[2];
                for (j = 0; j < number_of_steps - 1; j++) {
                    x[i][j + 1] = x[i][j] + distance_step * std::cos(theta[i][j]);
                    y[i][j + 1] = y[i][j] + distance_step * std::sin(theta[i][j]);
                    theta[i][j + 1] = theta[i][j] + distance_step_times_wheelbase_inverse * std::tan(U_phi[i]);
                }
                distances[i] = std::sqrt(std::pow(q_random[0] - x[i].back(), 2) + std::pow(q_random[1] - y[i].back(), 2));
            }
            index = std::min_element(distances.begin(), distances.end()) - distances.begin();
            q_new = {x[index].back(), y[index].back(), theta[index].back()};
            steered_edge = {x[index], y[index], theta[index]};
            
            return {q_new, steered_edge};
        }

        std::vector<std::vector<std::vector<double>>> tag_obstacle_generator(const std::vector<std::vector<double>>& tag_configurations, const int& number_of_tags, const double& tag_obstacle_length, const double& tag_obstacle_width) {
            int i;
            double tag_obstacle_half_length, tag_obstacle_half_width;
            std::vector<std::vector<double>> tag_obstacle;
            std::vector<std::vector<std::vector<double>>> tag_obstacles;
            
            tag_obstacle_half_length = tag_obstacle_length * 0.5;
            tag_obstacle_half_width = tag_obstacle_width * 0.5;
            tag_obstacle = {{-tag_obstacle_half_width, -tag_obstacle_half_length}, {tag_obstacle_half_width, -tag_obstacle_half_length}, {tag_obstacle_half_width, tag_obstacle_half_length}, {-tag_obstacle_half_width, tag_obstacle_half_length}};
            tag_obstacles = std::vector<std::vector<std::vector<double>>>(number_of_tags, std::vector<std::vector<double>>(4, std::vector<double>(2, 0)));
            for (i = 0; i < number_of_tags; i++) {
                tag_obstacles[i] = translate_obstacle(rotate_obstacle(tag_obstacle, tag_configurations[i][2]), tag_configurations[i][0], tag_configurations[i][1]);
            }

            return tag_obstacles;
        }

        std::vector<std::vector<double>> translate_obstacle(const std::vector<std::vector<double>>& tag_obstacle, const double& x, const double& y) {
            int i;
            std::vector<std::vector<double>> tag_obstacle_translated;

            tag_obstacle_translated = std::vector<std::vector<double>>(4, std::vector<double>(2, 0));
            for (i = 0; i < 4; i++) {
                tag_obstacle_translated[i][0] = tag_obstacle[i][0] + x;
                tag_obstacle_translated[i][1] = tag_obstacle[i][1] + y;
            }

            return tag_obstacle_translated;
        }

        int van_der_corput_sequence(const int& length, const int& i) {
            int i_new, index;
            double b, s;
            
            i_new = i;
            b = 0.5;
            s = 0;
            while (i_new > 0) {
                s += (i_new % 2) * b;
                i_new *= 0.5;
                b *= 0.5;
            }
            index = static_cast<int> (s * length);
            
            return index;
        }
};

// define main function
int main(int argc, char** argv) {
    // initialise node
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh("~");
    PlanningNode planning_node = PlanningNode(&nh);
    ros::spin();
    
    return 0;
}