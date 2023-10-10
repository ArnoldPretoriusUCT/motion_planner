// include standard C header files
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

// include standard C++ header files
#include <algorithm>
#include <boost/bind.hpp>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

// define node class
class ApriltagDetectorNode {
    // declare standard attributes
    bool camera_info_called, initialised;
    int baseline, counter, i, index, number_of_detections, number_of_tags;
    double q_tag_b_x, q_tag_b_y, tag_size;
    std::vector<double> bearing_estimate_vector, error_vector, range_estimate_vector, tag_id_vector, tag_vector;
    std::vector<std::vector<double>> tags;
    std::stringstream ss;
    apriltag_detection_t* detection_ptr;
    apriltag_detection_info_t detection_info;
    apriltag_detector_t* detector_ptr;
    apriltag_family_t* family_ptr;
    apriltag_pose_t q_tag_b;
    zarray_t* detections_ptr;
    cv::Mat D, K, undistorted_image;
    cv::Size bearing_estimate_text_size, name_text_size, range_estimate_text_size;
    cv::String bearing_estimate_text, name_text, range_estimate_text;
    cv_bridge::CvImagePtr cv_image_ptr;
    
    // declare ROS attributes
    ros::Publisher pub_apriltag_measurement;
    ros::Subscriber sub_camera_info, sub_initialise;
    image_transport::Publisher pub_image;
    image_transport::Subscriber sub_image;
    std_msgs::Float64MultiArray msg_apriltag_measurement;
    
    public:
        // define node constructor
        ApriltagDetectorNode(ros::NodeHandle* nh) {
            // initialise standard attributes
            camera_info_called = false;
            initialised = false;
            counter = 0;
            detector_ptr = apriltag_detector_create();
            family_ptr = tag36h11_create();
            apriltag_detector_add_family(detector_ptr, family_ptr);
            ss.precision(2);
            freopen("/dev/null", "w", stderr);

            // initialise ROS attributes
            pub_apriltag_measurement = nh->advertise<std_msgs::Float64MultiArray>("apriltag_measurement", 1);
            sub_camera_info = nh->subscribe("camera_info", 1, &ApriltagDetectorNode::camera_info_callback, this);
            sub_initialise = nh->subscribe<std_msgs::Bool>("initialise", 1, boost::bind(&ApriltagDetectorNode::initialise_callback, this, _1, nh));
            image_transport::ImageTransport it(*nh);
            image_transport::TransportHints hints("compressed");
            pub_image = it.advertise("image_out", 1);
            sub_image = it.subscribe("image", 1, &ApriltagDetectorNode::image_callback, this, hints);
        }

        // define callback methods
        void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg_camera_info) {
            if (initialised) {
                D = (cv::Mat1d(1, 5) << msg_camera_info->D[0], msg_camera_info->D[1], msg_camera_info->D[2], msg_camera_info->D[3], msg_camera_info->D[4]);
                K = (cv::Mat1d(3, 3) << msg_camera_info->K[0], msg_camera_info->K[1], msg_camera_info->K[2], msg_camera_info->K[3], msg_camera_info->K[4], msg_camera_info->K[5], msg_camera_info->K[6], msg_camera_info->K[7], msg_camera_info->K[8]);
                detection_info = {.tagsize = tag_size, .fx = msg_camera_info->K[0], .fy = msg_camera_info->K[4], .cx = msg_camera_info->K[2], .cy = msg_camera_info->K[5]};
                camera_info_called = true;
                sub_camera_info.shutdown();
            }
        }

        void initialise_callback(const std_msgs::BoolConstPtr& msg_initialise, ros::NodeHandle* nh) {
            // get parameter values
            nh->getParam("tag_size", tag_size);
            nh->getParam("tag_vector", tag_vector);

            // initialise standard attributes with parameter values
            number_of_tags = tag_vector.size() / 4;
            tags = std::vector<std::vector<double>>(number_of_tags, std::vector<double>(4, 0));
            for (i = 0; i < number_of_tags; i++) {
                tags[i] = {tag_vector[4 * i], tag_vector[4 * i + 1], tag_vector[4 * i + 2], tag_vector[4 * i + 3]};
            }
            initialised = true;
        }

        void image_callback(const sensor_msgs::ImageConstPtr& msg_image) {
            if (camera_info_called) {
                cv_image_ptr = cv_bridge::toCvCopy(msg_image, "mono8");
                cv::undistort(cv_image_ptr->image, undistorted_image, K, D);
                image_u8_t u8_image = {.width = undistorted_image.cols, .height = undistorted_image.rows, .stride = undistorted_image.cols, .buf = undistorted_image.data};
                detections_ptr = apriltag_detector_detect(detector_ptr, &u8_image);
                number_of_detections = zarray_size(detections_ptr);
                if (number_of_detections) {
                    error_vector = std::vector<double>(number_of_detections, 0);
                    tag_id_vector = std::vector<double>(number_of_detections, 0);
                    range_estimate_vector = std::vector<double>(number_of_detections, 0);
                    bearing_estimate_vector = std::vector<double>(number_of_detections, 0);
                    for (i = 0; i < number_of_detections; i++) {
                        zarray_get(detections_ptr, i, &detection_ptr);
                        tag_id_vector[i] = detection_ptr->id;
                        detection_info.det = detection_ptr;
                        error_vector[i] = estimate_tag_pose(&detection_info, &q_tag_b);
                        q_tag_b_x = q_tag_b.t->data[2] + 0.0582;
                        q_tag_b_y = -q_tag_b.t->data[0];
                        range_estimate_vector[i] = std::sqrt(q_tag_b_x * q_tag_b_x + q_tag_b_y * q_tag_b_y);
                        bearing_estimate_vector[i] = std::atan2(q_tag_b_y, q_tag_b_x);
                        ss << "Bearing: ~" << bearing_estimate_vector[i] * 180 / M_PI << " deg";
                        bearing_estimate_text = ss.str();
                        bearing_estimate_text_size = cv::getTextSize(bearing_estimate_text, cv::FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);
                        cv::putText(undistorted_image, bearing_estimate_text, cv::Point(detection_ptr->c[0] - bearing_estimate_text_size.width * 0.5, detection_ptr->c[1] - bearing_estimate_text_size.height * 0.5), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0xff, 0xff, 0xff), 2, cv::LINE_AA);
                        ss.str(std::string());
                        ss << "Range: ~" << range_estimate_vector[i] << " m";
                        range_estimate_text = ss.str();
                        range_estimate_text_size = cv::getTextSize(range_estimate_text, cv::FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);
                        cv::putText(undistorted_image, range_estimate_text, cv::Point(detection_ptr->c[0] - range_estimate_text_size.width * 0.5, detection_ptr->c[1] - bearing_estimate_text_size.height * 2 - range_estimate_text_size.height * 0.5), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0xff, 0xff, 0xff), 2, cv::LINE_AA);
                        ss.str(std::string());
                        if (detection_ptr->id == 20) {
                            name_text = "Goal tag";
                        }
                        else {
                            name_text = "Obstacle tag";
                        }
                        name_text_size = cv::getTextSize(name_text, cv::FONT_HERSHEY_SIMPLEX, 1.5, 2, &baseline);
                        cv::putText(undistorted_image, name_text, cv::Point(detection_ptr->c[0] - name_text_size.width * 0.5, detection_ptr->c[1] - bearing_estimate_text_size.height * 2 - range_estimate_text_size.height * 2 - name_text_size.height * 0.5), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0xff, 0xff, 0xff), 2, cv::LINE_AA);
                    }
                    index = std::min_element(error_vector.begin(), error_vector.end()) - error_vector.begin();
                    for (i = 0; i < number_of_tags; i++) {
                        if (tags[i][0] == tag_id_vector[index]) {
                            msg_apriltag_measurement.data = {range_estimate_vector[index], bearing_estimate_vector[index], tags[i][1], tags[i][2], tags[i][3]};
                            pub_apriltag_measurement.publish(msg_apriltag_measurement);
                            break;
                        }
                    }
                }
                pub_image.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", undistorted_image).toImageMsg());
            }
        }
};

// define main function
int main(int argc, char** argv) {
    // initialise node
    ros::init(argc, argv, "apriltag_detector_node");
    ros::NodeHandle nh("~");
    ApriltagDetectorNode apriltag_detector_node = ApriltagDetectorNode(&nh);
    ros::spin();
    
    return 0;
}
