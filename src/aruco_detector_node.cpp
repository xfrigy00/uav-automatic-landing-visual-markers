#include "rclcpp/rclcpp.hpp"          // ROS 2 C++ client library for creating and managing nodes
#include "sensor_msgs/msg/image.hpp"  // Includes the ROS message type for image data
#include "cv_bridge/cv_bridge.h"      // Bridges the conversion between ROS messages and OpenCV images
#include <opencv2/opencv.hpp>         // Includes OpenCV for general image processing
#include <opencv2/aruco.hpp>          // Includes OpenCV's ArUco library for detecting ArUco markers

// Define the class for ArUco marker detection, inheriting from rclcpp::Node
class ArucoDetector : public rclcpp::Node
{
public:
    // Constructor for the node
    ArucoDetector() : Node("aruco_detector_node")
    {
        // Subscribe to the ROS image topic "/image", with a queue size of 10
        // When a new image message is received, it triggers the `image_callback` function
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));
    }

private:
    // Callback function for processing incoming image messages
    void image_callback(sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert the incoming ROS image message to an OpenCV image format (BGR8 encoding)
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            // The toCvCopy method converts the image
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            // If an error occurs during conversion, log it and return from the function
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
            return;
        }

        // Load the predefined ArUco dictionary for marker detection (4x4 grid with 1000 markers)
        cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
        
        // Vectors to store detected marker information
        std::vector<std::vector<cv::Point2f>> corners;  // Marker corners
        std::vector<int> ids;  // Marker IDs
        std::vector<std::vector<cv::Point2f>> rejected_img_points;  // Points rejected by the detector

        // Create parameters for the marker detection algorithm
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

        // Perform marker detection on the OpenCV image
        cv::aruco::detectMarkers(cv_ptr->image, aruco_dict, corners, ids, detectorParams, rejected_img_points);

        // If any markers are detected, draw them on the image
        if (!ids.empty())
        {
            // Draw the detected markers with their IDs
            cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        }

        // Display the processed image with detected markers
        cv::imshow("Aruco Markers", cv_ptr->image);
        cv::waitKey(1);  // Wait briefly for a key press to update the image display
    }

    // ROS subscription to the image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
};

// Main function that initializes and runs the ROS node
int main(int argc, char *argv[])
{
    // Initialize the ROS system (this is required for any ROS 2 program)
    rclcpp::init(argc, argv);

    // Create an instance of the ArucoDetector node and start processing
    rclcpp::spin(std::make_shared<ArucoDetector>());

    // Shutdown ROS after the node finishes executing
    rclcpp::shutdown();
    return 0;
}

