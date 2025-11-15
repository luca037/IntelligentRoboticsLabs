#include <memory>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/refill_burrow.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Remove the walls from the input image, return the image without the
// walls. Walls are assumed to be straight lines (vertical or horizontal).
// To do so it uses hugh lines transform.
cv::Mat remove_walls(const cv::Mat img) {
    // Create the output image.
    cv::Mat output = img.clone();

    // Detect straight lines.
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(
        img,
        lines,
        1,         // Rho 
        CV_PI/180, // Theta
        8,        // Threshold
        0,
        0
    );
 
    // Filter detected lines, keep only vetical and horizontal ones.
    for(const cv::Vec2f &line : lines) {
        float rho = line[0];
        float theta = line[1];

        // From rad to degree.
        float deg = 180 * theta / CV_PI;


        // Remove vertical and horizontal lines.
        if ((deg == 0) || (deg == 90)) {

            // Define two points on the line.
            cv::Point pt1, pt2;
            double a = std::cos(theta), b = std::sin(theta);
            double x0 = a * rho, y0 = b * rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));

            // Remove line.
            cv::line(output, pt1, pt2, cv::Scalar(0), cv::LINE_AA);
        }
    }

    return output;
}


// Detects circles from the image givn in input. Returns a new image with
// the circles highlighted. In 'centers' it stores the centers.
cv::Mat hough_circles(const cv::Mat &img, std::vector<cv::Point2f> &centers) {
    // Create the output image.
    cv::Mat output;
    cv::cvtColor(img, output, cv::COLOR_GRAY2BGR);

    // Detect circles.
    std::vector<cv::Vec4f> circles;
    cv::HoughCircles(
            img, 
            circles,
            cv::HOUGH_GRADIENT, 
            1,     // dp
            50,   // Min distance between centers.
            100,   // Canny (higher) threshold.
            6,     // The smaller it is, 
                   // the more false circles 
                   // may be detected. 
            3, 15  // Min and max radious.
    );

    // Draw circles.
    for(const cv::Vec4f &circle : circles) {
        if (circle[3] <=2 ) continue;

        cv::Point center = cv::Point(circle[0], circle[1]);
        // Draw center.
        cv::circle(output, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // Draw circle.
        int radius = circle[2];
        cv::circle(output, center, radius, cv::Scalar(255,0,255), 2, cv::LINE_AA);
        // Save center.
        centers.push_back(cv::Point2f(center.x, center.y));
    }

    return output;
}


// Tranform polar to cartesian.
cv::Point2f polar2cartesian(float rho, float theta) {
    float x = rho * std::cos(theta);
    float y = rho * std::sin(theta);
    return {x, y};
}


// Transform cartesian to pixel.
cv::Point cartesian2pixel(
    const cv::Point2f point,  
    const cv::Point2f center, 
    float scale
) {

    // Scale opration.
    int px_x = int(point.x * scale);
    int px_y = int(point.y * scale);

    // Translate the coordinate system: 
    //      image origin (0,0) is top-left, lidar (0,0) is 'center'.
    // Also need to invert y axis:
    //      image y is downwards, rviz is upwards.
    px_x = int(center.x + px_x);
    px_y = int(center.y - px_y); // INVERT Y-AXIS!

    return {px_x, px_y};
}


// Transform cartesian to pixel.
cv::Point2f pixel2cartesian(
    const cv::Point2f pixel,  
    const cv::Point2f center, 
    float scale
) {

    // Invert translation:
    // (recall: y axis is inverted).
    float x = float(pixel.x) - center.x;
    float y = -1. * (float(pixel.y) - center.y);

    // Invert scale:
    x /= scale;
    y /= scale;

    return {x, y};
}


// Turtlebot server class.
class TurtlebotServer : public rclcpp::Node {

 public:
    using RefillBurrow = interfaces::srv::RefillBurrow;
    using LaserScanMsg = sensor_msgs::msg::LaserScan;
    using PoseArrayMsg = geometry_msgs::msg::PoseArray;

    TurtlebotServer() 
        : Node("turtlebot_server"),
          found_apples_count_(0)
    {
        // Init the service server.
        service_ = this->create_service<RefillBurrow>(
            "refill_burrow",
            std::bind(
                &TurtlebotServer::handle_burrow_request,
                this, 
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        
        RCLCPP_INFO(
            this->get_logger(),
            "Turtlebot Service Server ready."
        );

        // Subscribe to /scan topic.
        scan_sub_ =
            this->create_subscription<LaserScanMsg>(
                "/scan",
                10,
                std::bind(
                    &TurtlebotServer::lidar_callback,
                    this,
                    std::placeholders::_1
                )
            );

        // Init publisher to /apples topic.
        apple_pub_ = 
            this->create_publisher<PoseArrayMsg>(
                "/apples", 10
            );
    }

 private:
    size_t found_apples_count_;
    rclcpp::Service<RefillBurrow>::SharedPtr service_;
    rclcpp::Subscription<LaserScanMsg>::SharedPtr scan_sub_;
    rclcpp::Publisher<PoseArrayMsg>::SharedPtr apple_pub_;

    void handle_burrow_request(
        const std::shared_ptr<RefillBurrow::Request>  request,
              std::shared_ptr<RefillBurrow::Response> response
    ) {
        RCLCPP_INFO(
            this->get_logger(), 
            "Burrow state: n=%d (current apples), s=%d (burrow size)",
            request->n, request->s
        );

        size_t apples_needed = request->s - request->n;
        
        if (found_apples_count_ >= apples_needed) {
            response->success = true;
            RCLCPP_INFO(
                this->get_logger(), 
                "Found %lu apples. Needs %lu. Success: TRUE.",
                found_apples_count_, apples_needed
            );
        } else {
            response->success = false;
            RCLCPP_INFO(
                this->get_logger(), 
                "Found %lu apples. Needs %lu. Success: FALSE.",
                found_apples_count_, apples_needed
            );
        }
    }

    // Detect apples using scan datas.
    void lidar_callback(const LaserScanMsg::SharedPtr msg) {

        // Size of the image in which detection is performed.
        static const int MAP_SIZE_PX = 1000; 

        // Calculate the scale factor (pixels per meter).
        // Cannot use 1:1 because otherwise the image is to small
        // (low resolution).
        static const float SCALE_FACTOR = 
            (MAP_SIZE_PX / 0.4) / msg->range_max;

        // Image in which lidar scan is recreated.
        cv::Mat map(MAP_SIZE_PX, MAP_SIZE_PX, CV_8UC1, cv::Scalar(0));

        // Define robot position => middle point.
        // This is used for the transformations:
        //      cartesian coordinate <-> pixel.
        static const cv::Point2f
            img_center(MAP_SIZE_PX / 2., MAP_SIZE_PX / 2.);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            // Compute angle.
            float theta = msg->angle_min + (i * msg->angle_increment);

            // Get point in cartesian coordinates.
            cv::Point2f point = 
                polar2cartesian(msg->ranges[i], theta);

            cv::Point pixel =
                cartesian2pixel(point, img_center, SCALE_FACTOR);

            // Check bounds of the pixel point.
            if ((pixel.x >= 0 && pixel.x < MAP_SIZE_PX) && 
                (pixel.y >= 0 && pixel.y < MAP_SIZE_PX)
            ) {
                map.at<uchar>(pixel) = 255; 
            }
        }

        // Remove the walls.
        map = remove_walls(map);

        // Find circles aka apples.
        std::vector<cv::Point2f> centers;
        map = hough_circles(map, centers);

        // Update detected apples.
        found_apples_count_ = centers.size();

        // Display the map image with detected apples.
        cv::namedWindow("LIDAR Scan Plot", cv::WINDOW_NORMAL);
        cv::imshow("LIDAR Scan Plot", map);
        cv::waitKey(1);

        // Back transform from pixel to base frame.
        for (cv::Point2f &point : centers) {
            point = pixel2cartesian(
                point,
                img_center,
                SCALE_FACTOR
            );
        }

        // Publish data.
        apple_callback(centers);
    }

    // Publishes data into /apple topic.
    void apple_callback(const std::vector<cv::Point2f> &centers) {
        // Create the array of poses.
        PoseArrayMsg apple_poses;
        apple_poses.header.frame_id = "base_scan";
        apple_poses.header.stamp = this->now();

        for (const auto& point : centers) {
            geometry_msgs::msg::Pose apple_pose;
            
            // Set position.
            apple_pose.position.x = point.x;
            apple_pose.position.y = point.y;
            apple_pose.position.z = 0.0;
            
            // Set orientation.
            apple_pose.orientation.x = 0.0;
            apple_pose.orientation.y = 0.0;
            apple_pose.orientation.z = 0.0;
            apple_pose.orientation.w = 1.0; 
            
            apple_poses.poses.push_back(apple_pose);
        }

        apple_pub_->publish(apple_poses);
    }

};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotServer>());
    rclcpp::shutdown();
    return 0;
}
