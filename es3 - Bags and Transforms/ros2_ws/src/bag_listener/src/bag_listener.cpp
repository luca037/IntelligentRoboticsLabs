#include <memory.h>
#include <fstream>

#include "rclcpp/rclcpp.hpp" 

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/convert.hpp"

class BagListener : public rclcpp::Node {

public:
    BagListener() 
    : Node("bag_listener"),
      out_csv_("out.csv") {
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Check if csv output is open.
        if (out_csv_.is_open())
            out_csv_ << "x_vc,y_vc,x_cs,y_cs,diff_x,diff_y" << std::endl;
        else
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open CSV file for writing!"
            );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&BagListener::topic_callback, this)
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::ofstream out_csv_;

    const std::string floor_frame_ = "tag36h11:0";
    const std::string cs_frame_ = "tag36h11:1";
    const std::string vc_frame_ = "tag36h11:2";

    void topic_callback() {
        auto transform_vc_floor = 
            this->tf_buffer_->lookupTransform(
                floor_frame_, // Target frame.
                vc_frame_,    // Source frame.
                tf2::TimePointZero,
                std::chrono::seconds(1) // Wait up to 1 sec.
            );

        auto transform_cs_floor = 
            this->tf_buffer_->lookupTransform(
                floor_frame_, // Target frame.
                cs_frame_,    // Source frame.
                tf2::TimePointZero,
                std::chrono::seconds(1) // Wait up to 1 sec.
            );

        // Point in vc.
        geometry_msgs::msg::PointStamped pt_in_vc;
        pt_in_vc.header.frame_id = vc_frame_;
        pt_in_vc.header.stamp = transform_vc_floor.header.stamp;

        // Point in cs.
        geometry_msgs::msg::PointStamped pt_in_cs;
        pt_in_cs.header.frame_id = vc_frame_;
        pt_in_cs.header.stamp = transform_cs_floor.header.stamp;

        // Tranform points.
        geometry_msgs::msg::PointStamped pt_in_floor1, pt_in_floor2;
        tf2::doTransform(pt_in_vc, pt_in_floor1, transform_vc_floor);
        tf2::doTransform(pt_in_cs, pt_in_floor2, transform_cs_floor);

        // vc position w.r.t. cs.
        double diff_x = pt_in_floor1.point.x - pt_in_floor2.point.x;
        double diff_y = pt_in_floor1.point.y - pt_in_floor2.point.y;

        // Save to output csv file.
        if (out_csv_.is_open()) {
            out_csv_ << pt_in_floor1.point.x << "," 
                << pt_in_floor1.point.y << ","
                << pt_in_floor2.point.x << "," 
                << pt_in_floor2.point.y << ","
                << diff_x << "," << diff_y << std::endl;
        }

        // Log tranformed point.
        RCLCPP_INFO(this->get_logger(),
            "\n\tpt_in_floor1: x=%.3f y=%.3f z=%.3f"
            "\n\tpt_in_floor2: x=%.3f y=%.3f z=%.3f",
            pt_in_floor1.point.x,
            pt_in_floor1.point.y,
            pt_in_floor1.point.z,
            pt_in_floor2.point.x,
            pt_in_floor2.point.y,
            pt_in_floor2.point.z
        );
    }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BagListener>());
    rclcpp::shutdown();
    return 0;
}
