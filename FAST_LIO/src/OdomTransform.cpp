#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

class OdomTransform : public rclcpp::Node {
public:
    OdomTransform() : Node("odom_transformer"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&OdomTransform::cb_save_cur_odom, this, std::placeholders::_1));

        sub_map2odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/map2odom", 10, std::bind(&OdomTransform::cb_save_cur_map2odom, this, std::placeholders::_1));
    }

    void cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 转换并储存odom2body变换
        tf2::Transform transform_odom2body = odomMsgToTransform(*msg);
        tf_buffer_.setTransform(odomToTransformStamped(*msg, "odom", "body"), "default_authority", false);
    }

    void cb_save_cur_map2odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 转换并储存map2odom变换
        tf2::Transform transform_map2odom = odomMsgToTransform(*msg);
        tf_buffer_.setTransform(odomToTransformStamped(*msg, "map", "odom"), "default_authority", false);

        // 计算map到body的变换
        computeMapToBody();
    }

    void computeMapToBody() {
        geometry_msgs::msg::TransformStamped transform_map2body;
        try {
            transform_map2body = tf_buffer_.lookupTransform("map", "body", tf2::TimePointZero);
            RCLCPP_INFO(this->get_logger(), "Map to Body Transform: [%f, %f, %f]",
                        transform_map2body.transform.translation.x,
                        transform_map2body.transform.translation.y,
                        transform_map2body.transform.translation.z);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }

private:
    tf2::Transform odomMsgToTransform(const nav_msgs::msg::Odometry &odom_msg) {
        tf2::Transform transform;
        tf2::fromMsg(odom_msg.pose.pose, transform);
        return transform;
    }

    geometry_msgs::msg::TransformStamped odomToTransformStamped(const nav_msgs::msg::Odometry &odom_msg, 
                                                                const std::string &frame_id, 
                                                                const std::string &child_frame_id) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header = odom_msg.header;
        transform_stamped.child_frame_id = child_frame_id;
        transform_stamped.transform.translation.x = odom_msg.pose.pose.position.x;
        transform_stamped.transform.translation.y = odom_msg.pose.pose.position.y;
        transform_stamped.transform.translation.z = odom_msg.pose.pose.position.z;
        transform_stamped.transform.rotation = odom_msg.pose.pose.orientation;
        return transform_stamped;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry, sub_map2odom;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
