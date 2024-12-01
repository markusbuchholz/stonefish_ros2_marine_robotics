#include <rclcpp/rclcpp.hpp>
#include <cola2_msgs/msg/dvl.hpp>
#include <cola2_msgs/msg/nav_sts.hpp>
#include <cola2_msgs/msg/setpoints.hpp>
#include <stonefish_ros2/msg/dvl.hpp>
#include <stonefish_ros2/msg/ins.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class Cola2StonefishNode : public rclcpp::Node
{
public:
    Cola2StonefishNode() : Node("cola2_stonefish_node")
    {
        dvl_pub_ = this->create_publisher<cola2_msgs::msg::DVL>("cola2_dvl", 10);
        ins_pub_ = this->create_publisher<cola2_msgs::msg::NavSts>("cola2_navigation", 10);
        th_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("stonefish_thruster_setpoints", 10);

        dvl_sub_ = this->create_subscription<stonefish_ros2::msg::DVL>(
            "stonefish_dvl", 10, std::bind(&Cola2StonefishNode::sfDVLCallback, this, std::placeholders::_1));
        ins_sub_ = this->create_subscription<stonefish_ros2::msg::INS>(
            "stonefish_navigation", 10, std::bind(&Cola2StonefishNode::sfINSCallback, this, std::placeholders::_1));
        th_sub_ = this->create_subscription<cola2_msgs::msg::Setpoints>(
            "cola2_thruster_setpoints", 10, std::bind(&Cola2StonefishNode::cola2SetpointsCallback, this, std::placeholders::_1));
    }

private:
    void sfDVLCallback(const stonefish_ros2::msg::DVL::SharedPtr in)
    {
        cola2_msgs::msg::DVL out;
        out.header = in->header;
        out.velocity = in->velocity;
        out.velocity_covariance = in->velocity_covariance;
        out.altitude = in->altitude;
        out.beams.resize(in->beams.size());
        for(size_t i = 0; i < in->beams.size(); ++i)
        {
            out.beams[i].pose = in->beams[i].pose;
            out.beams[i].range = in->beams[i].range;
            out.beams[i].range_covariance = in->beams[i].range_covariance;
            out.beams[i].velocity = in->beams[i].velocity;
            out.beams[i].velocity_covariance = in->beams[i].velocity_covariance;
        }
        dvl_pub_->publish(out);
    }

    void sfINSCallback(const stonefish_ros2::msg::INS::SharedPtr in)
    {
        cola2_msgs::msg::NavSts out;
        out.header = in->header;
        out.global_position.latitude = in->latitude;
        out.global_position.longitude = in->longitude;
        out.origin.latitude = in->origin_latitude;
        out.origin.longitude = in->origin_longitude;
        out.altitude = in->altitude;
        out.body_velocity.x = in->body_velocity.x;
        out.body_velocity.y = in->body_velocity.y;
        out.body_velocity.z = in->body_velocity.z;
        out.orientation_rate.roll = in->rpy_rate.x;
        out.orientation_rate.pitch = in->rpy_rate.y;
        out.orientation_rate.yaw = in->rpy_rate.z;
        out.position.north = in->pose.north;
        out.position.east = in->pose.east;
        out.position.depth = in->pose.down;
        out.orientation.roll = in->pose.roll;
        out.orientation.pitch = in->pose.pitch;
        out.orientation.yaw = in->pose.yaw;
        out.position_variance.north = in->pose_variance.north;
        out.position_variance.east = in->pose_variance.east;
        out.position_variance.depth = in->pose_variance.down;
        out.orientation_variance.roll = in->pose_variance.roll;
        out.orientation_variance.pitch = in->pose_variance.pitch;
        out.orientation_variance.yaw = in->pose_variance.yaw;
        ins_pub_->publish(out);
    }

    void cola2SetpointsCallback(const cola2_msgs::msg::Setpoints::SharedPtr in)
    {
        std_msgs::msg::Float64MultiArray out;
        out.layout.data_offset = 0;
        out.layout.dim.resize(1);
        out.layout.dim[0].label = "setpoints";
        out.layout.dim[0].size = in->setpoints.size();
        out.layout.dim[0].stride = in->setpoints.size();
        out.data = in->setpoints;
        th_pub_->publish(out);
    }

    rclcpp::Publisher<cola2_msgs::msg::DVL>::SharedPtr dvl_pub_;
    rclcpp::Publisher<cola2_msgs::msg::NavSts>::SharedPtr ins_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr th_pub_;

    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Subscription<stonefish_ros2::msg::INS>::SharedPtr ins_sub_;
    rclcpp::Subscription<cola2_msgs::msg::Setpoints>::SharedPtr th_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cola2StonefishNode>());
    rclcpp::shutdown();
    return 0;
}

