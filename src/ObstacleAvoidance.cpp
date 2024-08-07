#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>



class ObstacleAvoidance
{
public:
    ObstacleAvoidance()
    {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        laser_sub_ = nh_.subscribe("/scan", 10, &ObstacleAvoidance::laserCallback, this);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        // 获取激光雷达数据
        std::vector<float> ranges = msg->ranges;
        // 计算左侧/右侧/前方的障碍物距离
        float left_distance = *std::min_element(ranges.begin(), ranges.begin() + 90);
        float right_distance = *std::min_element(ranges.end() - 90, ranges.end());
        float front_distance = std::min(*std::min_element(ranges.begin() + 270, ranges.end()), *std::min_element(ranges.begin(), ranges.begin() + 90));
        // 根据障碍物距离调整小车速度和转向
        if (front_distance < 0.5)
        {
            twist_msg_.linear.x = 0;
            twist_msg_.angular.z = 0.5;
        }
        else if (left_distance < right_distance)
        {
            twist_msg_.linear.x = 0.5;
            twist_msg_.angular.z = -0.5;
        }
        else
        {
            twist_msg_.linear.x = 0.5;
            twist_msg_.angular.z = 0.5;
        }
        vel_pub_.publish(twist_msg_);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber laser_sub_;
    geometry_msgs::Twist twist_msg_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoidance");
    ObstacleAvoidance obstacle_avoidance;
    ros::spin();
    return 0;
}