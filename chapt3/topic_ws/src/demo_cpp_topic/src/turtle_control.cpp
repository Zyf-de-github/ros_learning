#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;



class TurtlrControlNode: public rclcpp::Node
{
private:
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    double target_x_{1.0};
    double target_y_{1.0};
    double k_{1.0};
    double max_speed_{3.0};



public:
    explicit TurtlrControlNode(const std::string& node_name):Node(node_name)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&TurtlrControlNode::on_pose_received_,this,std::placeholders::_1));
        //timer_ = this->create_wall_timer(1000ms,std::bind(&TurtlrControlNode::timer_callback,this));
    }

    // void timer_callback()
    // {

    //     auto msg = geometry_msgs::msg::Twist();
    //     msg.linear.x=1.0;
    //     msg.angular.z=1.0;        
    //     publisher_->publish(msg);
    // }

    void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose)
    {
        auto current_x = pose->x;
        auto current_y = pose->y;
        RCLCPP_INFO(get_logger(),"current pose: x=%f, y=%f",current_x,current_y);


        auto distance =std::sqrt((target_x_-current_x)*(target_x_-current_x)+(target_y_-current_y)*(target_y_-current_y));
        auto angle = std::atan2((target_y_-current_y),(target_x_-current_x))-pose->theta;

        auto msg = geometry_msgs::msg::Twist();
        
        if(distance>0.1)
        {
            if(fabs(angle)>0.2)
            {
                msg.angular.z=fabs(angle);
            }
            else
            {
                if(fabs(k_*distance)>=max_speed_)
                    msg.linear.x = max_speed_;
                else
                    msg.linear.x = k_ * distance;
            }
        }

        publisher_->publish(msg);        

    }

};


int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtlrControlNode>("turtle_control");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}