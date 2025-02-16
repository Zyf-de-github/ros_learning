#include <QApplication>
#include <QLabel>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <status_interfaces/msg/system_status.hpp>

using SystemStatus = status_interfaces::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node
{
private:
    rclcpp::Subscription<SystemStatus>::SharedPtr subscriber_;
    QLabel *label_;
public:
    SysStatusDisplay():Node("sys_status_display")
    {
        label_ = new QLabel();
        subscriber_ = this->create_subscription<SystemStatus>("sys_status",10,[&]
        (const SystemStatus::SharedPtr msg)->void
        {
            label_->setText(get_qstr_from_msg(msg));
        });
        label_->setText(get_qstr_from_msg(std::make_shared<SystemStatus>()));
        label_->show();
    };
        // msg.stamp = self.get_clock().now().to_msg()
        // msg.host_name = platform.node()
        // msg.cpu_percent = cpu_percent
        // msg.memory_percent = memory_info.percent
        // msg.memory_total = memory_info.total /1024 /1024
        // msg.memory_available = memory_info.available /1024 /1024
        // msg.net_sent = net_to_counters.bytes_sent /1024 /1024
        // msg.net_recv = net_to_counters.bytes_recv / 1024 / 1024

    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg)
    {
        std::stringstream show_str;
        show_str << "============系统状态可视化显示工具=========\n"
                 << "数据时间  :\t" << msg->stamp.sec << "\ts\n"
                 << "主机名字  :\t" << msg->host_name << "\t\n"
                 << "cpu使用率 :\t" << msg->cpu_percent << "\t%\n"
                 << "内存使用率 :\t" << msg->memory_percent << "\t%\n"
                 << "内存总大小 :\t" << msg->memory_total<< "\tMB\n"
                 << "剩余内存   :\t" << msg->memory_available << "\tMB\n"
                 << "网络发送量 :\t" << msg->net_sent << "\tMB\n"
                 << "网络接收量 :\t" << msg->net_recv << "\tMB\n"
                 << "========================================\n"
                 ;
        return QString::fromStdString(show_str.str());
    };
};




int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    QApplication app(argc,argv);
    auto node = std::make_shared<SysStatusDisplay>();
    std::thread spin_thread([&]()->void
    {
        rclcpp::spin(node);
    });
    
    spin_thread.detach();
    app.exec();
    return 0;
}