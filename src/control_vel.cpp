#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <string>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_speed_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/vel/cmd", 10);
    ros::Rate rate(10);

    std::string input;
    while (ros::ok())
    {
        std::cout << "Masukkan kecepatan (kiri kanan) atau 'stop' untuk keluar: ";
        std::getline(std::cin, input);

        if (input == "stop")
        {
            ROS_INFO("Program dihentikan oleh pengguna.");
            break;
        }

        std::istringstream iss(input);
        float kiri, kanan;
        if (!(iss >> kiri >> kanan))
        {
            ROS_WARN("Input tidak valid. Gunakan format: <kiri> <kanan> atau ketik 'stop'");
            continue;
        }

        geometry_msgs::Twist msg;
        msg.linear.x = kiri;
        msg.angular.x = kanan;

        pub.publish(msg);
        ROS_INFO("Dikirim - Kiri: %.2f, Kanan: %.2f", kiri, kanan);

        rate.sleep();
    }

    return 0;
}
