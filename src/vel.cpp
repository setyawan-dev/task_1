#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <deque>
#include <numeric>
#include <cmath>

class VelocityController {
public:
    VelocityController() {
        ros::NodeHandle nh;
        pub_kiri = nh.advertise<std_msgs::Float64>("/motor_kiri_joint/command", 10);
        pub_kanan = nh.advertise<std_msgs::Float64>("/motor_kanan_joint/command", 10);
        pub_vel_kiri = nh.advertise<std_msgs::Float64>("/motor_kiri_controller/command", 10);
        pub_vel_kanan = nh.advertise<std_msgs::Float64>("/motor_kanan_controller/command", 10);
        pub_pivot = nh.advertise<std_msgs::Float64>("/pivot_joint_controller/command", 10);
        pub_gyro = nh.advertise<std_msgs::Float32>("/gyro/data", 10);
        sub_vel = nh.subscribe("/vel/cmd", 10, &VelocityController::VelCb, this);
        sub_imu = nh.subscribe("/imu/data", 10, &VelocityController::ImuCb, this);
        alpha = 0.05;
        buffer_size = 20;
        last_pitch = 0.0;
        pivot_angle = 0.0;
        konstanta_thrust = 0.5;
        noise_threshold = 0.02;
        pitch_offset_set = false;
        ROS_INFO("Thrust Controller Aktif");
    }

    void spin() {
        ros::spin();
    }

private:
    ros::Publisher pub_kiri, pub_kanan, pub_vel_kiri, pub_vel_kanan, pub_pivot, pub_gyro;
    ros::Subscriber sub_vel, sub_imu;

    double alpha, last_pitch, pivot_angle, konstanta_thrust, noise_threshold;
    int buffer_size;
    std::deque<double> pitch_raw;
    double pitch_offset;
    bool pitch_offset_set;

    void ImuCb(const sensor_msgs::Imu::ConstPtr& msg) {
        tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        if (!pitch_offset_set) {
            pitch_offset = pitch;
            pitch_offset_set = true;
        }
        double pitch_rel = pitch - pitch_offset;
        pitch_raw.push_back(pitch_rel);
        if (pitch_raw.size() > buffer_size) {
            pitch_raw.pop_front();
        }
        last_pitch = alpha * pitch_rel + (1.0 - alpha) * last_pitch;
        double filtered_pitch = last_pitch;
        if (pitch_raw.size() >= 5) {
            filtered_pitch = std::accumulate(pitch_raw.end() - 5, pitch_raw.end(), 0.0) / 5.0;
        }
        if (std::abs(filtered_pitch) < noise_threshold) {
            filtered_pitch = 0.0;
        }
        std_msgs::Float32 msg_pitch;
        msg_pitch.data = filtered_pitch;
        pub_gyro.publish(msg_pitch);
    }

    void VelCb(const geometry_msgs::Twist::ConstPtr& msg) {
        double vel_left = msg->linear.x;
        double vel_right = msg->angular.x;
        double thrust_kiri = konstanta_thrust * vel_left * vel_left;
        double thrust_kanan = konstanta_thrust * vel_right * vel_right;
        if (vel_left > vel_right) {
            thrust_kiri *= 1.2;
        } else if (vel_right > vel_left) {
            thrust_kanan *= 1.2;
        }
        double delta_thrust = thrust_kanan - thrust_kiri;
        double threshold = 0.5;
        double pivot_gain = 0.005;
        if (std::abs(delta_thrust) < threshold) {
            pivot_angle = 0.0;
        } else {
            pivot_angle = pivot_gain * delta_thrust;
        }
        std_msgs::Float64 kiri_msg, kanan_msg, vel_kiri_msg, vel_kanan_msg, pivot_msg;
        kiri_msg.data = thrust_kiri;
        kanan_msg.data = thrust_kanan;
        vel_kiri_msg.data = vel_left;
        vel_kanan_msg.data = vel_right;
        pivot_msg.data = pivot_angle;
        pub_kiri.publish(kiri_msg);
        pub_kanan.publish(kanan_msg);
        pub_vel_kiri.publish(vel_kiri_msg);
        pub_vel_kanan.publish(vel_kanan_msg);
        pub_pivot.publish(pivot_msg);
        ROS_INFO("\n[DATA] :\n Kiri: %.2f\n Thrust: %.2f\n Kanan: %.2f\n Thrust: %.2f\n Pivot: %.4f\n",
                 vel_left, thrust_kiri, vel_right, thrust_kanan, pivot_angle);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_control");
    VelocityController vc;
    vc.spin();
    return 0;
}
